#include "ota.h"

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <mbedtls/sha256.h>
#include <Preferences.h>
#if defined(CONFIG_NIMBLE_CPP_IDF)
#include "services/gatt/ble_svc_gatt.h"
#else
#include "nimble/nimble/host/services/gatt/include/services/gatt/ble_svc_gatt.h"
#endif
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#ifndef FW_VERSION
#define FW_VERSION "dev"
#endif

namespace {

const char *OTA_SERVICE_UUID = "6e7a0010-3c1b-4b8a-9f2e-6b1d5a7c0de1";
const char *OTA_CTRL_UUID = "6e7a0011-3c1b-4b8a-9f2e-6b1d5a7c0de1";
const char *OTA_DATA_UUID = "6e7a0012-3c1b-4b8a-9f2e-6b1d5a7c0de1";
const char *OTA_INFO_UUID = "6e7a0013-3c1b-4b8a-9f2e-6b1d5a7c0de1";

enum : uint8_t { CMD_BEGIN = 0x01, CMD_END = 0x02, CMD_ABORT = 0x03, CMD_DATA = 0x80 };
enum : uint8_t { ST_READY = 0x10, ST_PROGRESS = 0x11, ST_DONE = 0x12, ST_ERROR = 0x1F };
enum : uint8_t {
  ERR_NONE = 0, ERR_BAD_CMD = 1, ERR_TOO_BIG = 2, ERR_BEGIN = 3, ERR_WRITE = 4,
  ERR_SIZE = 5, ERR_HASH = 6, ERR_IMAGE = 7, ERR_NOT_STARTED = 8, ERR_BOOT = 9,
  ERR_PROBATION = 10, ERR_TIMEOUT = 11
};

const uint32_t kHealthyAfterMs = 15000;   // a new image must run this long to be kept
const uint64_t kProbationUs = 60000000;   // ...and must get there within 60 s
// PROGRESS is also the flow control for the fast path (write without
// response): the PC keeps at most a window of unacknowledged bytes in flight,
// smaller than the queue below, so the queue never fills and the BLE host
// task never blocks.
const uint32_t kProgressEvery = 1024;
const UBaseType_t kQueueLen = 32;  // 32 x 512 B = 16 KB of buffered image data
// Give up on a transfer that stops sending data. Without this, an updater that
// is killed while sharing X-Plane's connection (so no disconnect ever arrives)
// would leave tracking paused until the tracker is power-cycled.
const TickType_t kIdleTimeout = pdMS_TO_TICKS(10000);

struct Msg {
  uint8_t type;
  uint16_t len;
  uint8_t data[512];
};

QueueHandle_t gQueue;
NimBLECharacteristic *gCtrl, *gInfo;
NimBLEServer *gServer;
volatile bool gActive = false;
volatile uint16_t gOtaConn = 0xFFFF;
volatile bool gPendingVerify = false;
esp_timer_handle_t gProbationTimer;
#if CONFIG_PM_ENABLE
esp_pm_lock_handle_t gNoSleep, gFullSpeed;
#endif

void status(uint8_t code, uint8_t err, uint32_t received) {
  uint8_t p[6] = {code, err};
  memcpy(p + 2, &received, 4);
  gCtrl->setValue(p, sizeof(p));
  gCtrl->notify();
}

void updateInfo(const char *state) {
  const esp_partition_t *running = esp_ota_get_running_partition();
  String s = String("version=") + FW_VERSION + ";slot=" + (running ? running->label : "?") +
             ";state=" + state;
  // Not setValue(s.c_str()): NimBLE 1.4's setValue<T> template takes a
  // const char* as T and stores the 4 pointer bytes instead of the text.
  gInfo->setValue((const uint8_t *)s.c_str(), s.length());
}

void holdAwake(bool on) {
#if CONFIG_PM_ENABLE
  // Keep the chip awake and at full speed while an update is streaming in.
  if (on) { esp_pm_lock_acquire(gNoSleep); esp_pm_lock_acquire(gFullSpeed); }
  else { esp_pm_lock_release(gNoSleep); esp_pm_lock_release(gFullSpeed); }
#endif
}

// 7.5 ms connection interval while an image streams in: twice the connection
// events of the normal 7.5-15 ms request, which BlueZ settles at 15 ms.
// Tracking is paused during an update, so nothing else needs the link then.
void setFastLink(bool fast) {
  uint16_t conn = gOtaConn;
  if (!gServer || conn == 0xFFFF) return;
  if (fast) gServer->updateConnParams(conn, 6, 6, 0, 200);
  else gServer->updateConnParams(conn, 6, 12, 0, 200);
}

void enqueue(uint8_t type, const uint8_t *data, size_t len, TickType_t wait) {
  Msg m;
  m.type = type;
  m.len = len > sizeof(m.data) ? sizeof(m.data) : len;
  if (data && m.len) memcpy(m.data, data, m.len);
  // Blocking here delays the write response, which slows the PC down
  // to however fast the flash can keep up.
  xQueueSend(gQueue, &m, wait);
}

void otaTask(void *) {
  static Msg m;
  esp_ota_handle_t handle = 0;
  const esp_partition_t *part = nullptr;
  mbedtls_sha256_context sha;
  uint8_t expectedSha[32];
  uint32_t expected = 0, received = 0, lastReport = 0;
  bool failed = false;  // report only the first error of a transfer

  auto stop = [&](uint8_t err, bool mayRestart = true) {
    bool wasActive = gActive;
    if (gActive) setFastLink(false);
    if (gActive) esp_ota_abort(handle);
    if (gActive) holdAwake(false);
    gActive = false;
    if (err != ERR_NONE && !failed) status(ST_ERROR, err, received);
    if (err != ERR_NONE) failed = true;
    mbedtls_sha256_free(&sha);
    // A transfer that stopped partway (data lost, stall, abort, PC gone) may
    // have lost writes, and those can leave NimBLE's buffers stuck: every
    // later write then crawls (measured: 512 B took 2.4 s instead of 48 ms)
    // until a reset. The running firmware is untouched, so restarting is safe
    // and fixes it. Not when a new BEGIN replaces the transfer.
    if (mayRestart && wasActive && received > 0) {
      Serial.println("OTA: transfer failed, restarting to clear Bluetooth buffers");
      vTaskDelay(pdMS_TO_TICKS(300));  // let the error notification go out
      esp_restart();
    }
  };
  mbedtls_sha256_init(&sha);

  for (;;) {
    if (xQueueReceive(gQueue, &m, gActive ? kIdleTimeout : portMAX_DELAY) != pdTRUE) {
      if (gActive) {
        Serial.println("OTA: no data for 10 s, giving up");
        stop(ERR_TIMEOUT);
      }
      continue;
    }
    switch (m.type) {
      case CMD_BEGIN: {
        if (gActive) stop(ERR_NONE, false);
        failed = false;
        received = lastReport = 0;
        if (m.len != 1 + 4 + 32) { status(ST_ERROR, ERR_BAD_CMD, 0); break; }
        // While this image is still on probation, the other slot holds the
        // known-good firmware we'd roll back to. Don't overwrite it yet.
        if (gPendingVerify) { status(ST_ERROR, ERR_PROBATION, 0); break; }
        memcpy(&expected, m.data + 1, 4);
        memcpy(expectedSha, m.data + 5, 32);
        part = esp_ota_get_next_update_partition(nullptr);
        if (!part || expected == 0 || expected > part->size) { status(ST_ERROR, ERR_TOO_BIG, 0); break; }
        holdAwake(true);
        Serial.printf("OTA: receiving %u bytes into %s\n", (unsigned)expected, part->label);
        if (esp_ota_begin(part, expected, &handle) != ESP_OK) {  // erases the slot, takes a few s
          holdAwake(false);
          status(ST_ERROR, ERR_BEGIN, 0);
          break;
        }
        mbedtls_sha256_init(&sha);
        mbedtls_sha256_starts_ret(&sha, 0);
        setFastLink(true);
        gActive = true;
        status(ST_READY, ERR_NONE, 0);
        break;
      }
      case CMD_DATA:
        if (!gActive) {
          if (!failed) status(ST_ERROR, ERR_NOT_STARTED, received);
          failed = true;
          break;
        }
        if (received + m.len > expected) { stop(ERR_SIZE); break; }
        if (esp_ota_write(handle, m.data, m.len) != ESP_OK) { stop(ERR_WRITE); break; }
        mbedtls_sha256_update_ret(&sha, m.data, m.len);
        received += m.len;
        if (received - lastReport >= kProgressEvery || received == expected) {
          lastReport = received;
          status(ST_PROGRESS, ERR_NONE, received);
        }
        break;
      case CMD_END: {
        if (!gActive) { status(ST_ERROR, ERR_NOT_STARTED, received); break; }
        if (received != expected) { stop(ERR_SIZE); break; }
        uint8_t got[32];
        mbedtls_sha256_finish_ret(&sha, got);
        if (memcmp(got, expectedSha, 32) != 0) { stop(ERR_HASH); break; }
        gActive = false;
        if (esp_ota_end(handle) != ESP_OK) {  // checks the image header and checksum
          holdAwake(false);
          status(ST_ERROR, ERR_IMAGE, received);
          break;
        }
        if (esp_ota_set_boot_partition(part) != ESP_OK) {
          holdAwake(false);
          status(ST_ERROR, ERR_BOOT, received);
          break;
        }
        Serial.println("OTA: image OK, rebooting into it");
        status(ST_DONE, ERR_NONE, received);
        vTaskDelay(pdMS_TO_TICKS(500));  // let the notification go out
        esp_restart();
        break;
      }
      case CMD_ABORT:
        stop(ERR_NONE);
        Serial.println("OTA: aborted");
        break;
    }
  }
}

class CtrlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, ble_gap_conn_desc *desc) override {
    NimBLEAttValue v = c->getValue();
    if (v.length() < 1) return;
    uint8_t cmd = v.data()[0];
    if (cmd == CMD_BEGIN) gOtaConn = desc->conn_handle;
    if (cmd == CMD_BEGIN || cmd == CMD_END || cmd == CMD_ABORT)
      enqueue(cmd, v.data(), v.length(), pdMS_TO_TICKS(2000));
  }
};

class DataCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, ble_gap_conn_desc *) override {
    NimBLEAttValue v = c->getValue();
    enqueue(CMD_DATA, v.data(), v.length(), pdMS_TO_TICKS(2000));
  }
};

void probationExpired(void *) {
  if (!gPendingVerify) return;
  // The new firmware never got healthy (hung somewhere). Go back.
  esp_ota_mark_app_invalid_rollback_and_reboot();
}

// ---- Telling the PC that the Bluetooth services changed ----
// BlueZ caches a device's GATT table, even without pairing, and keeps using
// the cached copy after a firmware update. So a new or changed characteristic
// (like write-without-response on 0012 in 1.2.2) stays invisible until the
// cache is cleared. The standard fix: indicate "Service Changed" for the whole
// table. BlueZ subscribes to it right after connecting, so that's when we send
// it, but only once per firmware image (its ELF SHA-256 is kept in flash once
// the PC has acknowledged), so normal connections don't trigger a rediscovery.
// With several PCs, only the first one to connect after an update gets it.
uint16_t gSvcChangedHandle = 0;
bool gAnnounced = false;  // this image already told a PC (or did just now)
bool gIndicating = false;

const uint8_t *imageSha() { return esp_ota_get_app_description()->app_elf_sha256; }

bool imageAnnounced() {
  Preferences p;
  if (!p.begin("ota", true)) return false;
  uint8_t sha[8] = {};
  bool same = p.getBytes("gatt_sha", sha, sizeof(sha)) == sizeof(sha) && memcmp(sha, imageSha(), sizeof(sha)) == 0;
  p.end();
  return same;
}

void rememberAnnounced() {
  Preferences p;
  if (!p.begin("ota", false)) return;
  p.putBytes("gatt_sha", imageSha(), 8);
  p.end();
}

int onGapEvent(ble_gap_event *ev, void *) {
  if (!gSvcChangedHandle || gAnnounced) return 0;
  if (ev->type == BLE_GAP_EVENT_SUBSCRIBE && ev->subscribe.attr_handle == gSvcChangedHandle &&
      ev->subscribe.cur_indicate) {
    Serial.println("OTA: new firmware, telling the PC to re-read the services");
    gIndicating = true;
    ble_svc_gatt_changed(0x0001, 0xFFFF);
  } else if (ev->type == BLE_GAP_EVENT_NOTIFY_TX && gIndicating && ev->notify_tx.attr_handle == gSvcChangedHandle &&
             ev->notify_tx.indication && ev->notify_tx.status == BLE_HS_EDONE) {  // the PC acknowledged it
    gIndicating = false;
    gAnnounced = true;
    rememberAnnounced();
  }
  return 0;
}

}  // namespace

// The Arduino core asks this at boot. Returning true means "don't confirm the
// new image yet, the sketch will do it" (see otaLoop).
extern "C" bool verifyRollbackLater() { return true; }

void otaSetup(NimBLEServer *server) {
#if CONFIG_PM_ENABLE
  esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "ota", &gNoSleep);
  esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "ota", &gFullSpeed);
#endif
  gServer = server;
  gQueue = xQueueCreate(kQueueLen, sizeof(Msg));
  xTaskCreate(otaTask, "ota", 8192, nullptr, 5, nullptr);

  NimBLEService *svc = server->createService(OTA_SERVICE_UUID);
  gCtrl = svc->createCharacteristic(OTA_CTRL_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  gCtrl->setCallbacks(new CtrlCallbacks());
  NimBLECharacteristic *data = svc->createCharacteristic(OTA_DATA_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  data->setCallbacks(new DataCallbacks());
  gInfo = svc->createCharacteristic(OTA_INFO_UUID, NIMBLE_PROPERTY::READ);
  svc->start();

  esp_ota_img_states_t st;
  const esp_partition_t *running = esp_ota_get_running_partition();
  gPendingVerify = running && esp_ota_get_state_partition(running, &st) == ESP_OK &&
                   st == ESP_OTA_IMG_PENDING_VERIFY;
  updateInfo(gPendingVerify ? "probation" : "valid");

  gAnnounced = imageAnnounced();
  // GATT service 0x1801, Service Changed 0x2A05 (BLE_UUID16_DECLARE is C-only).
  static const ble_uuid16_t kGattSvc = {{BLE_UUID_TYPE_16}, 0x1801}, kSvcChanged = {{BLE_UUID_TYPE_16}, 0x2A05};
  if (ble_gatts_find_chr(&kGattSvc.u, &kSvcChanged.u, nullptr, &gSvcChangedHandle) != 0)
    gSvcChangedHandle = 0;
  NimBLEDevice::setCustomGapHandler(onGapEvent);
  if (gPendingVerify) {
    Serial.println("OTA: new firmware on probation");
    esp_timer_create_args_t args = {};
    args.callback = probationExpired;
    args.name = "ota_probation";
    esp_timer_create(&args, &gProbationTimer);
    esp_timer_start_once(gProbationTimer, kProbationUs);
  }
}

void otaLoop() {
  if (gPendingVerify && millis() > kHealthyAfterMs) {
    esp_ota_mark_app_valid_cancel_rollback();
    gPendingVerify = false;
    esp_timer_stop(gProbationTimer);
    updateInfo("valid");
    Serial.println("OTA: new firmware confirmed");
  }
}

void otaOnDisconnect(uint16_t connHandle) {
  // Not just while gActive: the updater may drop while the slot is still being
  // erased (gActive is set only after that), and the transfer would then start
  // with nobody left to feed it. The abort queues behind the begin, so it lands.
  if (connHandle == gOtaConn) {
    gOtaConn = 0xFFFF;
    enqueue(CMD_ABORT, nullptr, 0, 0);
  }
}

bool otaInProgress() { return gActive; }
