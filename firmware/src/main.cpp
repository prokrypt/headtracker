// ESP32-C3 Super Mini + BNO085 head tracker -> BLE notifications.
//
// The tracker does the mount correction and the recentering itself, so the
// packet already holds the head's angles away from "straight ahead".
//
// Head packet, characteristic 0002 (14 bytes, little-endian, notify), sent on
// every new sensor sample (~100 Hz):
//   float yaw, pitch, roll   degrees from the last recenter, X-Plane's
//                      conventions: yaw > 0 looks right, pitch > 0 looks up,
//                      roll > 0 tilts the right ear down
//   uint8 flags        bit0 = recenter button is held
//                      bit1 = mount is calibrated (0 = default: board flat, +Y to the nose)
//                      bits4-7 = recenter count, +1 on every recenter
//   uint8 seq          increments every packet
//
// Control, characteristic 0003 (write + notify). Commands from the PC:
//   01  recenter now
//   02  calibration step 1: capture "looking straight ahead"
//   03  calibration step 2: capture "nodded down", work out the mount, save it, recenter
//   04  forget the calibration (back to the default mount)
// Reply (notify): <u8 command> <u8 result>, result 0 = OK, see CalResult in mount.h.
//
// The tracker recenters by itself when the sensor (re)starts (after it slept
// because nobody listened for a while, or after a reset), and when the BOOT
// button is pressed. A short Bluetooth drop keeps the old center.
//
// Power: the chip light-sleeps whenever it's waiting (for the next sensor
// sample, between BLE connection events, and almost all the time while nothing
// is connected). Waking is driven by the BNO085's INT pin, so sleeping adds no
// latency to tracking. Without INT wired it still works, it just polls.
//
// Built with FAKE_SENSOR=1 (pio env esp32c3-fake) there is no BNO085: the
// tracker sends simulated head motion at 100 Hz instead, for testing Bluetooth,
// the plugin and OTA before the sensor arrives. That build keeps its mount
// calibration under its own flash key, so it never leaks into the real one.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#include "mount.h"
#include "ota.h"

#ifndef FAKE_SENSOR
#define FAKE_SENSOR 0
#endif
// NVS namespace for the mount calibration.
static const char *PREFS_NS = FAKE_SENSOR ? "trackerfake" : "tracker";

// ---- Pins (ESP32-C3 Super Mini) ----
// GPIO2/8/9 are strapping pins and GPIO8 drives the onboard LED, so avoid them
// for the sensor. GPIO9 is the BOOT button, which we reuse as "recenter".
static const int PIN_SDA = 5;
static const int PIN_SCL = 6;
static const int PIN_BNO_RST = 10;   // BNO085 RST (optional but recommended)
static const int PIN_BNO_INT = 4;    // BNO085 INT, active low (optional, saves power)
static const int PIN_RECENTER = 9;   // onboard BOOT button, active low
static const int PIN_LED = 8;        // onboard LED, active low

static const uint32_t REPORT_INTERVAL_US = 10000;  // 100 Hz
static const uint32_t SENSOR_LINGER_MS = 10000;    // sensor keeps running this long after X-Plane leaves

// ---- BLE identifiers (must match the plugin) ----
static const char *DEVICE_NAME = "HeadTrackerC3";
static const char *SERVICE_UUID = "6e7a0001-3c1b-4b8a-9f2e-6b1d5a7c0de1";
static const char *QUAT_CHAR_UUID = "6e7a0002-3c1b-4b8a-9f2e-6b1d5a7c0de1";
static const char *CTRL_CHAR_UUID = "6e7a0003-3c1b-4b8a-9f2e-6b1d5a7c0de1";

enum : uint8_t { CMD_RECENTER = 0x01, CMD_CAL_AHEAD = 0x02, CMD_CAL_NOD = 0x03, CMD_CAL_RESET = 0x04 };

// Advertise slowly while idle (units of 0.625 ms): about 0.5 s between beacons.
// The plugin scans for 4 s at a time, so it still finds the tracker quickly.
static const uint16_t ADV_INTERVAL_MIN = 720;
static const uint16_t ADV_INTERVAL_MAX = 800;

static Adafruit_BNO08x bno(PIN_BNO_RST);
static sh2_SensorValue_t sensorValue;
static NimBLEServer *server = nullptr;
static NimBLECharacteristic *quatChar = nullptr;
static NimBLECharacteristic *ctrlChar = nullptr;
static QueueHandle_t cmdQueue;  // control commands, from the BLE task to loop()
static bool sensorOk = false;
static bool sensorAwake = false;
static bool intWired = false;  // set once we see INT go low or fire
static uint8_t seq = 0;
static uint8_t sensorEpoch = 0;  // +1 whenever the BNO085 (re)starts; its yaw reference starts over then
static uint32_t sensorStartMs = 0;  // when it last (re)started, for the "stopped reporting" check
static SemaphoreHandle_t intSem;

// Mount and recenter state (only touched by loop()).
static Q mountRot;             // head = sensor * mountRot
static bool mountCalibrated = false;
static Angles headRef;         // head angles at the last recenter
static bool haveRef = false;
static uint8_t refEpoch = 0;   // sensor epoch the reference belongs to
static uint8_t recenterCount = 0;
static bool recenterPending = false;
static Q lastSensorQ;          // newest sample, for calibration
static uint32_t lastSampleMs = 0;
static bool haveSample = false;
static Q calAheadQ;            // calibration step 1
static bool calAheadValid = false;
static uint8_t calAheadEpoch = 0;
static Preferences prefs;

#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t usbLock;  // stay awake while plugged into a computer
static bool usbLockHeld = false;
#endif

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, ble_gap_conn_desc *desc) override {
    // 7.5-15 ms connection interval so 100 Hz updates get through, no
    // peripheral latency (it would delay updates), 2 s supervision timeout.
    s->updateConnParams(desc->conn_handle, 6, 12, 0, 200);
    // Keep advertising so the OTA updater can connect while X-Plane is running.
    if (s->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS) NimBLEDevice::startAdvertising();
  }
  void onDisconnect(NimBLEServer *, ble_gap_conn_desc *desc) override {
    otaOnDisconnect(desc->conn_handle);  // NimBLE restarts advertising itself
  }
};

class CtrlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) override {
    std::string v = c->getValue();
    if (!v.empty()) {
      uint8_t cmd = (uint8_t)v[0];
      xQueueSend(cmdQueue, &cmd, 0);
    }
  }
};

static void loadMount() {
  prefs.begin(PREFS_NS, true);
  float m[4] = {0, 0, 0, 0};
  bool ok = prefs.getBytes("mount", m, sizeof(m)) == sizeof(m);
  prefs.end();
  Q q{m[0], m[1], m[2], m[3]};
  float n = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (ok && isfinite(n) && fabsf(n - 1.0f) < 0.01f) {
    mountRot = qnorm(q);
    mountCalibrated = true;
    Serial.println("Mount calibration loaded");
  } else {
    mountRot = Q();
    mountCalibrated = false;
    Serial.println("No mount calibration, assuming the board lies flat with +Y to the nose");
  }
}

static bool saveMount() {
  if (!prefs.begin(PREFS_NS, false)) return false;
  bool ok;
  if (mountCalibrated) {
    float m[4] = {mountRot.w, mountRot.x, mountRot.y, mountRot.z};
    ok = prefs.putBytes("mount", m, sizeof(m)) == sizeof(m);
  } else {
    ok = !prefs.isKey("mount") || prefs.remove("mount");
  }
  prefs.end();
  return ok;
}

static void reply(uint8_t cmd, uint8_t result) {
  uint8_t p[2] = {cmd, result};
  ctrlChar->setValue(p, sizeof(p));
  ctrlChar->notify();
}

static bool sampleFresh() { return haveSample && millis() - lastSampleMs < 200; }

static void setReference(const Q &sensorQ) {
  headRef = headAngles(qmul(sensorQ, mountRot));
  haveRef = true;
  refEpoch = sensorEpoch;
  recenterPending = false;
  recenterCount++;
}

static void handleCommand(uint8_t cmd) {
  switch (cmd) {
    case CMD_RECENTER:
      // Applied on the next sample, which is at most 10 ms away while connected.
      recenterPending = true;
      reply(cmd, CAL_OK);
      Serial.println("Recenter requested");
      break;
    case CMD_CAL_AHEAD:
      if (!sampleFresh()) { reply(cmd, CAL_NO_DATA); break; }
      calAheadQ = lastSensorQ;
      calAheadValid = true;
      calAheadEpoch = sensorEpoch;
      reply(cmd, CAL_OK);
      Serial.println("Calibration: straight ahead captured");
      break;
    case CMD_CAL_NOD: {
      if (!calAheadValid || calAheadEpoch != sensorEpoch) { reply(cmd, CAL_NO_STEP1); break; }
      if (!sampleFresh()) { reply(cmd, CAL_NO_DATA); break; }
      Q m;
      CalResult r = computeMount(calAheadQ, lastSensorQ, m);
      if (r == CAL_OK) {
        mountRot = m;
        mountCalibrated = true;
        calAheadValid = false;
        setReference(calAheadQ);  // "straight ahead" is the pose from step 1
        if (!saveMount()) r = CAL_SAVE_FAILED;
      }
      reply(cmd, r);
      Serial.printf("Calibration: result %d\n", r);
      break;
    }
    case CMD_CAL_RESET:
      mountRot = Q();
      mountCalibrated = false;
      calAheadValid = false;
      recenterPending = true;
      reply(cmd, saveMount() ? CAL_OK : CAL_SAVE_FAILED);
      Serial.println("Calibration reset to default mount");
      break;
    default:
      break;
  }
}

static void IRAM_ATTR onBnoInt(void *) {
  // Level interrupt: mask it until the main loop has read the sample,
  // otherwise it would fire continuously while INT is held low.
  gpio_intr_disable((gpio_num_t)PIN_BNO_INT);
  BaseType_t woke = pdFALSE;
  xSemaphoreGiveFromISR(intSem, &woke);
  if (woke) portYIELD_FROM_ISR();
}

static void setupInt() {
  intSem = xSemaphoreCreateBinary();
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << PIN_BNO_INT;
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_ENABLE;  // reads high (idle) if INT isn't wired
  io.intr_type = GPIO_INTR_LOW_LEVEL;  // level, so it also fires right after light sleep
  gpio_config(&io);
  gpio_intr_disable((gpio_num_t)PIN_BNO_INT);
  gpio_install_isr_service(0);  // harmless if the Arduino core already did it
  gpio_isr_handler_add((gpio_num_t)PIN_BNO_INT, onBnoInt, nullptr);
  gpio_wakeup_enable((gpio_num_t)PIN_BNO_INT, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
}

static void setupPower() {
#if CONFIG_PM_ENABLE
  esp_pm_config_esp32c3_t pm = {};
  pm.max_freq_mhz = 160;
  pm.min_freq_mhz = 40;          // drop to the crystal clock when idle
  pm.light_sleep_enable = true;  // sleep automatically whenever every task is waiting
  if (esp_pm_configure(&pm) == ESP_OK) Serial.println("Power: automatic light sleep on");
  else Serial.println("Power: esp_pm_configure failed, running without sleep");
  esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "usb", &usbLock);
  esp_pm_lock_acquire(usbLock);  // until we know whether a computer is attached
  usbLockHeld = true;
#else
  Serial.println("Power: built without CONFIG_PM_ENABLE, no light sleep");
#endif
}

// Light sleep switches off USB, which would kill the serial monitor and make
// uploads awkward. So while a computer's USB port is attached, stay awake.
// On a power bank or battery there's no USB host, and the chip sleeps.
static void updateUsbLock() {
#if CONFIG_PM_ENABLE
  static uint32_t last = 0;
  if (millis() < 3000 || millis() - last < 1000) return;  // give USB 3 s to enumerate
  last = millis();
  bool plugged = HWCDC::isPlugged();
  if (plugged && !usbLockHeld) { esp_pm_lock_acquire(usbLock); usbLockHeld = true; }
  if (!plugged && usbLockHeld) { esp_pm_lock_release(usbLock); usbLockHeld = false; }
#endif
}

#if FAKE_SENSOR
// Simulated head: slow look left/right, up/down and a little ear tilt, each on
// its own period so the motion doesn't repeat quickly. Built so headAngles()
// gives exactly these angles: M = Rz(-yaw) * Rx(pitch) * Ry(roll).
static Q fakeSensorQ() {
  const float d2r = (float)M_PI / 180.0f, tau = 2.0f * (float)M_PI;
  float t = (millis() % 3600000UL) / 1000.0f;  // wrap hourly to keep float precision
  float yaw = 45.0f * d2r * sinf(tau * t / 8.0f);
  float pitch = 15.0f * d2r * sinf(tau * t / 5.3f);
  float roll = 8.0f * d2r * sinf(tau * t / 6.7f);
  Q qz{cosf(-yaw / 2), 0, 0, sinf(-yaw / 2)};
  Q qx{cosf(pitch / 2), sinf(pitch / 2), 0, 0};
  Q qy{cosf(roll / 2), 0, sinf(roll / 2), 0};
  return qmul(qmul(qz, qx), qy);
}
#endif

static bool startSensor() {
#if FAKE_SENSOR
  sensorEpoch++;
  sensorStartMs = millis();
  sensorAwake = true;
  return true;
#endif
  // Clone boards ship with ADO either low (0x4A) or high (0x4B); try both.
  if (bno.begin_I2C(0x4A, &Wire) || bno.begin_I2C(0x4B, &Wire)) {
    sensorEpoch++;
    sensorStartMs = millis();
    sensorAwake = bno.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US);
    return sensorAwake;
  }
  return false;
}

// The BNO085 draws several mA while fusing, so park it when nobody listens.
static void sensorSleep(bool sleep) {
  if (sleep == !sensorAwake) return;
  if (sleep) {
    if (!FAKE_SENSOR) sh2_devSleep();
  } else {
    if (!FAKE_SENSOR) {
      sh2_devOn();
      bno.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US);
    }
    sensorEpoch++;
    sensorStartMs = millis();
  }
  sensorAwake = !sleep;
  Serial.println(sleep ? "BNO085 sleeping (nobody listening)" : "BNO085 awake");
}

static void startBle() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(517);  // big packets make OTA fast
  server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  NimBLEService *service = server->createService(SERVICE_UUID);
  quatChar = service->createCharacteristic(
      QUAT_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  ctrlChar = service->createCharacteristic(
      CTRL_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);
  ctrlChar->setCallbacks(new CtrlCallbacks());
  service->start();
  otaSetup(server);
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);  // scan response carries the device name
  adv->setMinInterval(ADV_INTERVAL_MIN);
  adv->setMaxInterval(ADV_INTERVAL_MAX);
  NimBLEDevice::startAdvertising();
}

// LED: one short blink every 2 s while waiting for X-Plane, a double blink
// every 2 s when connected. Mostly off, to save power.
// Returns how many ms until the LED next has to change, so the idle loop can
// sleep exactly that long instead of waking up every few ms.
static uint32_t updateLed(bool connected) {
  uint32_t t = millis() % 2000;
  bool on = t < 40 || (connected && t >= 150 && t < 190);
  digitalWrite(PIN_LED, on ? LOW : HIGH);
  static const uint32_t edgesConnected[] = {40, 150, 190, 2000};
  static const uint32_t edgesIdle[] = {40, 2000};
  const uint32_t *e = connected ? edgesConnected : edgesIdle;
  size_t n = connected ? 4 : 2;
  for (size_t i = 0; i < n; ++i)
    if (e[i] > t) return e[i] - t;
  return 1;
}

void setup() {
  Serial.begin(115200);
  setupPower();
  pinMode(PIN_RECENTER, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);  // off
  // ESP-IDF on the C3 floats every GPIO during light sleep (a workaround for a
  // chip reset bug). The BNO085's RST line must stay driven high, or a board
  // without its own pull-up could reset the sensor every time we sleep. The
  // LED pin is kept too, so it stays cleanly off.
  pinMode(PIN_BNO_RST, OUTPUT);
  digitalWrite(PIN_BNO_RST, HIGH);
  gpio_sleep_sel_dis((gpio_num_t)PIN_BNO_RST);
  gpio_sleep_sel_dis((gpio_num_t)PIN_LED);
  // Same for the I2C lines, in case the sensor board has no pull-ups of its own.
  gpio_sleep_sel_dis((gpio_num_t)PIN_SDA);
  gpio_sleep_sel_dis((gpio_num_t)PIN_SCL);
  setupInt();

  cmdQueue = xQueueCreate(8, sizeof(uint8_t));
  loadMount();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  // Bluetooth first, so a firmware update still works if the sensor is broken.
  startBle();
  Serial.println("BLE advertising as HeadTrackerC3");
  if (FAKE_SENSOR) Serial.println("FAKE SENSOR build: no BNO085, sending simulated head motion");
}

void loop() {
  otaLoop();
  updateUsbLock();
  uint32_t ledDue = updateLed(server->getConnectedCount() > 0);
  uint8_t cmd;
  while (xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE) handleCommand(cmd);

  if (!sensorOk) {
    static uint32_t lastTry = 0;
    if (lastTry == 0 || millis() - lastTry > 1000) {
      lastTry = millis();
      sensorOk = startSensor();
      Serial.println(sensorOk ? (FAKE_SENSOR ? "Fake sensor ready" : "BNO085 ready")
                              : "BNO085 not found, check wiring. Retrying...");
    }
    vTaskDelay(pdMS_TO_TICKS(ledDue < 250 ? ledDue : 250));
    return;
  }

  if (!FAKE_SENSOR && bno.wasReset()) {
    Serial.println("BNO085 reset, re-enabling report");
    sensorEpoch++;
    sensorStartMs = millis();
    sensorAwake = bno.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US);
  }

  // Keep the sensor running for a while after the last listener left, so a
  // short Bluetooth drop doesn't restart it (that would lose the center).
  static uint32_t lastListened = 0;
  bool subscribed = quatChar->getSubscribedCount() > 0 && !otaInProgress();
  if (subscribed) lastListened = millis();
  bool listening = subscribed || (sensorAwake && !otaInProgress() && millis() - lastListened < SENSOR_LINGER_MS);
  sensorSleep(!listening);
  // The sensor stopped reporting while someone listens (a loose wire, or the
  // BNO08x's I2C hang): hard-reset it through RST. That starts a new epoch, so
  // the view recenters once it's back.
  if (listening) {
    uint32_t since = haveSample && (int32_t)(lastSampleMs - sensorStartMs) > 0 ? lastSampleMs : sensorStartMs;
    if (millis() - since > 1000) {
      Serial.println("BNO085 stopped reporting, restarting it");
      sensorOk = false;
      haveSample = false;
      return;
    }
  }
  if (!listening) {
    // INT also wakes the chip from light sleep. If the sensor has something
    // queued (a reply to the sleep command, a last sample), INT stays low until
    // it's read, and the chip would never get to sleep. So read it out.
    if (!FAKE_SENSOR)
      for (int i = 0; i < 8 && digitalRead(PIN_BNO_INT) == LOW; ++i) bno.getSensorEvent(&sensorValue);
    // Nothing to send: sleep until the LED needs to change, but check for a
    // new subscriber (X-Plane connecting) at least every 250 ms.
    vTaskDelay(pdMS_TO_TICKS(ledDue < 250 ? ledDue : 250));
    return;
  }

#if FAKE_SENSOR
  // One simulated sample every 10 ms (100 Hz, like the real report rate).
  // DelayUntil keeps the average at exactly 100 Hz; a plain delay adds the
  // loop's own time to every period (measured 89 Hz).
  static TickType_t nextWake = xTaskGetTickCount();
  if ((int32_t)(xTaskGetTickCount() - nextWake) > (int32_t)pdMS_TO_TICKS(100)) nextWake = xTaskGetTickCount();  // after a pause, don't burst to catch up
  vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(REPORT_INTERVAL_US / 1000));
  Q fq = fakeSensorQ();
  float q[4] = {fq.w, fq.x, fq.y, fq.z};
#else
  // Wait for the next sample. With INT wired we sleep until the sensor says
  // it's ready, so there's no added latency; otherwise poll every 2 ms.
  if (digitalRead(PIN_BNO_INT) == HIGH) {
    gpio_intr_enable((gpio_num_t)PIN_BNO_INT);
    // The interrupt firing proves INT is wired; until then, poll every 2 ms.
    if (xSemaphoreTake(intSem, pdMS_TO_TICKS(intWired ? 25 : 2)) == pdTRUE) intWired = true;
    gpio_intr_disable((gpio_num_t)PIN_BNO_INT);
  } else {
    intWired = true;
  }

  // Drain whatever is queued and keep the newest rotation. Copy it out as it
  // arrives: a later event of another kind would overwrite sensorValue.
  bool got = false;
  float q[4];
  while (bno.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      q[0] = sensorValue.un.gameRotationVector.real;
      q[1] = sensorValue.un.gameRotationVector.i;
      q[2] = sensorValue.un.gameRotationVector.j;
      q[3] = sensorValue.un.gameRotationVector.k;
      got = true;
    }
    if (digitalRead(PIN_BNO_INT) == HIGH) break;
  }
  if (!got) {
    // INT low but nothing to read (sensor wedged or mid-reset): don't spin,
    // or the idle task starves and the chip never sleeps.
    vTaskDelay(1);
    return;
  }
#endif

  Q sensorQ = qnorm(Q{q[0], q[1], q[2], q[3]});
  lastSensorQ = sensorQ;
  lastSampleMs = millis();
  haveSample = true;

  // Recenter on the BOOT button, on request, and whenever the sensor has
  // (re)started, because its yaw reference starts over then.
  static bool lastButton = false;
  bool button = digitalRead(PIN_RECENTER) == LOW;
  if (button && !lastButton) recenterPending = true;
  lastButton = button;
  if (recenterPending || !haveRef || refEpoch != sensorEpoch) setReference(sensorQ);

  Angles a = relativeTo(headAngles(qmul(sensorQ, mountRot)), headRef);
  float out[3] = {a.yaw, a.pitch, a.roll};
  uint8_t pkt[14];
  memcpy(pkt, out, sizeof(out));  // ESP32 is little-endian
  pkt[12] = (button ? 0x01 : 0x00) | (mountCalibrated ? 0x02 : 0x00) | uint8_t((recenterCount & 0x0F) << 4);
  pkt[13] = seq++;
  if (!subscribed) return;  // lingering after a drop: keep the sensor going, nobody to tell
  quatChar->setValue(pkt, sizeof(pkt));
  quatChar->notify();
}
