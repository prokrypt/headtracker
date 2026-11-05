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
//                      bit2 = the compass is correcting yaw right now
//                      bits4-7 = recenter count, +1 on every recenter
//   uint8 seq          increments every packet
//
// Control, characteristic 0003 (write + notify). Commands from the PC:
//   01  recenter now
//   02  calibration step 1: capture "looking straight ahead"
//   03  calibration step 2: capture "nodded down", work out the mount, save it, recenter
//   04  forget the calibration (back to the default mount)
//   05  compass calibration (M5Stack build): turn the tracker every way for
//       20s, the reply comes at the end (button B does the same)
//   06  forget the compass calibration (M5Stack build)
//   07  play the low-battery blip on the speaker (M5Stack build, for testing)
//   71..7F  the same blip at DAC amplitude 1..15, to find a bearable volume
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
//
// The BNO085 runs its ARVR-stabilized Rotation Vector: gyro + accelerometer +
// magnetometer, so yaw doesn't drift, with heading corrections applied only
// while the head moves (no visible creep). Its own sensor calibration (DCD)
// runs all the time and is saved to the BNO085's flash when it's good.
//
// Built with IMU_MPU6886=1 (pio env m5core) it runs on an M5Stack Core instead:
// classic ESP32, its built-in MPU6886 + BMM150, fusion done here (see
// imu6886.h). Same protocol; it advertises as HeadTracker-M5. Button A recenters, button B calibrates the compass.
// It powers itself off after a minute lying still or a minute without a
// connection (red button or button A turns it back on). With the M5GO/Fire
// base fitted, its LED bar shows power on/off and connection changes briefly,
// and blinks red at both ends when the battery is down to its last gauge step
// (about 25%; the whole bar when nearly empty). The speaker gives a quiet blip
// then too: every 5 minutes when low, every 10s when nearly empty. No light sleep on that board
// (no 32kHz crystal for BLE); it runs at a fixed 80MHz and sends at 0dBm instead.

#include <Arduino.h>
#include <Wire.h>
#ifndef IMU_MPU6886
#define IMU_MPU6886 0
#endif
#if IMU_MPU6886
#include "imu6886.h"
#include <Adafruit_NeoPixel.h>
#else
#include <Adafruit_BNO08x.h>
#endif
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#if IMU_MPU6886
#include <driver/dac_common.h>
#include <driver/rmt.h>
#include <esp_timer.h>
#endif
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

#if IMU_MPU6886
// ---- Pins (M5Stack Core) ----
static const int PIN_SDA = 21;       // internal I2C: MPU6886, BMM150, IP5306
static const int PIN_SCL = 22;
static const int PIN_RECENTER = 39;  // button A, active low (pull-up on the board)
static const int PIN_MAGCAL = 38;    // button B: compass calibration
static const int PIN_LCD_BL = 32;    // LCD backlight, kept off (LCD isn't used)
static const int PIN_SPEAKER = 25;   // held low, or the speaker hisses (no sounds: too loud)
static const int PIN_LEDBAR = 15;    // M5GO/Fire base: 10 SK6812 LEDs (fine without the base)
static const int NUM_LEDS = 10;
static const uint32_t SAMPLE_INTERVAL_US = 5000;   // read the IMU at 200Hz, send every 2nd
static const uint32_t OFF_STILL_S = 60;             // power off after lying still this long
static const uint32_t OFF_NO_CONN_MS = 60000;       // or after this long without a connection
#else
// ---- Pins (ESP32-C3 Super Mini) ----
// GPIO2/8/9 are strapping pins and GPIO8 drives the onboard LED, so avoid them
// for the sensor. GPIO9 is the BOOT button, which we reuse as "recenter".
static const int PIN_SDA = 5;
static const int PIN_SCL = 6;
static const int PIN_BNO_RST = 10;   // BNO085 RST (optional but recommended)
static const int PIN_BNO_INT = 4;    // BNO085 INT, active low (optional, saves power)
static const int PIN_RECENTER = 9;   // onboard BOOT button, active low
static const int PIN_LED = 8;        // onboard LED, active low
#endif

static const uint32_t REPORT_INTERVAL_US = 10000;  // 100 Hz
static const uint32_t SENSOR_LINGER_MS = 10000;    // sensor keeps running this long after X-Plane leaves

// ---- BLE identifiers (must match the plugin) ----
// The plugin and the updater find the tracker by SERVICE_UUID, so the name is
// only for telling trackers apart.
static const char *DEVICE_NAME = IMU_MPU6886 ? "HeadTracker-M5" : "HeadTracker-C3";
static const char *SERVICE_UUID = "6e7a0001-3c1b-4b8a-9f2e-6b1d5a7c0de1";
static const char *QUAT_CHAR_UUID = "6e7a0002-3c1b-4b8a-9f2e-6b1d5a7c0de1";
static const char *CTRL_CHAR_UUID = "6e7a0003-3c1b-4b8a-9f2e-6b1d5a7c0de1";

enum : uint8_t {
  CMD_RECENTER = 0x01, CMD_CAL_AHEAD = 0x02, CMD_CAL_NOD = 0x03, CMD_CAL_RESET = 0x04,
  CMD_MAG_CAL = 0x05, CMD_MAG_RESET = 0x06, CMD_BLIP = 0x07,
  CMD_BLIP_LEVEL = 0x70,  // 0x71..0x7F: blip at amplitude 1..15
};

// Advertise slowly while idle (units of 0.625 ms): about 0.5 s between beacons.
// The plugin scans for 4 s at a time, so it still finds the tracker quickly.
static const uint16_t ADV_INTERVAL_MIN = 720;
static const uint16_t ADV_INTERVAL_MAX = 800;

#if IMU_MPU6886
static Mpu6886 imu;
static Fusion fusion;
static uint32_t lastImuUs = 0;
static uint8_t imuSkip = 0;  // samples to throw away after (re)start
static Bmm150 bmm;
static MagCal magCal;
static MagCalibrator magCalibrator;
static uint32_t lastMagUs = 0;
static uint32_t noConnSince = 0;  // for powering off
enum class Anim : uint8_t { None, PowerOn, Connected, Disconnected, Recenter, CalOk, CalFail };
static void ledAnim(Anim a);
#else
static Adafruit_BNO08x bno(PIN_BNO_RST);
[[maybe_unused]] static sh2_SensorValue_t sensorValue;
// ARVR-stabilized Rotation Vector: uses the magnetometer, corrects only while moving.
static const sh2_SensorId_t ROTATION_REPORT = SH2_ARVR_STABILIZED_RV;
[[maybe_unused]] static uint8_t bnoAccuracy = 0;   // 0 unreliable .. 3 high (mostly the magnetometer calibration)
[[maybe_unused]] static uint32_t lastDcdSaveMs = 0;
#endif
static NimBLEServer *server = nullptr;
static NimBLECharacteristic *quatChar = nullptr;
static NimBLECharacteristic *ctrlChar = nullptr;
static QueueHandle_t cmdQueue;  // control commands, from the BLE task to loop()
static bool sensorOk = false;
static bool sensorAwake = false;
[[maybe_unused]] static bool intWired = false;  // set once we see INT go low or fire (unused in the fake build)
static uint8_t seq = 0;
static uint8_t sensorEpoch = 0;  // +1 whenever the BNO085 (re)starts; its yaw reference starts over then
static uint32_t sensorStartMs = 0;  // when it last (re)started, for the "stopped reporting" check
[[maybe_unused]] static SemaphoreHandle_t intSem;

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

#if CONFIG_PM_ENABLE && CONFIG_IDF_TARGET_ESP32C3
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

#if IMU_MPU6886
// Gyro bias, so a fresh start begins close to right (refined once it holds still).
static void loadBias() {
  float b[3] = {0, 0, 0};
  prefs.begin(PREFS_NS, true);
  bool ok = prefs.getBytes("gbias", b, sizeof(b)) == sizeof(b);
  prefs.end();
  ok = ok && isfinite(b[0]) && isfinite(b[1]) && isfinite(b[2]) && fabsf(b[0]) + fabsf(b[1]) + fabsf(b[2]) < 1.0f;
  fusion.setBias(ok ? V{b[0], b[1], b[2]} : V{}, true);
}

static void saveBias() {
  V v = fusion.bias();
  float b[3] = {v.x, v.y, v.z};
  if (prefs.begin(PREFS_NS, false)) {
    prefs.putBytes("gbias", b, sizeof(b));
    prefs.end();
  }
  const float k = 180.0f / (float)M_PI;
  Serial.printf("Gyro bias %.3f %.3f %.3f dps saved\n", v.x * k, v.y * k, v.z * k);
}
#endif

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

#if IMU_MPU6886
static void startMagCal();
static void forgetMagCal();
#endif

#if IMU_MPU6886
// ---- Speaker blip (low-battery reminder) ----
// The speaker amp (GPIO25 = DAC1 into it) is always powered on this board, so
// a blip costs nothing extra. PWM beeps were painfully loud even at 4% duty
// (full 3.3V edges), so this is a soft 1kHz sine from the DAC at a tiny
// amplitude, with 20ms fades so it doesn't click. Samples come from a 16kHz
// esp_timer for 150ms; the main loop keeps running.
static const uint8_t BLIP_AMPLITUDE = 8;  // DAC counts of 255 (3.3V), peak-to-peak = 2x this; 8 picked by ear (user)
static const uint32_t BLIP_RATE = 16000, BLIP_MS = 150, BLIP_FADE_MS = 20;
static const int8_t kSine16[16] = {0, 49, 90, 117, 127, 117, 90, 49, 0, -49, -90, -117, -127, -117, -90, -49};
static esp_timer_handle_t blipTimer = nullptr;
static volatile bool blipRunning = false, blipFinished = false;
static volatile uint32_t blipN = 0;
static volatile uint8_t blipAmp = 0;

static void blipTick(void *) {
  const uint32_t total = BLIP_RATE * BLIP_MS / 1000, fade = BLIP_RATE * BLIP_FADE_MS / 1000;
  uint32_t n = blipN++;
  if (n >= total) {
    esp_timer_stop(blipTimer);
    dac_output_voltage(DAC_CHANNEL_1, 0);
    blipFinished = true;
    return;
  }
  float env = fminf(1.0f, fminf((float)n / fade, (float)(total - n) / fade));
  // 0..2*amp, starting and ending at 0V (where the pin sits between blips)
  float v = blipAmp * env * (1.0f + kSine16[n % 16] / 127.0f);
  dac_output_voltage(DAC_CHANNEL_1, (uint8_t)(v + 0.5f));
}

static void blipStop(bool now);

// Returns false (nothing plays) if a blip is still playing or the timer
// couldn't be made.
static bool blipStart(uint8_t amp) {
  blipStop(false);  // a finished blip the main loop hasn't tidied up yet
  if (blipRunning || amp == 0) return false;
  if (!blipTimer) {
    esp_timer_create_args_t a = {};
    a.callback = blipTick;
    a.name = "blip";
    if (esp_timer_create(&a, &blipTimer) != ESP_OK) return false;
  }
  blipAmp = amp;
  blipN = 0;
  blipFinished = false;
  dac_output_voltage(DAC_CHANNEL_1, 0);
  dac_output_enable(DAC_CHANNEL_1);  // pin was a digital low: same 0V, no click
  blipRunning = true;
  esp_timer_start_periodic(blipTimer, 1000000 / BLIP_RATE);
  return true;
}

// Back to a plain digital low once a blip is done (or at once, before power off).
static void blipStop(bool now) {
  if (!blipRunning || (!now && !blipFinished)) return;
  if (now) esp_timer_stop(blipTimer);
  dac_output_voltage(DAC_CHANNEL_1, 0);
  dac_output_disable(DAC_CHANNEL_1);
  pinMode(PIN_SPEAKER, OUTPUT);
  digitalWrite(PIN_SPEAKER, LOW);
  blipRunning = false;
}
#endif

static void handleCommand(uint8_t cmd) {
  switch (cmd) {
#if IMU_MPU6886
    case CMD_MAG_CAL:
      startMagCal();  // replies when it's done, 20s later
      break;
    case CMD_MAG_RESET:
      forgetMagCal();
      reply(cmd, CAL_OK);
      break;
#endif
    case CMD_RECENTER:
      // Applied on the next sample, which is at most 10 ms away while connected.
      recenterPending = true;
      reply(cmd, CAL_OK);
      Serial.println("Recenter requested");
#if IMU_MPU6886
      ledAnim(Anim::Recenter);
#endif
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
#if IMU_MPU6886
      // 07 = blip at the built-in volume, 71..7F = blip at amplitude 1..15
      if (cmd == CMD_BLIP || (cmd > CMD_BLIP_LEVEL && cmd <= CMD_BLIP_LEVEL + 15)) {
        uint8_t amp = cmd == CMD_BLIP ? BLIP_AMPLITUDE : cmd - CMD_BLIP_LEVEL;
        bool ok = blipStart(amp);
        reply(cmd, ok ? CAL_OK : CAL_BUSY);
        Serial.printf(ok ? "Blip, amplitude %u\n" : "Blip at amplitude %u skipped: one is still playing\n", amp);
      }
#endif
      break;
  }
}

#if !IMU_MPU6886
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
#endif

static void setupPower() {
#if CONFIG_PM_ENABLE && CONFIG_IDF_TARGET_ESP32C3
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
#if IMU_MPU6886
  // sdkconfig.defaults asks for 80MHz, but a sdkconfig.m5core generated before
  // that keeps its old clock. The peripheral clock is 80MHz at any CPU clock
  // of 80MHz or more, so switching here changes nothing for UART, I2C or RMT.
  if (getCpuFrequencyMhz() != 80) setCpuFrequencyMhz(80);
#endif
  Serial.printf("Power: no light sleep on this build, CPU %uMHz\n", (unsigned)getCpuFrequencyMhz());
#endif
}

// Light sleep switches off USB, which would kill the serial monitor and make
// uploads awkward. So while a computer's USB port is attached, stay awake.
// On a power bank or battery there's no USB host, and the chip sleeps.
static void updateUsbLock() {
#if CONFIG_PM_ENABLE && CONFIG_IDF_TARGET_ESP32C3
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

#if !FAKE_SENSOR && !IMU_MPU6886
// Rotation report plus the BNO085's own dynamic calibration of all three
// sensors. Both are lost when the BNO085 resets, so this runs after every
// (re)start. The saved calibration (DCD) is loaded by the BNO085 at boot.
static bool enableRotation() {
  sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
  // We save it ourselves (saveDcdIfGood): only good calibrations, and rarely,
  // because the BNO085's flash wears out after some tens of thousands of erases.
  sh2_setDcdAutoSave(false);
  bnoAccuracy = 0;
  return bno.enableReport(ROTATION_REPORT, REPORT_INTERVAL_US);
}

// While running: at "high" accuracy, at most every 30 minutes. On the way to
// sleep (end of a session): at "medium" or better, if the last save was 15+
// minutes ago. A few saves per flying day.
static void saveDcdIfGood(bool sleeping) {
  if (bnoAccuracy < (sleeping ? 2 : 3)) return;
  uint32_t minGap = sleeping ? 15 * 60000UL : 30 * 60000UL;
  if (lastDcdSaveMs != 0 && millis() - lastDcdSaveMs < minGap) return;
  if (sh2_saveDcdNow() == SH2_OK) {
    lastDcdSaveMs = millis();
    Serial.printf("BNO085 calibration saved (accuracy %u)\n", bnoAccuracy);
  }
}
#endif

static bool startSensor() {
#if FAKE_SENSOR
  sensorEpoch++;
  sensorStartMs = millis();
  sensorAwake = true;
  return true;
#elif IMU_MPU6886
  if (!imu.begin(Wire)) return false;
  if (bmm.begin(Wire)) Serial.printf("BMM150 at 0x%02x\n", bmm.address());
  else Serial.println("BMM150 not found, yaw will drift");
  fusion.reset();
  sensorEpoch++;
  sensorStartMs = millis();
  sensorAwake = true;
  imuSkip = 10;
  return true;
#else
  // Clone boards ship with ADO either low (0x4A) or high (0x4B); try both.
  if (bno.begin_I2C(0x4A, &Wire) || bno.begin_I2C(0x4B, &Wire)) {
    sensorEpoch++;
    sensorStartMs = millis();
    sensorAwake = enableRotation();
    return sensorAwake;
  }
  return false;
#endif
}

// The BNO085 draws several mA while fusing, so park it when nobody listens.
static void sensorSleep(bool sleep) {
  if (sleep == !sensorAwake) return;
  if (sleep) {
#if IMU_MPU6886
    imu.sleep(true);
    bmm.suspend();
    if (fusion.biasChanged) { saveBias(); fusion.biasChanged = false; }
#elif !FAKE_SENSOR
    saveDcdIfGood(true);
    sh2_devSleep();
#endif
  } else {
#if IMU_MPU6886
    imu.sleep(false);
    bmm.begin(Wire);
    fusion.reset();
    imuSkip = 10;  // ~50ms for the gyro to settle
#elif !FAKE_SENSOR
    sh2_devOn();
    enableRotation();
#endif
    sensorEpoch++;
    sensorStartMs = millis();
  }
  sensorAwake = !sleep;
  Serial.println(sleep ? "Sensor sleeping (nobody listening)" : "Sensor awake");
}

static void startBle() {
  NimBLEDevice::init(DEVICE_NAME);
#if IMU_MPU6886
  // The M5Stack's antenna is decent and the PC is a metre or two away: 0dBm
  // instead of +9dBm saves some battery. DEFAULT covers the connections.
  NimBLEDevice::setPower(ESP_PWR_LVL_N0, ESP_BLE_PWR_TYPE_DEFAULT);
  NimBLEDevice::setPower(ESP_PWR_LVL_N0, ESP_BLE_PWR_TYPE_ADV);
  Serial.printf("BLE TX power: level %d default, %d adv (0..7 = -12..+9dBm)\n",
                esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT), esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV));
#else
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // the Super Mini's antenna is weak
#endif
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
#if IMU_MPU6886
  (void)connected;
  return 20;  // no status LED on the M5Stack Core; poll the buttons and the LED bar instead
#else
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
#endif
}

#if IMU_MPU6886
// ---- LED bar (M5GO/Fire base): short, dim animations, dark otherwise ----
// Our own RMT sender instead of Adafruit_NeoPixel::show(). On IDF 4 that one
// gives the RMT a single 64-bit memory block (2.7 LEDs) and refills it from an
// interrupt as it goes; an interrupt more than ~40us late (BLE, I2C, the 80MHz
// clock) stalls the line, the LEDs latch, and the rest of the frame starts
// over at LED 0 shifted by a byte: the middle LEDs' blue showed up as red on
// the left end. Here the whole frame (240 bits + end marker) fits in four RMT
// blocks, so it goes out in one piece whatever the interrupts do.
// Adafruit_NeoPixel stays for its Color()/ColorHSV() helpers.
class LedBar {
 public:
  void begin() {
    rmt_config_t c = RMT_DEFAULT_CONFIG_TX((gpio_num_t)PIN_LEDBAR, kChannel);
    c.clk_div = 2;          // 40MHz: 25ns ticks
    c.mem_block_num = 4;    // 256 items, channels 1-3 lend theirs
    ok_ = rmt_config(&c) == ESP_OK && rmt_driver_install(kChannel, 0, 0) == ESP_OK;
    if (!ok_) Serial.println("LED bar: RMT setup failed");
  }
  void setPixelColor(int i, uint32_t c) { if (i >= 0 && i < NUM_LEDS) px_[i] = c; }
  void clear() { memset(px_, 0, sizeof(px_)); }
  void show() {
    if (!ok_) return;
    // SK6812/WS2812 at 800kHz, GRB order, MSB first (same timings as Adafruit's).
    const rmt_item32_t bit0 = {{{16, 1, 34, 0}}};  // 0.40us high, 0.85us low
    const rmt_item32_t bit1 = {{{32, 1, 18, 0}}};  // 0.80us high, 0.45us low
    size_t n = 0;
    for (int i = 0; i < NUM_LEDS; ++i) {
      uint32_t c = px_[i];
      uint32_t grb = ((c >> 8) & 0xFF) << 16 | ((c >> 16) & 0xFF) << 8 | (c & 0xFF);
      for (int b = 23; b >= 0; --b) items_[n++] = (grb >> b) & 1 ? bit1 : bit0;
    }
    items_[n++] = rmt_item32_t{};  // end marker
    rmt_write_items(kChannel, items_, n, true);  // ~0.3ms, waits until sent
  }

 private:
  static constexpr rmt_channel_t kChannel = RMT_CHANNEL_0;
  uint32_t px_[NUM_LEDS] = {};
  rmt_item32_t items_[NUM_LEDS * 24 + 1];
  bool ok_ = false;
};
static_assert(NUM_LEDS * 24 + 1 <= 4 * 64, "LED frame must fit the RMT blocks");
static LedBar strip;
static Anim anim = Anim::None;
static uint32_t animStart = 0;
static uint32_t ledFrame[NUM_LEDS];
static bool ledsDirty = true;

static void ledAnim(Anim a) {
  anim = a;
  animStart = millis();
}

static uint32_t rgb(float r, float g, float b, float level) {  // 0..1 each, kept dim
  auto c = [&](float v) { return (uint8_t)fminf(255.0f, fmaxf(0.0f, v * level * 48.0f)); };
  return Adafruit_NeoPixel::Color(c(r), c(g), c(b));
}

// LEDs within `half` of the bar's middle (0 = none, 5 = all), soft edge.
static float fromCenter(int i, float half) {
  float d = fabsf(i - (NUM_LEDS - 1) / 2.0f) + 0.5f;
  return fmaxf(0.0f, fminf(1.0f, half - d + 1.0f));
}

static void setFrame(int i, uint32_t c) {
  if (ledFrame[i] != c) { ledFrame[i] = c; ledsDirty = true; }
}

enum class Battery : uint8_t { Ok, Low, Empty };  // see updateBattery()
static Battery battery = Battery::Ok;

static const uint32_t BLIP_ON_MS = 40;  // status blips: 2 frames at 50 frames/s

static void updateLeds(bool waiting, float calProgress) {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < 20) return;  // 50 frames/s is plenty
  lastMs = millis();
  uint32_t t = millis() - animStart;
  for (int i = 0; i < NUM_LEDS; ++i) {
    uint32_t c = 0;
    switch (anim) {
      case Anim::PowerOn: {  // rainbow sweeping along, fading out after 0.8s
        float level = t < 800 ? fminf(1.0f, t / 200.0f) : 1.0f - (t - 800) / 500.0f;
        c = Adafruit_NeoPixel::ColorHSV((uint16_t)(i * 6553 + t * 80), 255, 255);
        c = rgb(((c >> 16) & 0xFF) / 255.0f, ((c >> 8) & 0xFF) / 255.0f, (c & 0xFF) / 255.0f, level);
        if (t >= 1300) anim = Anim::None;
        break;
      }
      case Anim::Connected: {  // green from the middle outward, then fade
        float level = fromCenter(i, t / 60.0f) * (t < 400 ? 1.0f : 1.0f - (t - 400) / 400.0f);
        c = rgb(0.1f, 1, 0.2f, level);
        if (t >= 800) anim = Anim::None;
        break;
      }
      case Anim::Disconnected: {  // amber closing in toward the middle
        c = rgb(1, 0.45f, 0, fromCenter(i, 5.0f - t / 80.0f));
        if (t >= 450) anim = Anim::None;
        break;
      }
      case Anim::Recenter:  // quick white flash
        c = rgb(1, 1, 1, 0.6f * (1.0f - t / 250.0f));
        if (t >= 250) anim = Anim::None;
        break;
      case Anim::CalOk:
      case Anim::CalFail: {  // two flashes
        bool on = t % 300 < 150;
        c = on ? (anim == Anim::CalOk ? rgb(0, 1, 0.1f, 1) : rgb(1, 0, 0, 1)) : 0;
        if (t >= 600) anim = Anim::None;
        break;
      }
      case Anim::None:
        if (calProgress >= 0) {  // compass calibration: cyan bar filling over the 20s
          float fill = calProgress * NUM_LEDS - i;
          c = rgb(0, 0.8f, 1, fill >= 1 ? 0.5f : fill > 0 ? 0.5f * fill + 0.5f : 0.0f);
        } else if (waiting && millis() % 3000 < BLIP_ON_MS && (i == 4 || i == 5)) {
          c = rgb(0, 0.2f, 1, 0.2f);  // waiting for X-Plane: short, dim blue blip
        }
        // Low battery: red double blink at both ends every 3s. Nearly empty:
        // the whole bar blinks red every second. Short and dim, like the blue.
        if (calProgress < 0) {
          uint32_t b3 = millis() % 3000;
          bool lowOn = (b3 < BLIP_ON_MS || (b3 >= 200 && b3 < 200 + BLIP_ON_MS)) && (i <= 1 || i >= NUM_LEDS - 2);
          bool emptyOn = millis() % 1000 < BLIP_ON_MS;
          if (battery == Battery::Low && lowOn) c = rgb(1, 0, 0, 0.4f);
          if (battery == Battery::Empty && emptyOn) c = rgb(1, 0, 0, 0.4f);
        }
        break;
    }
    setFrame(i, c);
  }
  if (!ledsDirty) return;
  for (int i = 0; i < NUM_LEDS; ++i) strip.setPixelColor(i, ledFrame[i]);
  strip.show();
  ledsDirty = false;
}

// IP5306 power chip (I2C 0x75): read one register.
static bool ip5306Read(uint8_t reg, uint8_t &v) {
  Wire.beginTransmission(0x75);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(0x75, 1) != 1) return false;
  v = Wire.read();
  return true;
}

// Battery: the IP5306 only knows its four fuel-gauge steps (REG 0x78 high
// nibble 0x0/0x8/0xC/0xE = 100/75/50/25%, anything else = nearly empty), so
// "low" means down to its last step, about 25%. Checked every 10s; three low
// readings in a row (30s) turn the warning on. It stays on until external
// power shows up (REG 0x70 bit3), because the voltage, and with it the gauge,
// bounces back up whenever the load drops.
static void updateBattery() {
  static uint32_t last = 0;
  static uint8_t lowCount = 0, emptyCount = 0;
  if (last != 0 && millis() - last < 10000) return;
  last = millis();
  uint8_t ctl, gauge;
  if (!ip5306Read(0x70, ctl) || !ip5306Read(0x78, gauge)) return;
  if (ctl & 0x08) {  // on USB power (charging or full)
    if (battery != Battery::Ok) Serial.println("Battery: external power, warning off");
    battery = Battery::Ok;
    lowCount = emptyCount = 0;
    return;
  }
  uint8_t step = gauge >> 4;
  bool low = step != 0x0 && step != 0x8 && step != 0xC;  // 25% step or below
  bool empty = low && step != 0xE;
  lowCount = low ? (lowCount < 3 ? lowCount + 1 : 3) : 0;
  emptyCount = empty ? (emptyCount < 3 ? emptyCount + 1 : 3) : 0;
  Battery b = battery;
  if (emptyCount >= 3) b = Battery::Empty;
  else if (lowCount >= 3 && b == Battery::Ok) b = Battery::Low;
  if (b != battery) Serial.printf("Battery: %s (gauge 0x%02x)\n", b == Battery::Empty ? "nearly empty" : "low", gauge);
  battery = b;
}

// Speaker reminder while the battery warning is on: a blip right away, then
// every 5 minutes while low, every 10s once nearly empty.
static void batteryBlips() {
  static Battery last = Battery::Ok;
  static uint32_t lastBlip = 0;
  if (battery == Battery::Ok) { last = battery; return; }
  uint32_t period = battery == Battery::Empty ? 10000 : 300000;
  if (battery != last || millis() - lastBlip >= period) {
    if (blipStart(BLIP_AMPLITUDE)) {  // a test blip still playing: try again next loop
      lastBlip = millis();
      last = battery;
    }
  }
}

// IP5306 SYS_CTL0 bit1: keep the 5V boost on even at light load.
// Off, it cuts the board's power ~30s after the load drops (that's how we
// power off on battery: deep sleep draws next to nothing).
static void ip5306KeepOn(bool on) {
  uint8_t v;
  if (!ip5306Read(0x00, v)) return;
  v = on ? (v | 0x02) : (v & ~0x02);
  Wire.beginTransmission(0x75);
  Wire.write(0x00);
  Wire.write(v);
  Wire.endTransmission();
}

// ---- Compass calibration ----
static void loadMagCal() {
  MagCal c;
  prefs.begin(PREFS_NS, true);
  bool ok = prefs.getBytes("magcal", &c, sizeof(c)) == sizeof(c);
  prefs.end();
  magCal = ok && c.valid() ? c : MagCal();
  Serial.println(magCal.valid() ? "Compass calibration loaded" : "No compass calibration (press button B)");
}

static bool saveMagCal() {
  if (!prefs.begin(PREFS_NS, false)) return false;
  bool ok = magCal.valid() ? prefs.putBytes("magcal", &magCal, sizeof(magCal)) == sizeof(magCal)
                           : (!prefs.isKey("magcal") || prefs.remove("magcal"));
  prefs.end();
  return ok;
}

static void startMagCal() {
  if (magCalibrator.active) return;
  if (!sensorOk || !bmm.found()) {
    reply(CMD_MAG_CAL, CAL_NO_MAG);
    ledAnim(Anim::CalFail);
    Serial.println("Compass calibration: no magnetometer");
    return;
  }
  magCalibrator.start();
  Serial.println("Compass calibration: turn the tracker every way for 20s");
}

static void forgetMagCal() {
  magCal = MagCal();
  saveMagCal();
  fusion.resetMag();
  Serial.println("Compass calibration cleared");
}

static void finishMagCal() {
  MagCal c;
  uint8_t r = magCalibrator.finish(c);
  if (r == CAL_OK) {
    magCal = c;
    fusion.resetMag();
    if (!saveMagCal()) r = CAL_SAVE_FAILED;
    Serial.printf("Compass calibration: field %.1fuT, offset %.1f %.1f %.1f, scale %.3f %.3f %.3f, axes",
                  c.field, c.offset[0], c.offset[1], c.offset[2], c.scale[0], c.scale[1], c.scale[2]);
    for (int i = 0; i < 9; ++i) Serial.printf(" %d", c.P[i]);
    Serial.println();
  }
  Serial.printf("Compass calibration: result %u\n", r);
  reply(CMD_MAG_CAL, r);
  ledAnim(r == CAL_OK || r == CAL_SAVE_FAILED ? Anim::CalOk : Anim::CalFail);
}

// Every IMU sample: feed the calibrator or the fusion with the BMM150 (30Hz).
static void updateMag() {
  if (!bmm.found()) return;
  if (magCalibrator.active) magCalibrator.addGyro(fusion.lastGyro());
  static uint8_t div = 0;
  if (++div < 6) return;  // the BMM150 has a new reading every 33ms
  V raw;
  if (!bmm.read(raw)) return;  // not ready yet: try again next sample
  div = 0;
  uint32_t now = micros();
  float dt = (now - lastMagUs) * 1e-6f;
  lastMagUs = now;
  if (magCalibrator.active) magCalibrator.addMag(raw, dt);
  else if (magCal.valid()) fusion.updateMag(magCal.apply(raw), dt, magCal.field);
}

// ---- Power off ----
static void powerOff(const char *why) {
  Serial.printf("Powering off: %s\n", why);
  blipStop(true);
  if (fusion.biasChanged) saveBias();
  imu.sleep(true);
  bmm.suspend();
  for (uint32_t t = 0; t <= 600; t += 20) {  // red shrinking to the middle
    for (int i = 0; i < NUM_LEDS; ++i) strip.setPixelColor(i, rgb(1, 0, 0, fromCenter(i, 5.0f - t / 120.0f)));
    strip.show();
    delay(20);
  }
  strip.clear();
  strip.show();
  pinMode(PIN_LEDBAR, OUTPUT);
  digitalWrite(PIN_LEDBAR, LOW);
  gpio_hold_en((gpio_num_t)PIN_LEDBAR);
  // Keep the speaker and backlight pins low through deep sleep.
  gpio_hold_en((gpio_num_t)PIN_SPEAKER);
  gpio_hold_en((gpio_num_t)PIN_LCD_BL);
  gpio_deep_sleep_hold_en();
  ip5306KeepOn(false);  // on battery the IP5306 now cuts power within ~30s
  while (digitalRead(PIN_RECENTER) == LOW) delay(10);  // or button A would wake it at once
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_RECENTER, 0);  // on USB power: button A wakes it
  Serial.flush();
  esp_deep_sleep_start();
}

// Buttons, LEDs and the power-off timers, every loop.
static void m5Housekeeping() {
  bool connected = server->getConnectedCount() > 0;
  static bool wasConnected = false;
  if (connected != wasConnected) ledAnim(connected ? Anim::Connected : Anim::Disconnected);
  wasConnected = connected;
  float calProgress = magCalibrator.active ? fminf(1.0f, magCalibrator.elapsedMs() / (float)MagCalibrator::kDurationMs) : -1.0f;
  updateBattery();
  batteryBlips();
  blipStop(false);
  updateLeds(!connected, calProgress);
  static uint8_t bCount = 0;
  static bool bWas = false;
  bCount = digitalRead(PIN_MAGCAL) == LOW ? (bCount < 3 ? bCount + 1 : 3) : 0;
  bool b = bCount >= 3;
  if (b && !bWas) {
    startMagCal();
    noConnSince = millis();
  }
  bWas = b;
  if (digitalRead(PIN_RECENTER) == LOW) noConnSince = millis();  // any button = still in use

  if (magCalibrator.active || otaInProgress()) {
    noConnSince = millis();
    return;
  }
  if (server->getConnectedCount() > 0) noConnSince = millis();
  else if (millis() - noConnSince > OFF_NO_CONN_MS) powerOff("no connection for a minute");
  if (sensorAwake && fusion.stillSeconds >= OFF_STILL_S) powerOff("lying still for a minute");
}

static void m5Setup() {
  ip5306KeepOn(true);  // or on battery it may switch the board off at light load
  // List what's on the internal bus (MPU6886 0x68, BMM150 0x10, IP5306 0x75).
  Serial.print("I2C:");
  for (uint8_t a = 1; a < 127; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf(" 0x%02x", a);
  }
  Serial.println();
}
#endif

void setup() {
  Serial.begin(115200);
  setupPower();
#if IMU_MPU6886
  gpio_hold_dis((gpio_num_t)PIN_SPEAKER);  // held low through deep sleep
  gpio_hold_dis((gpio_num_t)PIN_LCD_BL);
  gpio_hold_dis((gpio_num_t)PIN_LEDBAR);
  gpio_deep_sleep_hold_dis();
  strip.begin();
  strip.clear();
  strip.show();
  ledAnim(Anim::PowerOn);
  pinMode(PIN_RECENTER, INPUT);
  pinMode(PIN_MAGCAL, INPUT);
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, LOW);
  pinMode(PIN_SPEAKER, OUTPUT);
  digitalWrite(PIN_SPEAKER, LOW);
  cmdQueue = xQueueCreate(8, sizeof(uint8_t));
  loadMount();
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  m5Setup();
  loadBias();
  loadMagCal();
  esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();
  if (wake == ESP_SLEEP_WAKEUP_EXT0) Serial.println("Woken by button A");
  noConnSince = millis();
#else
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
#endif

  // Bluetooth first, so a firmware update still works if the sensor is broken.
  startBle();
  Serial.printf("BLE advertising as %s\n", DEVICE_NAME);
  if (FAKE_SENSOR) Serial.println("FAKE SENSOR build: no BNO085, sending simulated head motion");
}

void loop() {
  otaLoop();
  updateUsbLock();
  uint32_t ledDue = updateLed(server->getConnectedCount() > 0);
  uint8_t cmd;
  while (xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE) handleCommand(cmd);
#if IMU_MPU6886
  m5Housekeeping();
#endif

  if (!sensorOk) {
    static uint32_t lastTry = 0;
    if (lastTry == 0 || millis() - lastTry > 1000) {
      lastTry = millis();
      sensorOk = startSensor();
      Serial.println(sensorOk ? (FAKE_SENSOR ? "Fake sensor ready" : IMU_MPU6886 ? "MPU6886 ready" : "BNO085 ready")
                              : "Sensor not found, check wiring. Retrying...");
    }
    vTaskDelay(pdMS_TO_TICKS(ledDue < 250 ? ledDue : 250));
    return;
  }

#if !FAKE_SENSOR && !IMU_MPU6886
  if (bno.wasReset()) {
    Serial.println("BNO085 reset, re-enabling report");
    sensorEpoch++;
    sensorStartMs = millis();
    sensorAwake = enableRotation();
  }
#endif

  // Keep the sensor running for a while after the last listener left, so a
  // short Bluetooth drop doesn't restart it (that would lose the center).
  static uint32_t lastListened = 0;
  bool subscribed = quatChar->getSubscribedCount() > 0 && !otaInProgress();
  if (subscribed) lastListened = millis();
#if IMU_MPU6886
  if (magCalibrator.active) lastListened = millis();  // keep the sensors running for it
#endif
  bool listening = subscribed || (sensorAwake && !otaInProgress() && millis() - lastListened < SENSOR_LINGER_MS);
  sensorSleep(!listening);
  // The sensor stopped reporting while someone listens (a loose wire, or the
  // BNO08x's I2C hang): hard-reset it through RST. That starts a new epoch, so
  // the view recenters once it's back.
  if (listening) {
    uint32_t since = haveSample && (int32_t)(lastSampleMs - sensorStartMs) > 0 ? lastSampleMs : sensorStartMs;
    if (millis() - since > 1000) {
      Serial.println("Sensor stopped reporting, restarting it");
      sensorOk = false;
      haveSample = false;
      return;
    }
  }
  if (!listening) {
    // INT also wakes the chip from light sleep. If the sensor has something
    // queued (a reply to the sleep command, a last sample), INT stays low until
    // it's read, and the chip would never get to sleep. So read it out.
#if !FAKE_SENSOR && !IMU_MPU6886
    for (int i = 0; i < 8 && digitalRead(PIN_BNO_INT) == LOW; ++i) bno.getSensorEvent(&sensorValue);
#endif
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
#elif IMU_MPU6886
  // Read the IMU every 5ms and fuse; send every second sample (100Hz).
  static TickType_t nextWake = xTaskGetTickCount();
  if ((int32_t)(xTaskGetTickCount() - nextWake) > (int32_t)pdMS_TO_TICKS(100)) nextWake = xTaskGetTickCount();
  vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(SAMPLE_INTERVAL_US / 1000));
  ImuSample smp;
  if (!imu.read(smp)) return;  // a lasting failure trips the "stopped reporting" restart
  uint32_t nowUs = micros();
  float dt = (nowUs - lastImuUs) * 1e-6f;
  lastImuUs = nowUs;
  if (imuSkip) { imuSkip--; return; }
  if (dt <= 0 || dt > 0.05f) dt = SAMPLE_INTERVAL_US * 1e-6f;
  fusion.update(smp, dt);
  updateMag();
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus >= 5000) {
    lastStatus = millis();
    const float k = 180.0f / (float)M_PI;
    V b = fusion.bias(), m = fusion.lastMean;
    Serial.printf("imu: wobble %.2fdps, g range %.3fg, still %us, mean %.2f %.2f %.2fdps, bias %.2f %.2f %.2fdps%s, %.1fC, compass %s\n",
                  fusion.lastStd * k, fusion.lastARange, (unsigned)fusion.stillSeconds, m.x * k, m.y * k, m.z * k,
                  b.x * k, b.y * k, b.z * k, fusion.biasFresh ? "" : " (old)", smp.tempC,
                  fusion.magActive() ? "on" : (magCal.valid() ? "waiting" : "not calibrated"));
  }
  if (magCalibrator.timeUp()) finishMagCal();
  static bool savedThisRun = false;  // save the first fresh bias right away, later ones on sleep
  if (fusion.biasChanged && !savedThisRun) { saveBias(); fusion.biasChanged = false; savedThisRun = true; }
  static bool sendThis = false;
  sendThis = !sendThis;
  if (!sendThis) return;
  Q fq = fusion.q();
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
    if (sensorValue.sensorId == ROTATION_REPORT) {
      q[0] = sensorValue.un.arvrStabilizedRV.real;
      q[1] = sensorValue.un.arvrStabilizedRV.i;
      q[2] = sensorValue.un.arvrStabilizedRV.j;
      q[3] = sensorValue.un.arvrStabilizedRV.k;
      bnoAccuracy = sensorValue.status & 0x03;
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
  saveDcdIfGood(false);
#endif

  Q sensorQ = qnorm(Q{q[0], q[1], q[2], q[3]});
  lastSensorQ = sensorQ;
  lastSampleMs = millis();
  haveSample = true;

  // Recenter on the BOOT button, on request, and whenever the sensor has
  // (re)started, because its yaw reference starts over then.
  // Pressed only after 3 low reads in a row (30ms): GPIO39 on the classic
  // ESP32 can glitch low for a moment while the radio is on.
  static bool lastButton = false;
  static uint8_t lowCount = 0;
  lowCount = digitalRead(PIN_RECENTER) == LOW ? (lowCount < 3 ? lowCount + 1 : 3) : 0;
  bool button = lowCount >= 3;
  if (button && !lastButton) {
    recenterPending = true;
#if IMU_MPU6886
    ledAnim(Anim::Recenter);
#endif
  }
  lastButton = button;
  if (recenterPending || !haveRef || refEpoch != sensorEpoch) setReference(sensorQ);

  Angles a = relativeTo(headAngles(qmul(sensorQ, mountRot)), headRef);
  float out[3] = {a.yaw, a.pitch, a.roll};
  uint8_t pkt[14];
  memcpy(pkt, out, sizeof(out));  // ESP32 is little-endian
#if IMU_MPU6886
  bool compass = fusion.magActive();
#elif FAKE_SENSOR
  bool compass = false;
#else
  bool compass = bnoAccuracy >= 2;
#endif
  pkt[12] = (button ? 0x01 : 0x00) | (mountCalibrated ? 0x02 : 0x00) | (compass ? 0x04 : 0x00) |
            uint8_t((recenterCount & 0x0F) << 4);
  pkt[13] = seq++;
  if (!subscribed) return;  // lingering after a drop: keep the sensor going, nobody to tell
  quatChar->setValue(pkt, sizeof(pkt));
  quatChar->notify();
}
