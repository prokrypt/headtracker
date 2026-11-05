// M5Stack Core sensors (MPU6886 accelerometer + gyro, BMM150 magnetometer)
// with our own sensor fusion, standing in for the BNO085.
//
// The BNO085 fuses on the chip and tracks its gyro bias by itself. These give
// raw readings only, so here we do both:
//   * Mahony filter: integrates the gyro, and pulls tilt (pitch/roll) toward
//     gravity from the accelerometer.
//   * Gyro bias: measured whenever the tracker holds still for 1s, blended in
//     slowly, and kept in flash so the next start begins close.
//   * Yaw from the BMM150, once it's calibrated (MagCalibrator): only the
//     heading is used, pulled in slowly (~10s), and only while the field looks
//     like the one at calibration (strength and dip). A speaker or a steel
//     frame near the head then gets ignored instead of turning the view.
//     Without a calibration, yaw drifts slowly like the BNO085's Game RV.
// Output: orientation sensor -> world, world z up, yaw 0 at start. Same
// convention as the Game Rotation Vector, so mount.h works unchanged.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "mount.h"

static const float kD2R = (float)M_PI / 180.0f;

struct ImuSample {
  V gyro;    // rad/s
  V accel;   // g
  float tempC;
};

// Register access for both chips.
class I2cDev {
 protected:
  TwoWire *wire = nullptr;
  uint8_t addr = 0;
  bool writeReg(uint8_t reg, uint8_t v) {
    wire->beginTransmission(addr);
    wire->write(reg);
    wire->write(v);
    return wire->endTransmission() == 0;
  }
  bool readRegs(uint8_t reg, uint8_t *buf, size_t n) {
    wire->beginTransmission(addr);
    wire->write(reg);
    if (wire->endTransmission(false) != 0) return false;
    if (wire->requestFrom((int)addr, (int)n) != (int)n) return false;
    for (size_t i = 0; i < n; ++i) buf[i] = wire->read();
    return true;
  }
};

class Mpu6886 : I2cDev {
 public:
  bool begin(TwoWire &w) {
    wire = &w;
    addr = 0x68;
    uint8_t id = 0;
    if (!readRegs(0x75, &id, 1) || id != 0x19) return false;  // WHO_AM_I
    if (!writeReg(0x6B, 0x80)) return false;                  // PWR_MGMT_1: reset
    delay(20);
    bool ok = writeReg(0x6B, 0x01)      // wake, best clock source
              && writeReg(0x6C, 0x00)   // PWR_MGMT_2: all axes on
              && writeReg(0x1A, 0x03)   // CONFIG: gyro low-pass 41Hz, 1kHz internal rate
              && writeReg(0x19, 0x00)   // SMPLRT_DIV: 1kHz (we read every 5ms)
              && writeReg(0x1B, 0x10)   // GYRO_CONFIG: +-1000dps, low-pass on
              && writeReg(0x1C, 0x08)   // ACCEL_CONFIG: +-4g
              && writeReg(0x1D, 0x03)   // ACCEL_CONFIG2: accel low-pass 44.8Hz
              && writeReg(0x23, 0x00)   // FIFO off
              && writeReg(0x38, 0x00);  // no interrupts
    delay(50);  // gyro start-up
    return ok;
  }

  // Sleep cuts it to a few uA; waking needs ~35ms before the gyro is good.
  bool sleep(bool on) { return writeReg(0x6B, on ? 0x41 : 0x01); }

  bool read(ImuSample &s) {
    uint8_t b[14];
    if (!readRegs(0x3B, b, sizeof(b))) return false;
    auto i16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
    const float g = 1.0f / 8192.0f;      // +-4g
    const float r = kD2R / 32.8f;        // +-1000dps
    s.accel = V{i16(0) * g, i16(2) * g, i16(4) * g};
    s.tempC = i16(6) / 326.8f + 25.0f;
    s.gyro = V{i16(8) * r, i16(10) * r, i16(12) * r};
    return true;
  }
};

// Bosch BMM150, normal mode at 30Hz, "regular" repetitions. Readings are
// temperature-compensated with the chip's trim values (Bosch's float formulas).
class Bmm150 : I2cDev {
 public:
  bool begin(TwoWire &w) {
    wire = &w;
    for (uint8_t a = 0x10; a <= 0x13; ++a) {
      addr = a;
      if (!writeReg(0x4B, 0x01)) continue;  // power on (sleep mode)
      delay(3);
      uint8_t id = 0;
      if (readRegs(0x40, &id, 1) && id == 0x32) return configure();
    }
    addr = 0;
    return false;
  }
  bool found() const { return addr != 0; }
  uint8_t address() const { return addr; }

  // Suspend: below 1uA. begin() brings it back.
  void suspend() { if (addr) writeReg(0x4B, 0x00); }

  // false if no new reading yet (or a bus error / overflow).
  bool read(V &uT) {
    uint8_t b[8];
    if (!readRegs(0x42, b, sizeof(b)) || !(b[6] & 0x01)) return false;
    int16_t x = (int16_t)((b[1] << 8) | b[0]) >> 3;
    int16_t y = (int16_t)((b[3] << 8) | b[2]) >> 3;
    int16_t z = (int16_t)((b[5] << 8) | b[4]) >> 1;
    uint16_t rh = ((b[7] << 8) | b[6]) >> 2;
    if (x == -4096 || y == -4096 || z == -16384) return false;
    uT = V{compXY(x, rh, digX1, digX2), compXY(y, rh, digY1, digY2), compZ(z, rh)};
    return isfinite(uT.x) && isfinite(uT.y) && isfinite(uT.z);
  }

 private:
  int8_t digX1, digY1, digX2, digY2, digXY2;
  uint8_t digXY1;
  int16_t digZ2, digZ3, digZ4;
  uint16_t digZ1, digXYZ1;

  bool configure() {
    if (!writeReg(0x4B, 0x83)) return false;  // soft reset, stays powered
    delay(3);
    uint8_t t[0x72 - 0x5D];
    if (!readRegs(0x5D, t, sizeof(t))) return false;
    auto at = [&](uint8_t reg) { return t[reg - 0x5D]; };
    auto u16 = [&](uint8_t reg) { return (uint16_t)(at(reg) | (at(reg + 1) << 8)); };
    digX1 = (int8_t)at(0x5D);
    digY1 = (int8_t)at(0x5E);
    digZ4 = (int16_t)u16(0x62);
    digX2 = (int8_t)at(0x64);
    digY2 = (int8_t)at(0x65);
    digZ2 = (int16_t)u16(0x68);
    digZ1 = u16(0x6A);
    digXYZ1 = u16(0x6C) & 0x7FFF;
    digZ3 = (int16_t)u16(0x6E);
    digXY2 = (int8_t)at(0x70);
    digXY1 = at(0x71);
    return writeReg(0x51, 0x04)     // REPXY: 9 repetitions
           && writeReg(0x52, 0x0E)  // REPZ: 15 repetitions
           && writeReg(0x4C, 0x38); // 30Hz, normal mode
  }

  float compXY(int16_t raw, uint16_t rhall, int8_t d1, int8_t d2) {
    float x0 = rhall ? rhall : digXYZ1;
    if (x0 == 0) return NAN;
    float r = (float)digXYZ1 * 16384.0f / x0 - 16384.0f;
    float x5 = digXY2 * (r * r / 268435456.0f) + r * digXY1 / 16384.0f;
    float x8 = (x5 + 256.0f) * (raw * (d2 + 160.0f));
    return (x8 / 8192.0f + d1 * 8.0f) / 16.0f;
  }
  float compZ(int16_t raw, uint16_t rhall) {
    if (!digZ2 || !digZ1 || !digXYZ1 || !rhall) return NAN;
    float z0 = (float)raw - digZ4;
    float z2 = (float)digZ3 * ((float)rhall - digXYZ1);
    float z4 = digZ2 + (float)digZ1 * rhall / 32768.0f;
    return ((z0 * 131072.0f - z2) / (z4 * 4.0f)) / 16.0f;
  }
};

// Compass calibration, kept in flash.
//   sensor-frame field = P^T * ((raw - offset) * scale)
// offset/scale: magnets and iron that move with the tracker (hard/soft iron,
// axis-aligned ellipsoid fit). P: how the BMM150's axes sit relative to the
// MPU6886's (one of the 48 signed axis swaps), found from the gyro during the
// calibration, so the board layout doesn't have to be known.
struct MagCal {
  float offset[3] = {0, 0, 0};
  float scale[3] = {1, 1, 1};
  int8_t P[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};  // row-major, imu -> mag
  float field = 0;                            // uT after correction, 0 = not calibrated
  bool valid() const { return field > 5 && field < 200; }

  V apply(const V &raw) const {
    float m[3] = {(raw.x - offset[0]) * scale[0], (raw.y - offset[1]) * scale[1],
                  (raw.z - offset[2]) * scale[2]};
    return V{P[0] * m[0] + P[3] * m[1] + P[6] * m[2], P[1] * m[0] + P[4] * m[1] + P[7] * m[2],
             P[2] * m[0] + P[5] * m[1] + P[8] * m[2]};
  }
};

// Collects 20s of turning the tracker every way, then fits a MagCal.
class MagCalibrator {
 public:
  static const int kMax = 640;  // 21s at 30Hz
  static const uint32_t kDurationMs = 20000;

  void start() {
    n = 0;
    gyroSum = V{};
    gyroN = 0;
    active = true;
    startMs = millis();
  }
  bool active = false;
  bool timeUp() const { return active && millis() - startMs >= kDurationMs; }
  uint32_t elapsedMs() const { return millis() - startMs; }

  // Every IMU sample (bias-corrected gyro), between magnetometer readings.
  void addGyro(const V &w) {
    gyroSum = V{gyroSum.x + w.x, gyroSum.y + w.y, gyroSum.z + w.z};
    gyroN++;
  }
  void addMag(const V &raw, float dt) {
    if (n < kMax) {
      mag[n] = raw;
      gyro[n] = gyroN ? vscale(gyroSum, 1.0f / gyroN) : V{};
      dts[n] = dt;
      n++;
    }
    gyroSum = V{};
    gyroN = 0;
  }

  // Result codes as in CalResult (mount.h).
  uint8_t finish(MagCal &out) {
    active = false;
    if (n < 200) return CAL_NO_DATA;
    MagCal c;
    if (!fitEllipsoid(c)) return CAL_MAG_FIT;
    // Coverage: corrected directions must reach well around on every axis.
    float lo[3] = {1, 1, 1}, hi[3] = {-1, -1, -1}, err = 0;
    for (int i = 0; i < n; ++i) {
      float m[3];
      corrected(c, i, m);
      float len = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
      err += (len / c.field - 1) * (len / c.field - 1);
      for (int k = 0; k < 3; ++k) {
        lo[k] = fminf(lo[k], m[k] / len);
        hi[k] = fmaxf(hi[k], m[k] / len);
      }
    }
    for (int k = 0; k < 3; ++k)
      if (hi[k] - lo[k] < 1.2f) return CAL_MAG_COVERAGE;
    if (sqrtf(err / n) > 0.06f) return CAL_MAG_FIT;  // field not steady: something magnetic moved
    if (!findAxes(c)) return CAL_MAG_AXES;
    out = c;
    return CAL_OK;
  }

 private:
  V mag[kMax], gyro[kMax];
  float dts[kMax];
  int n = 0;
  V gyroSum;
  int gyroN = 0;
  uint32_t startMs = 0;

  void corrected(const MagCal &c, int i, float m[3]) const {
    m[0] = (mag[i].x - c.offset[0]) * c.scale[0];
    m[1] = (mag[i].y - c.offset[1]) * c.scale[1];
    m[2] = (mag[i].z - c.offset[2]) * c.scale[2];
  }

  // a x^2 + b y^2 + c z^2 + d x + e y + f z = 1, least squares.
  bool fitEllipsoid(MagCal &c) {
    double A[6][7] = {};
    for (int i = 0; i < n; ++i) {
      double x = mag[i].x, y = mag[i].y, z = mag[i].z;
      double row[6] = {x * x, y * y, z * z, x, y, z};
      for (int r = 0; r < 6; ++r) {
        for (int k = 0; k < 6; ++k) A[r][k] += row[r] * row[k];
        A[r][6] += row[r];
      }
    }
    for (int col = 0; col < 6; ++col) {  // Gauss-Jordan with partial pivoting
      int piv = col;
      for (int r = col + 1; r < 6; ++r)
        if (fabs(A[r][col]) > fabs(A[piv][col])) piv = r;
      if (fabs(A[piv][col]) < 1e-12) return false;
      for (int k = 0; k < 7; ++k) std::swap(A[col][k], A[piv][k]);
      for (int r = 0; r < 6; ++r) {
        if (r == col) continue;
        double f = A[r][col] / A[col][col];
        for (int k = col; k < 7; ++k) A[r][k] -= f * A[col][k];
      }
    }
    double p[6];
    for (int r = 0; r < 6; ++r) p[r] = A[r][6] / A[r][r];
    if (p[0] <= 0 || p[1] <= 0 || p[2] <= 0) return false;
    double g = 1 + p[3] * p[3] / (4 * p[0]) + p[4] * p[4] / (4 * p[1]) + p[5] * p[5] / (4 * p[2]);
    double rad[3];
    for (int k = 0; k < 3; ++k) {
      c.offset[k] = -p[k + 3] / (2 * p[k]);
      rad[k] = sqrt(g / p[k]);
    }
    double mean = cbrt(rad[0] * rad[1] * rad[2]);
    for (int k = 0; k < 3; ++k) c.scale[k] = mean / rad[k];
    c.field = mean;
    return c.valid();
  }

  // Which signed axis swap P makes the field turn the way the gyro says:
  // in the sensor frame, m' = m x w (w = P * gyro).
  bool findAxes(MagCal &c) {
    double best = 1e30, second = 1e30, still = 0;
    int8_t bestP[9];
    static const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
    for (int pi = 0; pi < 6; ++pi)
      for (int s = 0; s < 8; ++s) {
        int8_t P[9] = {};
        for (int r = 0; r < 3; ++r) P[r * 3 + perms[pi][r]] = (s >> r) & 1 ? -1 : 1;
        double score = 0;
        still = 0;
        for (int i = 1; i < n; ++i) {
          float m0[3], m1[3];
          corrected(c, i - 1, m0);
          corrected(c, i, m1);
          const V &g = gyro[i];
          float w[3] = {P[0] * g.x + P[1] * g.y + P[2] * g.z, P[3] * g.x + P[4] * g.y + P[5] * g.z,
                        P[6] * g.x + P[7] * g.y + P[8] * g.z};
          float dt = dts[i];
          float pred[3] = {m0[0] + (m0[1] * w[2] - m0[2] * w[1]) * dt,
                           m0[1] + (m0[2] * w[0] - m0[0] * w[2]) * dt,
                           m0[2] + (m0[0] * w[1] - m0[1] * w[0]) * dt};
          for (int k = 0; k < 3; ++k) {
            score += (m1[k] - pred[k]) * (m1[k] - pred[k]);
            still += (m1[k] - m0[k]) * (m1[k] - m0[k]);
          }
        }
        if (score < best) {
          second = best;
          best = score;
          memcpy(bestP, P, sizeof(P));
        } else if (score < second) {
          second = score;
        }
      }
    // The winner must explain the motion well and clearly beat the runner-up.
    Serial.printf("Compass axes: best %.0f, next %.0f, no-turn %.0f\n", best, second, still);
    if (best > 0.4 * still || best > 0.6 * second) return false;
    memcpy(c.P, bestP, sizeof(bestP));
    return true;
  }
};

class Fusion {
 public:
  void reset() {
    started = false;
    sinceStartS = 0;
    haveMagRef = false;
    yawRateFix = 0;
    stillSeconds = 0;
    prevStill = false;
    resetWindow();
  }
  void setBias(const V &b, bool fromFlash) {
    gbias = b;
    biasFresh = !fromFlash;
  }
  V bias() const { return gbias; }
  // New compass calibration (or none): take a new heading reference.
  void resetMag() {
    haveMagRef = false;
    yawRateFix = 0;
  }
  V lastGyro() const { return gyroCorrected; }
  // True once since reset() a still moment has given a bias at today's temperature.
  bool biasFresh = false;
  // Set when the bias got a new value; the caller clears it after saving.
  bool biasChanged = false;
  // Whole seconds the tracker has sat still in a row (for powering off).
  uint32_t stillSeconds = 0;
  // Last 1s window, for the status line: gyro wobble (max std, rad/s), gravity range (g), mean gyro.
  float lastStd = 0, lastARange = 0;
  V lastMean;
  // The compass corrected yaw within the last 2s.
  bool magActive() const { return haveMagRef && millis() - lastMagUseMs < 2000; }

  Q q() const { return quat; }

  void update(const ImuSample &s, float dt) {
    trackStill(s);
    gyroCorrected = vsub(s.gyro, gbias);
    V a = s.accel;
    float an = sqrtf(vdot(a, a));
    if (!started) {
      // Start level with gravity, yaw 0: the shortest rotation taking the
      // measured "up" (sensor frame) onto world z.
      if (an < 0.5f) return;
      V up = vscale(a, 1.0f / an);
      V axis = vcross(up, V{0, 0, 1});
      float c = up.z;
      quat = vnorm(axis) ? qnorm(Q{sqrtf((1 + c) / 2), axis.x * sqrtf((1 - c) / 2),
                                   axis.y * sqrtf((1 - c) / 2), axis.z * sqrtf((1 - c) / 2)})
                         : (c > 0 ? Q() : Q{0, 1, 0, 0});
      started = true;
      return;
    }
    sinceStartS += dt;
    V w = gyroCorrected;
    // Tilt correction from gravity, only while the accelerometer reads about
    // 1g (not during sharp head moves). Fast for the first second.
    if (fabsf(an - 1.0f) < 0.15f) {
      V meas = vscale(a, 1.0f / an);
      V est = vrot(qconj(quat), V{0, 0, 1});  // world up seen from the sensor
      float kp = sinceStartS < 1.0f ? 5.0f : 0.5f;
      V e = vcross(meas, est);
      w = V{w.x + kp * e.x, w.y + kp * e.y, w.z + kp * e.z};
    }
    // q' = q * (0, w) / 2
    Q dq = qmul(quat, Q{0, w.x, w.y, w.z});
    quat = qnorm(Q{quat.w + 0.5f * dt * dq.w, quat.x + 0.5f * dt * dq.x,
                   quat.y + 0.5f * dt * dq.y, quat.z + 0.5f * dt * dq.z});
  }

  // A calibrated magnetometer reading (sensor frame), dt since the last one.
  void updateMag(const V &m, float dt, float calField) {
    if (!started || sinceStartS < 1.0f) return;  // let tilt settle first
    float len = sqrtf(vdot(m, m));
    if (fabsf(len / calField - 1.0f) > 0.2f) return;  // disturbed
    V h = vrot(quat, m);                              // field in the world frame
    float hh = sqrtf(h.x * h.x + h.y * h.y);
    if (hh < 0.2f * len) return;                      // nearly vertical, no heading
    float heading = atan2f(h.y, h.x), dip = atan2f(h.z, hh);
    if (!haveMagRef) {
      // Our world yaw is wherever the tracker started; keep the heading it
      // saw then as the reference.
      headingRef = heading;
      dipRef = dip;
      haveMagRef = true;
      return;
    }
    if (fabsf(dip - dipRef) > 10.0f * kD2R) return;   // disturbed
    float e = heading - headingRef;
    if (e > (float)M_PI) e -= 2 * (float)M_PI;
    if (e < -(float)M_PI) e += 2 * (float)M_PI;
    if (dt > 0.2f) dt = 0.2f;
    // Proportional pull (~10s), plus an integral that learns the leftover
    // gyro drift about the vertical, so the view doesn't sit a bit off.
    yawRateFix += 0.002f * e * dt;
    const float maxFix = 3.0f * kD2R;
    yawRateFix = fmaxf(-maxFix, fminf(maxFix, yawRateFix));
    float turn = -(0.1f * e + yawRateFix) * dt;
    quat = qnorm(qmul(Q{cosf(turn / 2), 0, 0, sinf(turn / 2)}, quat));
    lastMagUseMs = millis();
  }

 private:
  Q quat;
  bool started = false;
  float sinceStartS = 0;
  V gbias, gyroCorrected;
  bool haveMagRef = false;
  float headingRef = 0, dipRef = 0, yawRateFix = 0;
  uint32_t lastMagUseMs = 0;

  // Stillness window: 200 samples = 1s at 200Hz.
  static const int kWindow = 200;
  int n = 0;
  V sum, sumSq;
  float aMin = 0, aMax = 0;
  bool prevStill = false;
  V prevMean;
  void resetWindow() {
    n = 0;
    sum = sumSq = V{};
    aMin = 1e9f;
    aMax = -1e9f;
  }

  void trackStill(const ImuSample &s) {
    const V &g = s.gyro;
    sum = V{sum.x + g.x, sum.y + g.y, sum.z + g.z};
    sumSq = V{sumSq.x + g.x * g.x, sumSq.y + g.y * g.y, sumSq.z + g.z * g.z};
    float an = sqrtf(vdot(s.accel, s.accel));
    aMin = fminf(aMin, an);
    aMax = fmaxf(aMax, an);
    if (++n < kWindow) return;
    V mean = vscale(sum, 1.0f / n);
    V var = V{sumSq.x / n - mean.x * mean.x, sumSq.y / n - mean.y * mean.y, sumSq.z / n - mean.z * mean.z};
    float maxStd = sqrtf(fmaxf(0.0f, fmaxf(var.x, fmaxf(var.y, var.z))));
    float aRange = aMax - aMin;
    lastStd = maxStd;
    lastARange = aRange;
    lastMean = mean;
    resetWindow();
    // The MPU6886's own noise is ~0.45dps (std over 1s at 41Hz bandwidth),
    // so "still" means under 1dps with gravity steady. Lying on a desk passes;
    // a head never holds that still for a whole minute (breathing, pulse).
    bool still = maxStd < 1.0f * kD2R && aRange < 0.05f;
    stillSeconds = still ? stillSeconds + 1 : 0;
    // Bias: two still seconds in a row that agree within 0.3dps. A slow head
    // turn wouldn't hold its rate that exactly.
    bool agree = still && prevStill && sqrtf(vdot(vsub(mean, prevMean), vsub(mean, prevMean))) < 0.3f * kD2R;
    V pair = vscale(V{mean.x + prevMean.x, mean.y + prevMean.y, mean.z + prevMean.z}, 0.5f);
    prevStill = still;
    prevMean = mean;
    if (!agree) return;
    mean = pair;
    prevStill = false;  // next pair starts fresh
    V diff = vsub(mean, gbias);
    if (!biasFresh) {
      gbias = mean;  // first still moment since start: take it as is
      biasFresh = true;
      biasChanged = true;
    } else if (sqrtf(vdot(diff, diff)) < 2.0f * kD2R) {
      // Later ones: blend in. A big jump is more likely a slow, steady head
      // turn than the bias moving, so ignore those.
      gbias = V{gbias.x + 0.5f * diff.x, gbias.y + 0.5f * diff.y, gbias.z + 0.5f * diff.z};
      biasChanged = true;
    }
  }
};
