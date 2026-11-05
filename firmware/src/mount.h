// Mount calibration and recentering, done on the tracker itself.
//
// The BNO085 reports how the *sensor* is oriented. How the sensor sits on the
// head can be anything, so a fixed rotation C ("the mount") turns it into how
// the *head* is oriented:   head = sensor * C
// C is measured with a two-step calibration:
//   1. look straight ahead   -> gives which sensor direction is "up" for the head
//   2. nod down (chin to chest, 20+ degrees)
//                            -> the nod turns about the head's left/right axis
// From those two, forward = up x right. C is kept in flash (NVS).
//
// Head frame: x = right, y = forward (nose), z = up.
// Recentering works on the head's yaw/pitch/roll (see headAngles), which are
// measured against gravity, so a recenter with the head a bit tilted doesn't
// make later turns come out tilted.
// The default mount (C = identity) is the board flat on the head, +Y to the nose.
#pragma once
#include <math.h>

struct Q {
  float w, x, y, z;
  Q(float w_ = 1, float x_ = 0, float y_ = 0, float z_ = 0) : w(w_), x(x_), y(y_), z(z_) {}
};
struct V {
  float x, y, z;
  V(float x_ = 0, float y_ = 0, float z_ = 0) : x(x_), y(y_), z(z_) {}
};

inline Q qmul(const Q &a, const Q &b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
inline Q qconj(const Q &q) { return {q.w, -q.x, -q.y, -q.z}; }
inline Q qnorm(const Q &q) {
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n < 1e-6f || !isfinite(n)) return Q();
  return {q.w / n, q.x / n, q.y / n, q.z / n};
}
inline V vrot(const Q &q, const V &v) {  // q v q*
  Q p = qmul(qmul(q, Q{0, v.x, v.y, v.z}), qconj(q));
  return {p.x, p.y, p.z};
}
inline float vdot(const V &a, const V &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V vcross(const V &a, const V &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline V vscale(const V &a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline V vsub(const V &a, const V &b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline bool vnorm(V &a) {
  float n = sqrtf(vdot(a, a));
  if (n < 1e-6f) return false;
  a = vscale(a, 1.0f / n);
  return true;
}

// Rotation whose matrix has the columns c0, c1, c2 (orthonormal, right-handed).
inline Q qFromColumns(const V &c0, const V &c1, const V &c2) {
  float m00 = c0.x, m10 = c0.y, m20 = c0.z;
  float m01 = c1.x, m11 = c1.y, m21 = c1.z;
  float m02 = c2.x, m12 = c2.y, m22 = c2.z;
  float tr = m00 + m11 + m22;
  Q q;
  if (tr > 0) {
    float s = sqrtf(tr + 1.0f) * 2;
    q = {0.25f * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s};
  } else if (m00 > m11 && m00 > m22) {
    float s = sqrtf(1.0f + m00 - m11 - m22) * 2;
    q = {(m21 - m12) / s, 0.25f * s, (m01 + m10) / s, (m02 + m20) / s};
  } else if (m11 > m22) {
    float s = sqrtf(1.0f + m11 - m00 - m22) * 2;
    q = {(m02 - m20) / s, (m01 + m10) / s, 0.25f * s, (m12 + m21) / s};
  } else {
    float s = sqrtf(1.0f + m22 - m00 - m11) * 2;
    q = {(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25f * s};
  }
  return qnorm(q);
}

enum CalResult : uint8_t {
  CAL_OK = 0,
  CAL_NO_DATA = 1,      // no fresh sensor sample (not connected, sensor missing)
  CAL_TOO_SMALL = 2,    // the nod was less than kMinNodDeg
  CAL_SIDEWAYS = 3,     // the head turned left/right instead of nodding
  CAL_NO_STEP1 = 4,     // step 2 without a (still valid) step 1
  CAL_SAVE_FAILED = 5,  // worked, but couldn't be written to flash
  // Compass calibration (M5Stack build)
  CAL_MAG_COVERAGE = 6, // not turned far enough in every direction
  CAL_MAG_FIT = 7,      // readings don't fit one steady field (something magnetic nearby moved?)
  CAL_MAG_AXES = 8,     // couldn't match the compass axes to the gyro (turn it more)
  CAL_NO_MAG = 9,       // no magnetometer (or the sensor isn't running)
  CAL_BUSY = 10,        // blip: one is still playing (or the timer couldn't be made)
};

static const float kMinNodDeg = 15.0f;

// q1: sensor orientation looking straight ahead; q2: nodding down.
// On success writes the mount C (head = sensor * C).
inline CalResult computeMount(const Q &q1, const Q &q2, Q &mount) {
  // "Up" for the head, in sensor coordinates. The Game Rotation Vector's
  // world z axis points up (gravity), so this is world up seen from the sensor.
  V up = vrot(qconj(q1), V{0, 0, 1});
  if (!vnorm(up)) return CAL_NO_DATA;
  // The nod, as a rotation in sensor coordinates.
  Q r = qnorm(qmul(qconj(q1), q2));
  if (r.w < 0) r = {-r.w, -r.x, -r.y, -r.z};
  float angle = 2.0f * acosf(fminf(1.0f, r.w)) * 180.0f / (float)M_PI;
  if (angle < kMinNodDeg) return CAL_TOO_SMALL;
  V axis{r.x, r.y, r.z};
  if (!vnorm(axis)) return CAL_TOO_SMALL;
  // Nodding down turns about the head's -x (left) axis, so right = -axis.
  V right = vscale(axis, -1.0f);
  // A nod turns about a horizontal axis. If the axis is far from horizontal,
  // the head mostly turned (yaw) instead.
  if (fabsf(vdot(right, up)) > 0.5f) return CAL_SIDEWAYS;
  right = vsub(right, vscale(up, vdot(right, up)));
  if (!vnorm(right)) return CAL_SIDEWAYS;
  V forward = vcross(up, right);
  mount = qFromColumns(right, forward, up);
  return CAL_OK;
}

// Head angles in degrees, X-Plane's conventions: yaw > 0 looks right,
// pitch > 0 looks up, roll > 0 tilts the right ear down.
struct Angles {
  float yaw, pitch, roll;
  Angles(float y = 0, float p = 0, float r = 0) : yaw(y), pitch(p), roll(r) {}
};

inline float wrap180(float a) {
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0) a += 360.0f;
  return a - 180.0f;
}

// From the head's orientation in the world (Game Rotation Vector frame, z up).
// Order: yaw about the vertical, then pitch, then roll (M = Rz * Rx * Ry).
inline Angles headAngles(const Q &h) {
  Q r = qnorm(h);
  float m01 = 2 * (r.x * r.y - r.w * r.z), m11 = 1 - 2 * (r.x * r.x + r.z * r.z);
  float m21 = 2 * (r.y * r.z + r.w * r.x);
  float m20 = 2 * (r.x * r.z - r.w * r.y), m22 = 1 - 2 * (r.x * r.x + r.y * r.y);
  const float kDeg = 180.0f / (float)M_PI;
  float yawCcw = atan2f(-m01, m11);
  float pitch = asinf(fmaxf(-1.0f, fminf(1.0f, m21)));
  float roll = atan2f(-m20, m22);
  return Angles(-yawCcw * kDeg, pitch * kDeg, roll * kDeg);
}

// Angles relative to the recenter pose.
inline Angles relativeTo(const Angles &a, const Angles &ref) {
  return Angles(wrap180(a.yaw - ref.yaw), a.pitch - ref.pitch, wrap180(a.roll - ref.roll));
}
