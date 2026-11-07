// Head angles come from the tracker (already recentered and corrected for how
// it's mounted). This is the neck and body-turn models on top of them.
#pragma once
#include <algorithm>
#include <cmath>

namespace ht {

struct Vec3 { double x = 0, y = 0, z = 0; };
struct Angles { double yaw = 0, pitch = 0, roll = 0; };  // degrees, X-Plane sign convention

// Neck model: the eyes sit in front of and above the neck pivot, so turning
// or tilting the head also moves them. Returns how far the eyes move (meters)
// in X-Plane's cockpit axes: x right, y up, z aft. Angles use X-Plane's head
// conventions: yaw + = look right, pitch + = look up, roll + = tilt right.
inline Vec3 neckOffset(const Angles &a, double forwardM, double upM) {
  const double d = 3.14159265358979323846 / 180.0;
  Vec3 e{0, upM, -forwardM};  // eye position relative to the pivot, head level
  // Rotate by roll (about z), then pitch (about x), then yaw (about y).
  double b = -a.roll * d, t = a.pitch * d, y = -a.yaw * d;
  Vec3 r{e.x * std::cos(b) - e.y * std::sin(b), e.x * std::sin(b) + e.y * std::cos(b), e.z};
  r = {r.x, r.y * std::cos(t) - r.z * std::sin(t), r.y * std::sin(t) + r.z * std::cos(t)};
  r = {r.x * std::cos(y) + r.z * std::sin(y), r.y, -r.x * std::sin(y) + r.z * std::cos(y)};
  return {r.x - e.x, r.y - e.y, r.z - e.z};
}

// Body turn: how far the torso turns for a given head yaw. Up to the
// threshold only the neck turns; past it, `share` of the extra yaw is taken
// by the torso (0.5 = half).
inline double torsoYaw(double headYaw, double thresholdDeg, double share) {
  double excess = std::fabs(headYaw) - thresholdDeg;
  return excess > 0 ? std::copysign(excess * share, headYaw) : 0.0;
}

// Eye shift from turning the torso by `yawDeg` around a vertical axis that
// sits `radiusM` behind the neck (an effective pivot that also stands in for
// the lean you'd do to look over your shoulder). Same axes as neckOffset.
// Because yaw is the outermost rotation, the full eye shift is simply
// neckOffset(head angles) + torsoOffset(torso yaw).
inline Vec3 torsoOffset(double yawDeg, double radiusM) {
  double y = -yawDeg * 3.14159265358979323846 / 180.0;
  // neck base at (0, 0, -r) relative to the pivot, rotated about y
  return {-radiusM * std::sin(y), 0.0, -radiusM * std::cos(y) + radiusM};
}

// Side lean: tilting the head (roll) also leans the upper body, as when you
// lean to look round a pillar. The eyes swing around a pivot `radiusM` below
// the neck: sideways by radius * sin(roll), and down a little. Same axes as
// neckOffset, roll + = tilt right = eyes move to the right of where you look:
// the lean turns with `yawDeg`, so looking back, a tilt right moves the eyes
// to the aircraft's left. Added on top of the neck model, which only swings
// them around the neck.
inline Vec3 leanOffset(double rollDeg, double yawDeg, double radiusM) {
  const double d = 3.14159265358979323846 / 180.0;
  double b = rollDeg * d, y = yawDeg * d;
  double side = radiusM * std::sin(b);
  // your right, in cockpit axes: (cos yaw, 0, sin yaw); looking right (+90) it points aft
  return {side * std::cos(y), -radiusM * (1.0 - std::cos(b)), side * std::sin(y)};
}

}  // namespace ht
