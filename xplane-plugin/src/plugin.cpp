// HeadTrackerBLE: X-Plane 12 plugin that reads the ESP32-C3 + BNO085 tracker
// over Bluetooth LE and moves the pilot's head in the 3D cockpit.
//
// The tracker itself works out how it's mounted (calibration) and recenters;
// this plugin sends it the commands and turns its head angles into the view.
//
// Commands (bind them in Settings > Keyboard / Joystick):
//   headtracker/recenter   look straight ahead now
//   headtracker/calibrate  measure how the tracker sits on your head
//   headtracker/toggle     turn tracking on/off
//   headtracker/settings   show/hide the settings window (also Plugins > Head Tracker)

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <locale>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "XPLMDataAccess.h"
#include "XPLMPlanes.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

#include "ble_link.h"
#include "app.h"
#include "head_math.h"

using namespace ht;

namespace {

Config gConfig;
std::unique_ptr<BleLink> gLink;
// Links from an earlier enable whose thread was still stuck connecting when
// the plugin was disabled. They finish by themselves; XPluginStop waits for them.
std::vector<std::unique_ptr<BleLink>> gRetiredLinks;

void pruneRetiredLinks() {
  gRetiredLinks.erase(std::remove_if(gRetiredLinks.begin(), gRetiredLinks.end(),
                                     [](const std::unique_ptr<BleLink> &l) { return l->finished(); }),
                      gRetiredLinks.end());
}

XPLMDataRef gPsi, gThe, gPhi, gViewType, gHeadX, gHeadY, gHeadZ;
XPLMCommandRef gCmdRecenter, gCmdToggle, gCmdSettings, gCmdCalibrate;

bool gEnabled = true;

Angles gSmoothed;  // smoothed view offsets (deg)
Angles gSmoothedHead;  // smoothed real head angles (deg), for the neck and body models
Angles gApplied;   // offsets currently added to X-Plane's head angles
Vec3 gAppliedPos;  // neck/body offsets currently added to the head position (m)
double gTorso = 0; // smoothed torso yaw (deg)

const int kView3dCockpit = 1026;
const double kStaleSeconds = 0.5;

// Packet rate, for the settings window.
uint32_t gRateCount = 0;
double gRateTime = 0, gRateHz = 0;

std::string pluginDir() {
  char path[1024] = {0};
  XPLMGetPluginInfo(XPLMGetMyID(), nullptr, path, nullptr, nullptr);
  std::string p(path);  // .../plugins/HeadTrackerBLE/lin_x64/HeadTrackerBLE.xpl
  for (int i = 0; i < 2; ++i) {
    auto slash = p.find_last_of('/');
    if (slash == std::string::npos) return ".";
    p = p.substr(0, slash);
  }
  return p;
}

std::string iniPath() { return pluginDir() + "/head_tracker.ini"; }

// Numbers in the ini always use a decimal point, whatever the system locale
// says (std::stod and printf would follow a German "1,5" locale, for example).
std::string fmt(double v) {
  std::ostringstream o;
  o.imbue(std::locale::classic());
  o << v;
  return o.str();
}

double toNumber(const std::string &v) {
  std::istringstream in(v);
  in.imbue(std::locale::classic());
  double d;
  if (!(in >> d)) throw std::invalid_argument(v);
  return d;
}

std::string trim(const std::string &s) {
  size_t a = s.find_first_not_of(" \t\r"), b = s.find_last_not_of(" \t\r");
  return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

bool parseBool(const std::string &v) { return v == "1" || v == "true" || v == "yes" || v == "on"; }

// Keep hand-edited ini values in a range that can't break the math
// (a negative smoothing time, for example, would make the view blow up).
void sanitizeConfig() {
  Config &c = gConfig;
  const Config d;
  auto fix = [](double &v, double lo, double hi, double def) {
    if (!std::isfinite(v)) v = def;
    v = std::fmax(lo, std::fmin(hi, v));
  };
  fix(c.gainYaw, 0, 10, d.gainYaw);
  fix(c.gainPitch, 0, 10, d.gainPitch);
  fix(c.gainRoll, 0, 10, d.gainRoll);
  fix(c.deadzoneDeg, 0, 30, d.deadzoneDeg);
  fix(c.smoothingMs, 0, 2000, d.smoothingMs);
  fix(c.maxYaw, 0, 180, d.maxYaw);
  fix(c.maxPitch, 0, 90, d.maxPitch);
  fix(c.maxRoll, 0, 90, d.maxRoll);
  fix(c.neckForwardCm, -30, 30, d.neckForwardCm);
  fix(c.neckUpCm, -30, 40, d.neckUpCm);
  fix(c.bodyThresholdDeg, 0, 180, d.bodyThresholdDeg);
  fix(c.bodyShare, 0, 1, d.bodyShare);
  fix(c.bodyRadiusCm, 0, 60, d.bodyRadiusCm);
  fix(c.bodyExtraYaw, 0, 3, d.bodyExtraYaw);
  fix(c.bodySmoothingMs, 0, 5000, d.bodySmoothingMs);
}

// Reads "key = value" lines into c. A profile only holds the tuning, not the
// hardware settings (Bluetooth name, sensor mounting), so those are skipped.
bool readIni(const std::string &file, Config &c, bool profile) {
  std::ifstream in(file);
  if (!in) return false;
  std::string line;
  while (std::getline(in, line)) {
    auto hash = line.find_first_of("#;");
    if (hash != std::string::npos) line = line.substr(0, hash);
    auto eq = line.find('=');
    if (eq == std::string::npos) continue;
    std::string k = trim(line.substr(0, eq)), v = trim(line.substr(eq + 1));
    if (k == "sensor_forward" || k == "sensor_up") continue;  // old setting, the tracker calibrates itself now
    if (profile && k == "device_name") continue;
    try {
      if (k == "device_name") c.deviceName = v;
      else if (k == "gain_yaw") c.gainYaw = toNumber(v);
      else if (k == "gain_pitch") c.gainPitch = toNumber(v);
      else if (k == "gain_roll") c.gainRoll = toNumber(v);
      else if (k == "invert_yaw") c.invertYaw = parseBool(v);
      else if (k == "invert_pitch") c.invertPitch = parseBool(v);
      else if (k == "invert_roll") c.invertRoll = parseBool(v);
      else if (k == "deadzone_deg") c.deadzoneDeg = toNumber(v);
      else if (k == "smoothing_ms") c.smoothingMs = toNumber(v);
      else if (k == "neck_model") c.neckModel = parseBool(v);
      else if (k == "neck_forward_cm") c.neckForwardCm = toNumber(v);
      else if (k == "neck_up_cm") c.neckUpCm = toNumber(v);
      else if (k == "body_turn") c.bodyTurn = parseBool(v);
      else if (k == "body_threshold_deg") c.bodyThresholdDeg = toNumber(v);
      else if (k == "body_share") c.bodyShare = toNumber(v);
      else if (k == "body_radius_cm") c.bodyRadiusCm = toNumber(v);
      else if (k == "body_extra_yaw") c.bodyExtraYaw = toNumber(v);
      else if (k == "body_smoothing_ms") c.bodySmoothingMs = toNumber(v);
      else if (k == "max_yaw") c.maxYaw = toNumber(v);
      else if (k == "max_pitch") c.maxPitch = toNumber(v);
      else if (k == "max_roll") c.maxRoll = toNumber(v);
      else debug("unknown setting '" + k + "' in " + file);
    } catch (...) {
      debug("bad value for '" + k + "' in " + file);
    }
  }
  return true;
}

void loadConfig() {
  gConfig = Config();  // anything the file doesn't set goes back to its default
  std::string file = iniPath();
  if (!readIni(file, gConfig, false)) debug("no " + file + ", using defaults");
  sanitizeConfig();
}

// The tuning part of the settings, shared by head_tracker.ini and profiles.
void writeTuning(std::ostream &out, const Config &c) {
  auto b = [](bool v) { return std::string(v ? "true" : "false"); };
  out << "# View degrees per degree of head movement. 1.0 = 1:1, the view follows\n"
         "# your real head exactly.\n"
      << "gain_yaw = " << fmt(c.gainYaw) << "\n"
      << "gain_pitch = " << fmt(c.gainPitch) << "\n"
      << "gain_roll = " << fmt(c.gainRoll) << "\n\n"
      << "invert_yaw = " << b(c.invertYaw) << "\n"
      << "invert_pitch = " << b(c.invertPitch) << "\n"
      << "invert_roll = " << b(c.invertRoll) << "\n\n"
      << "# Degrees ignored around center (0 = off).\n"
      << "deadzone_deg = " << fmt(c.deadzoneDeg) << "\n"
      << "# Smoothing against jitter; adds a little lag. 0 = off.\n"
      << "smoothing_ms = " << fmt(c.smoothingMs) << "\n\n"
      << "# Neck model: turning your head also moves your eyes a little, like in\n"
         "# real life (the eyes are in front of and above the neck pivot).\n"
      << "neck_model = " << b(c.neckModel) << "\n"
      << "neck_forward_cm = " << fmt(c.neckForwardCm) << "\n"
      << "neck_up_cm = " << fmt(c.neckUpCm) << "\n\n"
      << "# Body turn: past body_threshold_deg of head yaw, the torso takes\n"
         "# body_share of the extra turn, shifting your eyes like leaning to look\n"
         "# over your shoulder. body_radius_cm sets how far they shift.\n"
         "# body_extra_yaw > 0 also turns the view further past the threshold\n"
         "# (0 keeps the view 1:1 with your head).\n"
      << "body_turn = " << b(c.bodyTurn) << "\n"
      << "body_threshold_deg = " << fmt(c.bodyThresholdDeg) << "\n"
      << "body_share = " << fmt(c.bodyShare) << "\n"
      << "body_radius_cm = " << fmt(c.bodyRadiusCm) << "\n"
      << "body_extra_yaw = " << fmt(c.bodyExtraYaw) << "\n"
      << "body_smoothing_ms = " << fmt(c.bodySmoothingMs) << "\n\n"
      << "max_yaw = " << fmt(c.maxYaw) << "\n"
      << "max_pitch = " << fmt(c.maxPitch) << "\n"
      << "max_roll = " << fmt(c.maxRoll) << "\n";
}

// ---- Profiles: profiles/<name>.ini next to head_tracker.ini ----

std::string gActiveProfile;    // last profile loaded or saved ("" = none)
bool gAircraftLoaded = false;  // current settings came from an aircraft link

std::string profilesDir() { return pluginDir() + "/profiles"; }
std::string profilePath(const std::string &name) { return profilesDir() + "/" + name + ".ini"; }
std::string aircraftMapPath() { return profilesDir() + "/aircraft.txt"; }

// aircraft.txt: one "<aircraft file> = <profile>" per line.
std::map<std::string, std::string> readAircraftMap() {
  std::map<std::string, std::string> m;
  std::ifstream in(aircraftMapPath());
  std::string line;
  while (std::getline(in, line)) {
    if (trim(line).empty() || trim(line)[0] == '#') continue;
    auto eq = line.rfind('=');  // profile names never contain '='
    if (eq == std::string::npos) continue;
    std::string acf = trim(line.substr(0, eq)), prof = trim(line.substr(eq + 1));
    if (!acf.empty() && !prof.empty()) m[acf] = prof;
  }
  return m;
}

bool writeAircraftMap(const std::map<std::string, std::string> &m) {
  std::error_code ec;
  std::filesystem::create_directories(profilesDir(), ec);
  std::ofstream out(aircraftMapPath(), std::ios::trunc);
  if (!out) return false;
  out << "# Profiles loaded automatically with an aircraft (set in the settings window).\n"
         "# <aircraft .acf path, relative to the X-Plane folder> = <profile name>\n";
  for (auto &kv : m) out << kv.first << " = " << kv.second << "\n";
  return bool(out);
}

// Load the profile linked to the user's aircraft. An aircraft without one
// goes back to head_tracker.ini, if the last aircraft had changed things.
void applyAircraftProfile() {
  std::string prof = aircraftProfile();
  if (!prof.empty() && loadProfile(prof)) {
    gAircraftLoaded = true;
    debug("loaded profile '" + prof + "' for " + currentAircraft());
  } else {
    if (!prof.empty()) debug("profile '" + prof + "' for " + currentAircraft() + " is missing");
    if (gAircraftLoaded) {
      loadConfig();
      gActiveProfile.clear();
      debug("aircraft has no profile, back to head_tracker.ini");
    }
    gAircraftLoaded = false;
  }
  uiReloadValues();
}

double shape(double deg, double gain, bool invert, double limit) {
  double mag = std::fabs(deg) - gConfig.deadzoneDeg;
  if (mag <= 0) return 0;
  double out = std::copysign(mag, deg) * gain * (invert ? -1 : 1);
  return std::fmax(-limit, std::fmin(limit, out));
}

// Add our offset on top of whatever X-Plane (mouse, hat switch) already has.
void applyOffsets(const Angles &target, const Vec3 &pos) {
  float psi = XPLMGetDataf(gPsi), the = XPLMGetDataf(gThe), phi = XPLMGetDataf(gPhi);
  XPLMSetDataf(gPsi, float(psi - gApplied.yaw + target.yaw));
  XPLMSetDataf(gThe, float(the - gApplied.pitch + target.pitch));
  XPLMSetDataf(gPhi, float(phi - gApplied.roll + target.roll));
  gApplied = target;

  float x = XPLMGetDataf(gHeadX), y = XPLMGetDataf(gHeadY), z = XPLMGetDataf(gHeadZ);
  XPLMSetDataf(gHeadX, float(x - gAppliedPos.x + pos.x));
  XPLMSetDataf(gHeadY, float(y - gAppliedPos.y + pos.y));
  XPLMSetDataf(gHeadZ, float(z - gAppliedPos.z + pos.z));
  gAppliedPos = pos;
}

Vec3 eyeOffset(const Angles &view) {
  Vec3 pos;  // zero = no offset
  if (gConfig.neckModel)
    pos = neckOffset(view, gConfig.neckForwardCm / 100.0, gConfig.neckUpCm / 100.0);
  if (gConfig.bodyTurn) {
    Vec3 b = torsoOffset(gTorso, gConfig.bodyRadiusCm / 100.0);
    pos = {pos.x + b.x, pos.y + b.y, pos.z + b.z};
  }
  return pos;
}

// Take all of our offsets (angles and eye position) back out of X-Plane's head.
// Writes the head datarefs only if something is applied: writing them while
// an outside view is active sends X-Plane back to the 3D cockpit.
void removeOffsets() {
  bool applied = gApplied.yaw != 0 || gApplied.pitch != 0 || gApplied.roll != 0 ||
                 gAppliedPos.x != 0 || gAppliedPos.y != 0 || gAppliedPos.z != 0;
  if (applied) applyOffsets(Angles(), Vec3());
  gSmoothed = gSmoothedHead = Angles();
  gTorso = 0;
}

// X-Plane's view-change commands. Before one runs we take our offsets out,
// while still in the cockpit, so nothing is left to write once the view has
// changed (that write would pull X-Plane back into the cockpit).
const char *const kViewCommands[] = {
    "sim/view/free_camera", "sim/view/default_view", "sim/view/forward_with_2d_panel",
    "sim/view/forward_with_hud", "sim/view/forward_with_nothing", "sim/view/linear_spot",
    "sim/view/still_spot", "sim/view/runway", "sim/view/circle", "sim/view/tower",
    "sim/view/ridealong", "sim/view/track_weapon", "sim/view/chase", "sim/view/wingman",
    "sim/view/3d_cockpit_toggle", "sim/view/ai_controls_views",
};
std::vector<XPLMCommandRef> gViewCmds;

int beforeViewCommand(XPLMCommandRef, XPLMCommandPhase phase, void *) {
  if (phase == xplm_CommandBegin) removeOffsets();
  return 1;  // let X-Plane carry on with the command
}

double wrap180(double a) {
  a = std::fmod(a + 180.0, 360.0);
  if (a < 0) a += 360.0;
  return a - 180.0;
}

// ---- Mount calibration: the tracker measures it, the plugin walks you through it ----
//   1. look straight ahead (a few seconds to get settled) -> tracker captures it
//   2. nod down, chin toward your chest, and hold        -> tracker captures it,
//      works out how it's mounted, stores it and recenters
// X-Plane says each step out loud, since you can't read the screen while nodding.

enum class CalPhase { Idle, Ahead, AheadReply, Nod, NodReply };
struct Calibration {
  CalPhase phase = CalPhase::Idle;
  double until = 0;     // when the current step ends
  std::string prompt;   // what to do now
  std::string result;   // how the last calibration ended
  double doneAt = -100;
};
Calibration gCal;
const double kCalAheadSeconds = 4, kCalNodSeconds = 4, kCalReplySeconds = 2;

void finishCalibration(const std::string &msg, bool ok) {
  gCal.phase = CalPhase::Idle;
  gCal.result = msg;
  gCal.doneAt = XPLMGetElapsedTime();
  XPLMSpeakString(ok ? "Calibration done." : "Calibration failed.");
  debug("calibration: " + msg);
}

std::string calError(uint8_t result) {
  switch (result) {
    case 1: return "Calibration failed: no data from the tracker.";
    case 2: return "Calibration failed: nod further down and hold it.";
    case 3: return "Calibration failed: nod straight down, without turning your head.";
    case 4: return "Calibration failed: the tracker restarted in between. Try again.";
    case 5: return "Calibrated, but the tracker couldn't store it. It's lost on power off.";
  }
  return "Calibration failed (error " + std::to_string(result) + ").";
}

void updateCalibration(double now) {
  uint8_t cmd, result;
  bool gotReply = false;
  uint8_t replyCmd = 0, replyResult = 0;
  while (gLink && gLink->popReply(cmd, result)) {
    if (cmd == BleLink::kCalAhead || cmd == BleLink::kCalNod) {
      gotReply = true;
      replyCmd = cmd;
      replyResult = result;
    }
  }
  if (gCal.phase == CalPhase::Idle) return;
  if (!gLink || !gLink->connected()) {
    finishCalibration("Calibration stopped: tracker disconnected.", false);
    return;
  }
  char b[96];
  switch (gCal.phase) {
    case CalPhase::Ahead:
      std::snprintf(b, sizeof(b), "Calibrating: look straight ahead and hold still... %.0f",
                    std::ceil(gCal.until - now));
      gCal.prompt = b;
      if (now >= gCal.until) {
        gLink->send(BleLink::kCalAhead);
        gCal.phase = CalPhase::AheadReply;
        gCal.until = now + kCalReplySeconds;
      }
      break;
    case CalPhase::AheadReply:
      if (gotReply && replyCmd == BleLink::kCalAhead) {
        if (replyResult != 0) return finishCalibration(calError(replyResult), false);
        gCal.phase = CalPhase::Nod;
        gCal.until = now + kCalNodSeconds;
        XPLMSpeakString("Now nod down, chin toward your chest, and hold.");
      } else if (now >= gCal.until) {
        finishCalibration("Calibration failed: the tracker didn't answer.", false);
      }
      break;
    case CalPhase::Nod:
      std::snprintf(b, sizeof(b), "Calibrating: nod down, chin toward your chest, and hold... %.0f",
                    std::ceil(gCal.until - now));
      gCal.prompt = b;
      if (now >= gCal.until) {
        gLink->send(BleLink::kCalNod);
        gCal.phase = CalPhase::NodReply;
        gCal.until = now + kCalReplySeconds;
      }
      break;
    case CalPhase::NodReply:
      if (gotReply && replyCmd == BleLink::kCalNod) {
        if (replyResult == 0) finishCalibration("Calibrated and stored on the tracker. View recentered.", true);
        else finishCalibration(calError(replyResult), replyResult == 5);
      } else if (now >= gCal.until) {
        finishCalibration("Calibration failed: the tracker didn't answer.", false);
      }
      break;
    case CalPhase::Idle:
      break;
  }
}

bool gWasInCockpit = false;
bool gReleased = false;  // true while we leave the head datarefs alone

// X-Plane's G-loaded camera pushes the head around every frame, and our
// read-modify-write of the head datarefs bakes that push in, so the view
// creeps (down, under normal G). While we write the head, hold X-Plane's
// "suspend the G-loaded camera" command; let go once we stop. That leaves the
// user's G-load camera setting itself alone.
XPLMCommandRef gCmdGloadSuspend = nullptr;
bool gGloadHeld = false;
void holdGloadSuspend(bool hold) {
  if (!gCmdGloadSuspend || hold == gGloadHeld) return;
  if (hold) XPLMCommandBegin(gCmdGloadSuspend);
  else XPLMCommandEnd(gCmdGloadSuspend);
  gGloadHeld = hold;
}

float flightLoop(float elapsedSinceLastCall, float, int, void *) {
  std::string msg;
  while (gLink && gLink->popLog(msg)) debug(msg);
  if (!gLink) return -1;

  Sample s = gLink->latest();
  bool connected = gLink->connected();
  double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - s.time).count();
  bool fresh = connected && s.count > 0 && age < kStaleSeconds;

  double now = XPLMGetElapsedTime();
  if (now - gRateTime >= 1.0) {
    gRateHz = (s.count - gRateCount) / (now - gRateTime);
    gRateCount = s.count;
    gRateTime = now;
  }
  if (!connected) gRateHz = 0;
  uiUpdate();

  updateCalibration(now);

  // The tracker sends the head's angles already recentered and corrected for
  // how it's mounted. Older firmware can't do that, so it's not used.
  Angles target, head;  // zero = no offset
  bool inCockpit = XPLMGetDatai(gViewType) == kView3dCockpit;
  bool tracking = gEnabled && fresh && gLink->canCommand() && inCockpit;
  if (tracking) {
    Angles a = s.head;
    head = a;
    target.yaw = shape(a.yaw, gConfig.gainYaw, gConfig.invertYaw, gConfig.maxYaw);
    target.pitch = shape(a.pitch, gConfig.gainPitch, gConfig.invertPitch, gConfig.maxPitch);
    target.roll = shape(a.roll, gConfig.gainRoll, gConfig.invertRoll, gConfig.maxRoll);
    if (gConfig.bodyTurn && gConfig.bodyExtraYaw > 0) {
      // Optional: turn the view further once the body joins in.
      double extra = torsoYaw(head.yaw, gConfig.bodyThresholdDeg, gConfig.bodyExtraYaw);
      target.yaw = std::fmax(-gConfig.maxYaw, std::fmin(gConfig.maxYaw, target.yaw + extra));
    }
  }

  // Exponential smoothing that behaves the same at any frame rate.
  double alpha = 1.0;
  if (gConfig.smoothingMs > 0)
    alpha = 1.0 - std::exp(-double(elapsedSinceLastCall) * 1000.0 / gConfig.smoothingMs);
  // With the full +-180 deg yaw range, turning past straight back flips the
  // sign of the angle; take the short way round instead of spinning the view.
  double dYaw = target.yaw - gSmoothed.yaw;
  if (gConfig.maxYaw >= 180) dYaw = wrap180(dYaw);
  gSmoothed.yaw += dYaw * alpha;
  // Only fold it back once it's really past +-180, so a view held at exactly
  // +180 (the max_yaw limit) doesn't flip to -180 and swing the body the
  // wrong way.
  if (gSmoothed.yaw > 180) gSmoothed.yaw -= 360;
  if (gSmoothed.yaw < -180) gSmoothed.yaw += 360;
  gSmoothed.pitch += (target.pitch - gSmoothed.pitch) * alpha;
  gSmoothed.roll += (target.roll - gSmoothed.roll) * alpha;
  // The real head angles (before gain, invert and limits) drive the neck and
  // body models, so the eyes move as far as your head really did.
  gSmoothedHead.yaw = wrap180(gSmoothedHead.yaw + wrap180(head.yaw - gSmoothedHead.yaw) * alpha);
  gSmoothedHead.pitch += (head.pitch - gSmoothedHead.pitch) * alpha;
  gSmoothedHead.roll += (head.roll - gSmoothedHead.roll) * alpha;

  // The torso follows the (smoothed) head yaw with its own, slower smoothing.
  double torsoTarget = gConfig.bodyTurn ? torsoYaw(gSmoothedHead.yaw, gConfig.bodyThresholdDeg, gConfig.bodyShare) : 0;
  double beta = 1.0;
  if (gConfig.bodySmoothingMs > 0)
    beta = 1.0 - std::exp(-double(elapsedSinceLastCall) * 1000.0 / gConfig.bodySmoothingMs);
  gTorso += (torsoTarget - gTorso) * beta;

  if (inCockpit) {
    Vec3 eye = eyeOffset(gSmoothedHead);
    bool settled = std::fabs(gSmoothed.yaw) < 0.01 && std::fabs(gSmoothed.pitch) < 0.01 &&
                   std::fabs(gSmoothed.roll) < 0.01 && std::fabs(eye.x) < 0.0001 &&
                   std::fabs(eye.y) < 0.0001 && std::fabs(eye.z) < 0.0001;
    if (tracking || !settled) {
      holdGloadSuspend(true);
      applyOffsets(gSmoothed, eye);
      gReleased = false;
    } else if (!gReleased) {
      // No tracker (or tracking off) and our offsets have eased back to zero:
      // take them out exactly, then stop writing the head datarefs. Writing
      // them back every frame fights X-Plane's own camera effects (the
      // G-load camera's push got baked in each frame and the view crept
      // down). We write again as soon as tracking resumes.
      removeOffsets();
      gReleased = true;
      holdGloadSuspend(false);
      debug("no tracking: view released to X-Plane");
    }
  } else if (gWasInCockpit) {
    // Leaving the 3D cockpit: take our offsets back out, so they aren't baked
    // into X-Plane's saved head position when you come back.
    removeOffsets();
    holdGloadSuspend(false);
  } else {
    gSmoothed = gSmoothedHead = Angles();
    gTorso = 0;
  }
  gWasInCockpit = inCockpit;
  return -1;  // every frame
}

int handleCommand(XPLMCommandRef cmd, XPLMCommandPhase phase, void *) {
  if (phase != xplm_CommandBegin) return 0;
  if (cmd == gCmdRecenter) {
    requestRecenter();
  } else if (cmd == gCmdCalibrate) {
    startCalibration();
  } else if (cmd == gCmdToggle) {
    gEnabled = !gEnabled;
    debug(gEnabled ? "tracking on" : "tracking off");
    uiReloadValues();
  } else if (cmd == gCmdSettings) {
    uiToggle();
  }
  return 0;
}

}  // namespace

namespace ht {

void debug(const std::string &s) {
  std::string line = "HeadTrackerBLE: " + s + "\n";
  XPLMDebugString(line.c_str());
}

Config &config() { return gConfig; }
bool &trackingEnabled() { return gEnabled; }

bool requestRecenter() {
  bool ok = gLink && gLink->send(BleLink::kRecenter);
  debug(ok ? "recenter sent to the tracker" : "can't recenter: tracker not connected (or firmware too old)");
  return ok;
}

bool mountCalibrated() { return gLink && gLink->connected() && (gLink->latest().flags & 0x02); }

bool resetMount() {
  if (gCal.phase != CalPhase::Idle) return false;
  bool ok = gLink && gLink->send(BleLink::kCalReset);
  if (ok) debug("mount calibration reset to default (flat, +Y to the nose)");
  return ok;
}

bool startCalibration() {
  if (!gLink || !gLink->canCommand()) {
    finishCalibration(gLink && gLink->connected() ? "Update the tracker's firmware first (needs 1.2.0)."
                                                 : "Tracker not connected.", false);
    return false;
  }
  gCal = Calibration();
  gCal.phase = CalPhase::Ahead;
  gCal.until = XPLMGetElapsedTime() + kCalAheadSeconds;
  XPLMSpeakString("Head tracker calibration. Look straight ahead and hold still.");
  debug("calibration started");
  return true;
}

std::string calibrationText() {
  if (gCal.phase == CalPhase::Idle)
    return XPLMGetElapsedTime() - gCal.doneAt < 10 ? gCal.result : "";
  return gCal.prompt;
}
Angles currentView() { return gSmoothed; }
Vec3 currentEyeOffset() { return gAppliedPos; }

void resetConfig() {
  std::string name = gConfig.deviceName;
  gConfig = Config();
  sanitizeConfig();
  gConfig.deviceName = name;
}

std::string statusText() {
  if (!gLink) return "Off";
  if (!gLink->connected()) return "Searching for " + gConfig.deviceName + "...";
  if (!gLink->canCommand()) return "Connected, but its firmware is too old (needs 1.2.0)";
  char b[96];
  std::snprintf(b, sizeof(b), "Connected, %.0f updates/s, mount %s", gRateHz,
                mountCalibrated() ? "calibrated" : "not calibrated (flat, +Y forward)");
  return b;
}

bool saveConfig() {
  const Config &c = gConfig;
  std::ofstream out(iniPath(), std::ios::trunc);
  if (!out) return false;
  out << "# HeadTrackerBLE settings. Lives next to the lin_x64 folder:\n"
         "#   X-Plane 12/Resources/plugins/HeadTrackerBLE/head_tracker.ini\n"
         "# Easiest to change in the sim: Plugins > Head Tracker > Settings.\n"
         "# The Save default button there rewrites this file. Named profiles live\n"
         "# in the profiles/ folder next to it.\n\n"
         "# Changing the name needs a plugin reload (Plugin Admin) or restart.\n"
      << "device_name = " << c.deviceName << "\n\n";
  writeTuning(out, c);
  out << "\n# How the tracker sits on your head is measured by the tracker itself\n"
         "# (Calibrate in the settings window) and stored on it.\n";
  return bool(out);
}

std::string profileName(const std::string &raw) {
  std::string n = trim(raw);
  if (n.empty() || n.size() > 31 || n[0] == '.') return "";
  for (char ch : n)
    if (!std::isalnum((unsigned char)ch) && ch != ' ' && ch != '-' && ch != '_' && ch != '.') return "";
  return n;
}

std::vector<std::string> listProfiles() {
  std::vector<std::string> names;
  std::error_code ec;
  for (auto it = std::filesystem::directory_iterator(profilesDir(), ec);
       !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
    auto p = it->path();
    if (p.extension() != ".ini") continue;
    std::string n = p.stem().string();
    if (profileName(n) == n) names.push_back(n);
  }
  std::sort(names.begin(), names.end());
  return names;
}

bool loadProfile(const std::string &name) {
  if (profileName(name) != name) return false;
  // Start from the defaults so an older profile file without a newer setting
  // gets that setting's default; keep the hardware settings as they are.
  Config c;
  c.deviceName = gConfig.deviceName;
  if (!readIni(profilePath(name), c, true)) return false;
  gConfig = c;
  sanitizeConfig();
  gActiveProfile = name;
  return true;
}

bool saveProfile(const std::string &name) {
  if (profileName(name) != name) return false;
  std::error_code ec;
  std::filesystem::create_directories(profilesDir(), ec);
  std::ofstream out(profilePath(name), std::ios::trunc);
  if (!out) return false;
  out << "# HeadTrackerBLE profile \"" << name << "\" (Plugins > Head Tracker > Settings).\n"
         "# Same keys as head_tracker.ini, without the Bluetooth name.\n\n";
  writeTuning(out, gConfig);
  if (!out) return false;
  gActiveProfile = name;
  return true;
}

bool deleteProfile(const std::string &name) {
  if (profileName(name) != name) return false;
  std::error_code ec;
  if (!std::filesystem::remove(profilePath(name), ec)) return false;
  auto m = readAircraftMap();
  bool changed = false;
  for (auto it = m.begin(); it != m.end();) {
    if (it->second == name) {
      it = m.erase(it);
      changed = true;
    } else {
      ++it;
    }
  }
  if (changed) writeAircraftMap(m);
  if (gActiveProfile == name) gActiveProfile.clear();
  return true;
}

std::string activeProfile() { return gActiveProfile; }

std::string currentAircraft() {
  char file[512] = {0}, path[1024] = {0};
  XPLMGetNthAircraftModel(0, file, path);
  return file;
}

// Aircraft links are keyed by the .acf path relative to the X-Plane folder,
// so two aircraft whose .acf files share a name don't share a profile.
std::string aircraftKey() {
  char file[512] = {0}, path[1024] = {0}, sys[1024] = {0};
  XPLMGetNthAircraftModel(0, file, path);
  XPLMGetSystemPath(sys);
  std::string p(path), root(sys);
  if (p.empty()) return file;
  if (!root.empty() && p.compare(0, root.size(), root) == 0) p = p.substr(root.size());
  return p;
}

std::string aircraftProfile() {
  std::string acf = aircraftKey();
  if (acf.empty()) return "";
  auto m = readAircraftMap();
  auto it = m.find(acf);
  return it == m.end() ? "" : it->second;
}

bool linkAircraft(const std::string &name) {
  std::string acf = aircraftKey();
  if (acf.empty()) return false;
  auto m = readAircraftMap();
  if (name.empty()) m.erase(acf);
  else m[acf] = name;
  return writeAircraftMap(m);
}

}  // namespace ht

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc) {
  std::strcpy(outName, "HeadTrackerBLE");
  std::strcpy(outSig, "diy.headtracker.ble");
  std::strcpy(outDesc, "ESP32-C3 + BNO085 Bluetooth head tracker");
  XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

  gPsi = XPLMFindDataRef("sim/graphics/view/pilots_head_psi");
  gThe = XPLMFindDataRef("sim/graphics/view/pilots_head_the");
  gPhi = XPLMFindDataRef("sim/graphics/view/pilots_head_phi");
  gViewType = XPLMFindDataRef("sim/graphics/view/view_type");
  gHeadX = XPLMFindDataRef("sim/graphics/view/pilots_head_x");
  gHeadY = XPLMFindDataRef("sim/graphics/view/pilots_head_y");
  gHeadZ = XPLMFindDataRef("sim/graphics/view/pilots_head_z");
  if (!gPsi || !gThe || !gPhi || !gViewType || !gHeadX || !gHeadY || !gHeadZ) {
    debug("could not find head datarefs");
    return 0;
  }

  gCmdRecenter = XPLMCreateCommand("headtracker/recenter", "Head tracker: recenter");
  gCmdToggle = XPLMCreateCommand("headtracker/toggle", "Head tracker: on/off");
  XPLMRegisterCommandHandler(gCmdRecenter, handleCommand, 1, nullptr);
  gCmdSettings = XPLMCreateCommand("headtracker/settings", "Head tracker: settings window");
  gCmdCalibrate = XPLMCreateCommand("headtracker/calibrate", "Head tracker: calibrate how it's mounted");
  XPLMRegisterCommandHandler(gCmdCalibrate, handleCommand, 1, nullptr);
  XPLMRegisterCommandHandler(gCmdToggle, handleCommand, 1, nullptr);
  XPLMRegisterCommandHandler(gCmdSettings, handleCommand, 1, nullptr);
  gCmdGloadSuspend = XPLMFindCommand("sim/view/g_loaded_camera_suspend");
  if (!gCmdGloadSuspend) debug("no G-loaded camera suspend command; the view may creep with the G-load camera on");
  for (const char *name : kViewCommands)
    if (XPLMCommandRef c = XPLMFindCommand(name)) {
      XPLMRegisterCommandHandler(c, beforeViewCommand, 1, nullptr);
      gViewCmds.push_back(c);
    }

  uiStart();
  return 1;
}

PLUGIN_API void XPluginStop() {
  XPLMUnregisterCommandHandler(gCmdRecenter, handleCommand, 1, nullptr);
  XPLMUnregisterCommandHandler(gCmdToggle, handleCommand, 1, nullptr);
  XPLMUnregisterCommandHandler(gCmdSettings, handleCommand, 1, nullptr);
  XPLMUnregisterCommandHandler(gCmdCalibrate, handleCommand, 1, nullptr);
  for (XPLMCommandRef c : gViewCmds) XPLMUnregisterCommandHandler(c, beforeViewCommand, 1, nullptr);
  gViewCmds.clear();
  uiStop();
  if (!gRetiredLinks.empty()) debug("waiting for the Bluetooth thread to finish connecting");
  gRetiredLinks.clear();  // joins them
}

PLUGIN_API int XPluginEnable() {
  // Read the ini here, not in XPluginStart, so disable/enable in Plugin Admin
  // picks up a changed file (the Bluetooth name, for example).
  loadConfig();
  gActiveProfile.clear();
  gAircraftLoaded = false;
  applyAircraftProfile();  // also refreshes the settings window
  // Fresh link: its packet count starts over.
  gCal = Calibration();
  gRateCount = 0;
  gRateTime = 0;
  gRateHz = 0;
  pruneRetiredLinks();
  gLink = std::make_unique<BleLink>(gConfig.deviceName);
  gLink->start();
  XPLMRegisterFlightLoopCallback(flightLoop, -1.0f, nullptr);
  debug("enabled, looking for " + gConfig.deviceName);
  return 1;
}

PLUGIN_API void XPluginDisable() {
  XPLMUnregisterFlightLoopCallback(flightLoop, nullptr);
  if (gWasInCockpit) removeOffsets();  // put the head back where it was
  holdGloadSuspend(false);
  gApplied = gSmoothed = gSmoothedHead = Angles();
  gAppliedPos = Vec3();
  gTorso = 0;
  gWasInCockpit = false;
  // Stop the Bluetooth thread. Don't freeze the sim if it's stuck in a
  // connect: let it finish in the background instead.
  if (gLink && !gLink->stopFor(std::chrono::milliseconds(1500))) {
    debug("Bluetooth is still busy connecting; it stops in the background");
    gRetiredLinks.push_back(std::move(gLink));
  }
  gLink.reset();
  pruneRetiredLinks();
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int msg, void *param) {
  // A new user aircraft (index 0): load its profile, if it has one.
  if (msg == XPLM_MSG_PLANE_LOADED && param == nullptr) applyAircraftProfile();
}
