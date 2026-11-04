// Shared state between the plugin core (plugin.cpp) and the settings window.
#pragma once
#include <string>
#include <vector>

#include "head_math.h"

namespace ht {

struct Config {
  std::string deviceName = "HeadTrackerC3";
  double gainYaw = 1.0, gainPitch = 1.0, gainRoll = 1.0;  // 1:1 head-to-view by default
  bool invertYaw = false, invertPitch = false, invertRoll = false;
  double deadzoneDeg = 0.0;      // optional: ignore tiny movements around center
  double smoothingMs = 15;       // 0 = off; higher is smoother but laggier
  double maxYaw = 180, maxPitch = 90, maxRoll = 90;
  // Neck model: eyes move as the head rotates around the neck (x/y/z offsets).
  bool neckModel = true;
  double neckForwardCm = 9;   // eyes in front of the neck pivot
  double neckUpCm = 12;       // eyes above the neck pivot
  // Body turn: past a yaw threshold the torso turns too, shifting the eyes
  // sideways like when you look over your shoulder.
  bool bodyTurn = true;
  double bodyThresholdDeg = 60;  // neck-only yaw before the body joins in
  double bodyShare = 0.5;        // share of the yaw past the threshold the body takes
  double bodyRadiusCm = 15;      // neck distance in front of the torso's turning axis
  double bodyExtraYaw = 0.0;     // optional: extra view yaw per degree past the threshold
  double bodySmoothingMs = 150;  // the body follows a bit slower than the head
};

// Implemented in plugin.cpp
Config &config();
bool saveConfig();          // write head_tracker.ini
void resetConfig();         // back to defaults (1:1), keeps device name

// Named profiles: profiles/<name>.ini next to head_tracker.ini. A profile
// holds the tuning only; the Bluetooth name stays as it is.
std::string profileName(const std::string &raw);  // cleaned name, "" if not allowed
std::vector<std::string> listProfiles();           // sorted
bool loadProfile(const std::string &name);         // applies it live
bool saveProfile(const std::string &name);         // current settings, overwrites
bool deleteProfile(const std::string &name);       // also drops aircraft links to it
std::string activeProfile();                       // last loaded/saved, "" = none
std::string currentAircraft();                     // user aircraft's .acf file name
std::string aircraftProfile();                     // profile linked to it, "" = none
bool linkAircraft(const std::string &name);        // "" unlinks
// The tracker recenters and calibrates itself; these send it the command.
// They return false if it isn't connected (or its firmware is too old).
bool requestRecenter();
bool startCalibration();    // "look ahead, then nod down", guided by voice and calibrationText()
bool resetMount();          // forget the calibration: board flat, +Y to the nose
bool mountCalibrated();     // the connected tracker has a stored calibration
std::string calibrationText();  // current calibration step or last result, "" if none
bool &trackingEnabled();
std::string statusText();   // "Connected, 100 Hz" etc.
Angles currentView();       // offsets being applied right now (deg)
Vec3 currentEyeOffset();    // neck + body eye shift being applied right now (m)
void debug(const std::string &s);

// Implemented in settings_ui.cpp
void uiStart();             // adds the Plugins > Head Tracker menu
void uiStop();
void uiToggle();
void uiUpdate();            // call every frame; refreshes the live readout
void uiReloadValues();      // after config changed outside the window

}  // namespace ht
