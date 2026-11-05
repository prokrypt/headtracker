// In-sim settings window: Plugins > Head Tracker > Settings (or the
// headtracker/settings command). Changes apply live; Save default writes the
// ini, and the profile row saves/loads named profiles.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "XPLMDisplay.h"
#include "XPLMMenus.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include "XPStandardWidgets.h"
#include "XPWidgets.h"

#include "app.h"

namespace ht {
namespace {

struct Slider {
  const char *label;
  double Config::*field;
  double min, max, step;
  const char *format;
  XPWidgetID bar = nullptr, value = nullptr;
};

struct Check {
  const char *label;
  bool Config::*field;
  XPWidgetID box = nullptr;
};

std::vector<Slider> gSliders = {
    {"Yaw gain", &Config::gainYaw, 0.0, 3.0, 0.05, "%.2fx"},
    {"Pitch gain", &Config::gainPitch, 0.0, 3.0, 0.05, "%.2fx"},
    {"Roll gain", &Config::gainRoll, 0.0, 3.0, 0.05, "%.2fx"},
    {"Deadzone", &Config::deadzoneDeg, 0.0, 5.0, 0.1, "%.1f deg"},
    {"Smoothing", &Config::smoothingMs, 0.0, 100.0, 1.0, "%.0f ms"},
    {"Max yaw", &Config::maxYaw, 30.0, 180.0, 5.0, "%.0f deg"},
    {"Max pitch", &Config::maxPitch, 15.0, 90.0, 5.0, "%.0f deg"},
    {"Max roll", &Config::maxRoll, 0.0, 90.0, 5.0, "%.0f deg"},
    {"Neck: fwd", &Config::neckForwardCm, 0.0, 20.0, 0.5, "%.1f cm"},
    {"Neck: up", &Config::neckUpCm, 0.0, 25.0, 0.5, "%.1f cm"},
    {"Body: from", &Config::bodyThresholdDeg, 20.0, 150.0, 5.0, "%.0f deg"},
    {"Body: share", &Config::bodyShare, 0.0, 1.0, 0.05, "%.2f"},
    {"Body: shift", &Config::bodyRadiusCm, 0.0, 40.0, 1.0, "%.0f cm"},
    {"Body: +view", &Config::bodyExtraYaw, 0.0, 1.0, 0.05, "%.2fx"},
    {"Body: lag", &Config::bodySmoothingMs, 0.0, 500.0, 10.0, "%.0f ms"},
};
std::vector<Check> gChecks = {
    {"Invert yaw", &Config::invertYaw},
    {"Invert pitch", &Config::invertPitch},
    {"Invert roll", &Config::invertRoll},
    {"Neck model (eyes move with head)", &Config::neckModel},
    {"Body turn (body joins in past a yaw limit)", &Config::bodyTurn},
};

XPWidgetID gWindow = nullptr;
XPWidgetID gStatus, gView, gEye, gEnabledBox, gRecenterBtn, gCalibrateBtn, gResetMountBtn;
XPWidgetID gResetBtn, gSaveBtn, gMessage;
XPWidgetID gProfileField, gPrevBtn, gNextBtn, gLoadBtn, gSaveAsBtn, gDeleteBtn, gAircraftBox;
std::string gPendingDelete;  // Delete needs a second press to confirm
float gPendingDeleteTime = 0;
XPLMMenuID gMenu = nullptr;
float gLastRefresh = 0;

const int kWidth = 420, kRow = 24, kLabelW = 95, kValueW = 70;

void setText(XPWidgetID w, const std::string &s) { XPSetWidgetDescriptor(w, s.c_str()); }

// Text fields keep the keyboard after you type in them, which would swallow
// X-Plane's key commands. Hand the keyboard back to X-Plane.
void releaseKeyboard() {
  XPWidgetID f = XPGetWidgetWithFocus();
  if (f && f == gProfileField) XPSetKeyboardFocus(0);
}

void hideWindow() {
  releaseKeyboard();
  XPHideWidget(gWindow);
}

std::string sliderText(const Slider &s) {
  char b[32];
  std::snprintf(b, sizeof(b), s.format, config().*s.field);
  return b;
}

XPWidgetID caption(int l, int t, int r, const char *text) {
  return XPCreateWidget(l, t, r, t - kRow + 4, 1, text, 0, gWindow, xpWidgetClass_Caption);
}

XPWidgetID button(int l, int t, int r, const char *text, bool checkbox) {
  XPWidgetID w = XPCreateWidget(l, t, r, t - kRow + 4, 1, text, 0, gWindow, xpWidgetClass_Button);
  if (checkbox) {
    XPSetWidgetProperty(w, xpProperty_ButtonType, xpRadioButton);
    XPSetWidgetProperty(w, xpProperty_ButtonBehavior, xpButtonBehaviorCheckBox);
  } else {
    XPSetWidgetProperty(w, xpProperty_ButtonType, xpPushButton);
    XPSetWidgetProperty(w, xpProperty_ButtonBehavior, xpButtonBehaviorPushButton);
  }
  return w;
}

XPWidgetID textField(int l, int t, int r, int maxChars) {
  XPWidgetID w = XPCreateWidget(l, t, r, t - kRow + 4, 1, "", 0, gWindow, xpWidgetClass_TextField);
  XPSetWidgetProperty(w, xpProperty_TextFieldType, xpTextEntryField);
  XPSetWidgetProperty(w, xpProperty_MaxCharacters, maxChars);
  return w;
}

std::string widgetText(XPWidgetID w);

// The aircraft box is ticked when the named profile is the one this aircraft loads.
void refreshAircraftBox() {
  std::string name = profileName(widgetText(gProfileField));
  bool linked = !name.empty() && name == aircraftProfile();
  XPSetWidgetProperty(gAircraftBox, xpProperty_ButtonState, linked ? 1 : 0);
}

void loadValues() {
  if (!gWindow) return;
  const Config &c = config();
  for (auto &s : gSliders) {
    XPSetWidgetProperty(s.bar, xpProperty_ScrollBarSliderPosition,
                        intptr_t(std::lround(c.*s.field / s.step)));
    setText(s.value, sliderText(s));
  }
  for (auto &k : gChecks) XPSetWidgetProperty(k.box, xpProperty_ButtonState, c.*k.field ? 1 : 0);
  XPSetWidgetProperty(gEnabledBox, xpProperty_ButtonState, trackingEnabled() ? 1 : 0);
  if (!activeProfile().empty()) setText(gProfileField, activeProfile());
  refreshAircraftBox();
}

std::string widgetText(XPWidgetID w) {
  char b[64] = {0};
  XPGetWidgetDescriptor(w, b, sizeof(b) - 1);
  std::string s(b);
  while (!s.empty() && s.back() == ' ') s.pop_back();
  while (!s.empty() && s.front() == ' ') s.erase(0, 1);
  return s;
}

const char *kBadName = "Names: letters, digits, space, - _ . (up to 31).";

// Step through the saved profiles with the < and > buttons.
void cycleProfile(int dir) {
  std::vector<std::string> names = listProfiles();
  if (names.empty()) {
    setText(gMessage, "No profiles yet. Type a name and press Save as.");
    return;
  }
  std::string cur = profileName(widgetText(gProfileField));
  int n = int(names.size()), i = -1;
  for (int k = 0; k < n; ++k)
    if (names[k] == cur) i = k;
  i = i < 0 ? (dir > 0 ? 0 : n - 1) : (i + dir + n) % n;
  setText(gProfileField, names[i]);
  refreshAircraftBox();
  char b[64];
  std::snprintf(b, sizeof(b), "Profile %d of %d. Press Load to use it.", i + 1, n);
  setText(gMessage, b);
}

void onProfileButton(XPWidgetID b) {
  releaseKeyboard();
  std::string typed = widgetText(gProfileField), name = profileName(typed);
  if (b != gDeleteBtn) gPendingDelete.clear();
  if (name.empty()) {
    setText(gMessage, typed.empty() ? "Type a profile name, or pick one with < >." : kBadName);
    return;
  }
  setText(gProfileField, name);
  if (b == gLoadBtn) {
    if (loadProfile(name)) {
      loadValues();
      setText(gMessage, "Loaded profile '" + name + "'.");
      debug("loaded profile '" + name + "'");
    } else {
      setText(gMessage, "There's no profile named '" + name + "'.");
    }
  } else if (b == gSaveAsBtn) {
    bool ok = saveProfile(name);
    setText(gMessage, ok ? "Saved profile '" + name + "'." : "Couldn't write the profile file!");
    debug(ok ? "saved profile '" + name + "'" : "could not save profile '" + name + "'");
  } else if (b == gDeleteBtn) {
    float now = XPLMGetElapsedTime();
    if (gPendingDelete != name || now - gPendingDeleteTime > 5) {
      gPendingDelete = name;
      gPendingDeleteTime = now;
      setText(gMessage, "Press Delete again to delete '" + name + "'.");
      return;
    }
    gPendingDelete.clear();
    bool ok = deleteProfile(name);
    setText(gMessage, ok ? "Deleted profile '" + name + "'." : "There's no profile named '" + name + "'.");
    if (ok) debug("deleted profile '" + name + "'");
    refreshAircraftBox();
  }
}

void onAircraftBox(bool on) {
  releaseKeyboard();
  std::string name = profileName(widgetText(gProfileField)), acf = currentAircraft();
  std::vector<std::string> names = listProfiles();
  bool exists = std::find(names.begin(), names.end(), name) != names.end();
  if (acf.empty()) {
    setText(gMessage, "No aircraft loaded.");
  } else if (on && !exists) {
    setText(gMessage, "Save the profile first (Save as), then tick this.");
  } else if (on) {
    bool ok = linkAircraft(name);
    setText(gMessage, ok ? "'" + name + "' now loads with " + acf + "." : "Couldn't write profiles/aircraft.txt!");
  } else if (aircraftProfile() == name) {
    bool ok = linkAircraft("");
    setText(gMessage, ok ? acf + " no longer loads a profile." : "Couldn't write profiles/aircraft.txt!");
  }
  refreshAircraftBox();
}

int handler(XPWidgetMessage msg, XPWidgetID, intptr_t p1, intptr_t) {
  if (msg == xpMessage_CloseButtonPushed) {
    hideWindow();
    return 1;
  }
  if (msg == xpMsg_ScrollBarSliderPositionChanged) {
    for (auto &s : gSliders) {
      if (XPWidgetID(p1) != s.bar) continue;
      intptr_t pos = XPGetWidgetProperty(s.bar, xpProperty_ScrollBarSliderPosition, nullptr);
      config().*s.field = pos * s.step;
      setText(s.value, sliderText(s));
      return 1;
    }
  }
  if (msg == xpMsg_ButtonStateChanged) {
    bool on = XPGetWidgetProperty(XPWidgetID(p1), xpProperty_ButtonState, nullptr) != 0;
    if (XPWidgetID(p1) == gEnabledBox) {
      trackingEnabled() = on;
      return 1;
    }
    if (XPWidgetID(p1) == gAircraftBox) {
      onAircraftBox(on);
      return 1;
    }
    for (auto &k : gChecks) {
      if (XPWidgetID(p1) != k.box) continue;
      config().*k.field = on;
      return 1;
    }
  }
  if (msg == xpMsg_TextFieldChanged && XPWidgetID(p1) == gProfileField) {
    refreshAircraftBox();
    return 1;
  }
  if (msg == xpMsg_PushButtonPressed) {
    XPWidgetID b = XPWidgetID(p1);
    if (b == gPrevBtn || b == gNextBtn) {
      releaseKeyboard();
      gPendingDelete.clear();
      cycleProfile(b == gNextBtn ? 1 : -1);
    } else if (b == gLoadBtn || b == gSaveAsBtn || b == gDeleteBtn) {
      onProfileButton(b);
    } else if (b == gRecenterBtn) {
      if (!requestRecenter()) setText(gMessage, "Can't recenter: the tracker isn't connected.");
    } else if (b == gCalibrateBtn) {
      releaseKeyboard();
      startCalibration();  // progress shows in the message line
    } else if (b == gResetMountBtn) {
      releaseKeyboard();
      setText(gMessage, resetMount() ? "Calibration cleared: board flat, +Y to the nose."
                                     : "Can't reset now (not connected, or calibrating).");
    } else if (b == gResetBtn) {
      resetConfig();
      loadValues();
      setText(gMessage, "Back to 1:1 defaults (not saved yet).");
    } else if (b == gSaveBtn) {
      bool ok = saveConfig();
      setText(gMessage, ok ? "Saved as default (head_tracker.ini)." : "Couldn't write head_tracker.ini!");
      debug(ok ? "settings saved" : "could not save settings");
    }
    return 1;
  }
  return 0;
}

void createWindow() {
  int sl, st, sr, sb;
  XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
  int rows = 7 + int(gSliders.size()) + 6;
  int height = rows * kRow + 40;
  int left = sl + 80, top = st - 120;
  gWindow = XPCreateWidget(left, top, left + kWidth, top - height, 0, "Head Tracker", 1, nullptr,
                           xpWidgetClass_MainWindow);
  XPSetWidgetProperty(gWindow, xpProperty_MainWindowHasCloseBoxes, 1);
  XPAddWidgetCallback(gWindow, handler);

  int l = left + 12, r = left + kWidth - 12;
  int y = top - 26;
  gStatus = caption(l, y, r, "");
  y -= kRow;
  gView = caption(l, y, r, "");
  y -= kRow;
  gEye = caption(l, y, r, "");
  y -= kRow;
  gEnabledBox = button(l, y, l + 20, "", true);
  caption(l + 24, y, l + 140, "Tracking on");
  gRecenterBtn = button(r - 110, y, r, "Recenter", false);
  y -= kRow + 6;

  caption(l, y, l + 56, "Profile");
  gProfileField = textField(l + 58, y, r - 176, 31);
  gPrevBtn = button(r - 170, y, r - 142, "<", false);
  gNextBtn = button(r - 138, y, r - 110, ">", false);
  gLoadBtn = button(r - 100, y, r, "Load", false);
  y -= kRow;
  gSaveAsBtn = button(l, y, l + 90, "Save as", false);
  gDeleteBtn = button(l + 96, y, l + 186, "Delete", false);
  gAircraftBox = button(l + 200, y, l + 220, "", true);
  caption(l + 224, y, r, "Load with this aircraft");
  y -= kRow + 6;

  for (auto &s : gSliders) {
    caption(l, y, l + kLabelW, s.label);
    s.bar = XPCreateWidget(l + kLabelW, y, r - kValueW - 6, y - kRow + 6, 1, "", 0, gWindow,
                           xpWidgetClass_ScrollBar);
    XPSetWidgetProperty(s.bar, xpProperty_ScrollBarType, xpScrollBarTypeSlider);
    XPSetWidgetProperty(s.bar, xpProperty_ScrollBarMin, intptr_t(std::lround(s.min / s.step)));
    XPSetWidgetProperty(s.bar, xpProperty_ScrollBarMax, intptr_t(std::lround(s.max / s.step)));
    XPSetWidgetProperty(s.bar, xpProperty_ScrollBarPageAmount, 1);
    s.value = caption(r - kValueW, y, r, "");
    y -= kRow;
  }

  int cw = (r - l) / 3;
  for (size_t i = 0; i < gChecks.size(); ++i) {
    if (i >= 3) y -= kRow;  // neck model and body turn get their own rows
    int cl = i < 3 ? l + int(i) * cw : l;
    gChecks[i].box = button(cl, y, cl + 20, "", true);
    caption(cl + 24, y, i < 3 ? cl + cw : r, gChecks[i].label);
  }
  y -= kRow + 6;

  caption(l, y, l + 120, "Tracker mounting");
  gCalibrateBtn = button(l + 125, y, l + 255, "Calibrate...", false);
  gResetMountBtn = button(r - 110, y, r, "Reset", false);
  y -= kRow + 6;

  gResetBtn = button(l, y, l + 130, "Reset to 1:1", false);
  gSaveBtn = button(r - 110, y, r, "Save default", false);
  y -= kRow;
  gMessage = caption(l, y, r, "Changes apply right away. Save default keeps them.");

  loadValues();
}

void menuHandler(void *, void *item) {
  if (item == (void *)1) uiToggle();
  if (item == (void *)2) requestRecenter();
}

}  // namespace

void uiStart() {
  int idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "Head Tracker", nullptr, 0);
  gMenu = XPLMCreateMenu("Head Tracker", XPLMFindPluginsMenu(), idx, menuHandler, nullptr);
  XPLMAppendMenuItem(gMenu, "Settings...", (void *)1, 0);
  XPLMAppendMenuItem(gMenu, "Recenter", (void *)2, 0);
}

void uiStop() {
  if (gWindow) XPDestroyWidget(gWindow, 1);
  gWindow = nullptr;
  if (gMenu) XPLMDestroyMenu(gMenu);
  gMenu = nullptr;
}

void uiToggle() {
  if (!gWindow) createWindow();
  if (XPIsWidgetVisible(gWindow)) {
    hideWindow();
  } else {
    loadValues();
    XPShowWidget(gWindow);
    XPBringRootWidgetToFront(gWindow);
  }
}

void uiReloadValues() { loadValues(); }

void uiUpdate() {
  if (!gWindow || !XPIsWidgetVisible(gWindow)) return;
  float now = XPLMGetElapsedTime();
  if (now - gLastRefresh < 0.1f) return;
  gLastRefresh = now;
  setText(gStatus, "Tracker: " + statusText());
  // Calibration steps and results go to the message line, once per change so
  // other messages aren't overwritten all the time.
  static std::string lastCal;
  std::string cal = calibrationText();
  if (cal != lastCal && !cal.empty()) setText(gMessage, cal);
  lastCal = cal;
  Angles v = currentView();
  char b[96];
  std::snprintf(b, sizeof(b), "View offset:  yaw %+6.1f   pitch %+6.1f   roll %+6.1f", v.yaw, v.pitch,
                v.roll);
  setText(gView, b);
  if (config().neckModel || config().bodyTurn) {
    Vec3 p = currentEyeOffset();
    std::snprintf(b, sizeof(b), "Eye offset:  x %+5.1f   y %+5.1f   z %+5.1f cm", p.x * 100, p.y * 100,
                  p.z * 100);
  } else {
    std::snprintf(b, sizeof(b), "Eye offset:  off");
  }
  setText(gEye, b);
}

}  // namespace ht
