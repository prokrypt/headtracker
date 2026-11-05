# DIY Bluetooth head tracker for X-Plane 12 (Linux)

ESP32-C3 Super Mini + BNO085 → Bluetooth LE → `HeadTrackerBLE` plugin → pilot head view.
Nothing has to run outside X-Plane. OpenTrack and your old head-track plugin are not needed.

## 1. Wiring

| BNO085 board | ESP32-C3 Super Mini |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO5 |
| SCL | GPIO6 |
| RST | GPIO10 |
| PS0, PS1 | GND (I2C mode; skip if the board already pulls them low) |
| INT | GPIO4 (optional, but needed for the best battery life) |
| CS, ADO | leave unconnected |

The C3's **BOOT button (GPIO9)** works as the recenter button once it's running.
The onboard LED gives one short blink every 2 s while it waits for X-Plane, and a double blink every 2 s once connected. It's mostly off to save power.

Mounting: put the board on your headset or cap **any way round, at any angle**. Then calibrate it once (section 4): the tracker works out how it sits on your head and remembers it, even after a power off or a firmware update.
Until it's calibrated it assumes the board lies flat on top of your head with **+Y pointing at your nose**.

> **Firmware 1.2.0 and the plugin go together.** Calibration and recentering now happen on the tracker, so this plugin needs firmware 1.2.0 or newer, and older plugins can't read the new firmware. Update both. With old firmware the plugin connects but doesn't move the view, and the settings window says the firmware is too old.

## 2. Flash the firmware

```
pipx install platformio  # Ubuntu 23.04+/Debian 12 refuse a plain "pip install" (sudo apt install pipx)
sudo usermod -aG dialout $USER   # serial port access; log out and back in afterwards
cd firmware
pio run -t upload        # hold BOOT while plugging in if upload doesn't start
pio device monitor       # then press RESET: it prints "BLE advertising" and "BNO085 ready"
```

Also install PlatformIO's udev rules once (https://docs.platformio.org/en/latest/core/installation/udev-rules.html). Pressing RESET drops the C3's USB port for a moment, so you may have to start `pio device monitor` again after it.
The first build takes a few minutes, because it compiles ESP-IDF itself (needed for light sleep). Later builds are quick.
If you change `sdkconfig.defaults` (or got a new copy of it), delete `firmware/sdkconfig.esp32c3` first. PlatformIO only reads the defaults when that file doesn't exist yet.
Flash over USB once like this. After that you can update over Bluetooth (section 5).
The first USB flash has to be `pio run -t upload`, because it also writes the bootloader and the partition table. `builds/firmware.bin` on its own is only for Bluetooth updates.

### No sensor yet? Fake-sensor build

`pio run -e esp32c3-fake -t upload` builds the same firmware without the BNO085 (version `1.2.0-fake`). It sends simulated head motion at 100 Hz (yaw ±45°, pitch ±15°, roll ±8°, slow sine waves), so you can test Bluetooth, the plugin, recentering and OTA with just the bare C3. The serial log says "FAKE SENSOR build". Its mount calibration is kept under a separate flash key, so calibrating against fake motion never carries over to the real firmware. Its `.bin` for Bluetooth updates is `.pio/build/esp32c3-fake/firmware.bin`. Going from the fake build to the real one (or back) over Bluetooth works like any other update.

## 3. Build the plugin

```
sudo apt install build-essential cmake git libdbus-1-dev   # or your distro's equivalents
cd xplane-plugin
# Download the X-Plane SDK from https://developer.x-plane.com/sdk/ (Plugin SDK downloads)
# and unzip it so that xplane-plugin/SDK/CHeaders/ exists.
cmake -B build -DXPLANE_DIR="$HOME/X-Plane 12"
cmake --build build -j
cmake --install build     # copies HeadTrackerBLE/ into X-Plane 12/Resources/plugins/
```

Or skip the build and use the prebuilt `builds/HeadTrackerBLE.xpl`: copy it to `X-Plane 12/Resources/plugins/HeadTrackerBLE/lin_x64/` and, the first time only, `xplane-plugin/head_tracker.ini` to `X-Plane 12/Resources/plugins/HeadTrackerBLE/` (copying it again would overwrite your saved settings). `cmake --install` already leaves an existing ini alone. It was built on Ubuntu 24.04; if X-Plane's Log.txt says it can't load it (a `GLIBC` or `GLIBCXX` version error), build it yourself as above.

## 4. Fly

1. Make sure Bluetooth is on (`bluetoothctl show`). You don't need to pair; the plugin finds the tracker by name.
2. **Disable your old head-track plugin** in Plugin Admin, or both will fight over the view.
3. Start X-Plane and go to the 3D cockpit. The tracker recenters by itself when X-Plane connects to it.
4. **First time: calibrate the mounting.** Put the headset on, open **Plugins > Head Tracker > Settings...** and press **Calibrate...**. X-Plane talks you through it:
   * "Look straight ahead and hold still" (4 s). This pose becomes your straight ahead.
   * "Now nod down, chin toward your chest, and hold" (4 s). Nod straight down without turning; 30° or more is plenty.
   * "Calibration done." The view is recentered on your straight-ahead pose.

   It's stored on the tracker. Redo it if you move the board on the headset. **Reset** next to it goes back to "flat, +Y to the nose". You can also bind **headtracker/calibrate** to a button.
5. Bind **headtracker/recenter** (and **headtracker/toggle** if you want it) to a joystick button under Settings > Joystick.

Recentering happens on the tracker: the Recenter button, the command, and the tracker's BOOT button all tell it to take your current head pose as straight ahead. It also recenters by itself whenever the sensor restarts, which includes connecting after nothing was connected for 10 s or more. A shorter Bluetooth drop keeps your center.
Yaw is measured around the vertical (the sensor knows where down is), so recentering with your head a bit tilted doesn't skew later turns.

Status messages go to `X-Plane 12/Log.txt`, prefixed with `HeadTrackerBLE:`.

## In-sim settings

**Plugins > Head Tracker > Settings...** (or bind **headtracker/settings** to a key) opens a window with:

* live tracker status (connected, updates per second, mount calibrated or not) and the current view offset
* tracking on/off and a Recenter button
* sliders for gain, deadzone, smoothing and maximum angle per axis
* invert checkboxes
* **Calibrate...** and **Reset** for the tracker's mounting (section 4)
* the neck model and body turn, each with an on/off box and its distances (see below)
* named profiles (see below)
* **Reset to 1:1** and **Save default**

Changes apply immediately. Save default writes them to `head_tracker.ini`, so they're there next flight.

### Profiles

Keep different setups (say a heli profile with body turn and an airliner one without) and switch between them in flight:

* **Save as:** type a name in the Profile box and press Save as. It stores the current settings as that profile (saving again under the same name updates it).
* **Load:** pick a saved profile with **<** and **>** (or type its name) and press Load. It applies right away.
* **Delete:** press it twice within 5 s.
* **Load with this aircraft:** tick it and that profile loads by itself whenever you fly the current aircraft. An aircraft without a linked profile uses the default settings from `head_tracker.ini`.

Profiles are plain files in `HeadTrackerBLE/profiles/<name>.ini` (same keys as `head_tracker.ini`), and the aircraft links are in `profiles/aircraft.txt`. They hold the tuning only: the Bluetooth name belongs to your hardware, so loading a profile leaves it alone (and the mounting calibration lives on the tracker). Loading a profile doesn't change your default; press Save default if you want it to be.

The Bluetooth name is the only setting that still needs the ini file. After changing it, disable and re-enable the plugin in Plugin Admin (it re-reads the ini then) or restart X-Plane.

## 5. Update the firmware over Bluetooth

```
python3 -m venv ~/.venvs/headtracker && ~/.venvs/headtracker/bin/pip install bleak   # once; needs Python 3.10+
cd firmware && pio run              # builds .pio/build/esp32c3/firmware.bin
~/.venvs/headtracker/bin/python ../tools/ota_update.py .pio/build/esp32c3/firmware.bin
```

`builds/firmware.bin` is a prebuilt copy of the current firmware (1.2.0) if you don't want to build it.

It takes about a minute. X-Plane can stay open: tracking pauses during the update and comes back by itself.
Bump `FW_VERSION` in `platformio.ini` so you can see the new version reported at the end.

If X-Plane is running on the same PC, the updater shares the plugin's Bluetooth connection, so the plugin drops and reconnects by itself when the tracker reboots.

It's safe to interrupt. The tracker keeps two firmware slots and only switches after the whole image arrives and its SHA-256 checks out.
The new firmware then runs "on probation": if it crashes, or isn't up and healthy within 60 s, the tracker goes back to the old one on its own.
If the sensor is unplugged the firmware still counts as healthy, so you can always push a fix over Bluetooth.
While new firmware is still on probation (its first 15 s), the tracker refuses another update, because the other slot holds the old firmware it would go back to.

Anyone within Bluetooth range could send firmware too, since there's no pairing. For a head tracker that's a small risk, but keep it in mind.

## Battery life

The tracker light-sleeps whenever it can:

* **While tracking:** between sensor samples and between Bluetooth packets. It's woken by the BNO085's INT pin the moment a sample is ready, so there's no added lag. Without INT wired it still works, but it polls and saves less.
* **While nothing is connected:** after 10 s the BNO085 is put to sleep, Bluetooth advertises only every 0.5 s, and the chip sleeps almost all the time. X-Plane still finds it within a couple of seconds.
* **During a Bluetooth update:** sleep is paused so the update runs at full speed.
* **On a computer's USB port:** it stays awake, because sleep would drop the USB serial connection and make `pio run -t upload` awkward. On a power bank or battery there's no USB host, so it sleeps. It decides this by watching for USB traffic. If you plug it into a computer after it has started on battery and the serial port doesn't show up, unplug it and plug it in again (or press RESET).

The Super Mini has no 32 kHz crystal, so the main crystal stays on during sleep to keep Bluetooth timing. That costs a little, but it's what lets the connection survive sleep.

## Tuning (`head_tracker.ini`)

Everything here can also be changed in the settings window.


* By default the view follows your head 1:1 (gain 1.0, no deadzone). Optionally, `gain_yaw = 1.5` turns a 60° head turn into 90° in the sim, so you can check your six without turning your neck as far.
* `smoothing_ms` trades jitter for lag. Default 15; try up to 40 if it jitters.
* If an axis moves the wrong way, calibrate first (section 4). `invert_yaw`, `invert_pitch` and `invert_roll` are there if you want it reversed on purpose.
* Old `sensor_forward`/`sensor_up` lines in the ini are ignored now; calibration replaces them.

## Notes

* Rotation is measured directly. Position (x/y/z) comes from a **neck model**: your eyes sit about 9 cm in front of and 12 cm above the pivot in your neck, so turning or tilting your head moves them, and the plugin moves the sim's eye point to match. Looking down moves your eyes forward and down; tilting your head to the side moves them sideways. It isn't real lean tracking, because an IMU can't measure position reliably, but it makes looking around feel natural. Turn it off or adjust the distances in the settings window.
* **Body turn:** once you turn your head past 60°, the plugin assumes your body joins in (half of the extra turn by default) and shifts your eyes sideways and back, like leaning to look over your shoulder. At 120° that's about 7 cm sideways (the body has turned 30°), and about 13 cm when you look straight back. The view itself stays 1:1 with your head. If you want to see further behind you without twisting as far, raise **Body: +view** in the settings window. That adds extra view yaw past the threshold. **Body: lag** makes the body follow a little slower than your head.
* Neck model and body turn follow your real head angles, before gain, so a yaw gain above 1 doesn't swing your eyes further than your head actually turned.
* Yaw drifts slowly because the Game Rotation Vector ignores the magnetometer on purpose. Tap recenter now and then.
* SimpleBLE (the Bluetooth library) is free for personal, non-commercial use under its license.
