# PlatformIO pre-build script: patches Adafruit BNO08x 1.2.5's SHTP layer.
#
# shtp_open() assumes the sensor sends its advertisement by itself after the
# soft reset, and only waits for it. A GY-BNO08X (SH-2 2.6.6) sends none, so
# the library never learns the channel numbers and begin_I2C fails
# (sh2_getProdIds -> SH2_ERR_BAD_PARAM). Asking for it with the SHTP
# "advertise" command works on every board: a second advertisement is harmless.
Import("env")  # noqa: F821 (provided by PlatformIO)
import os

OLD = "    pShtp->advertPhase = ADVERT_REQUESTED;\n\n    // Open HAL\n"
NEW = "    pShtp->advertPhase = ADVERT_NEEDED;  // patched: ask (patch_bno08x.py)\n\n    // Open HAL\n"

libdeps = env.subst("$PROJECT_LIBDEPS_DIR/$PIOENV")  # noqa: F821
path = os.path.join(libdeps, "Adafruit BNO08x", "src", "shtp.c")
if os.path.exists(path):
    src = open(path).read()
    if NEW not in src:
        if OLD not in src:
            raise SystemExit("patch_bno08x.py: shtp.c changed, update the patch")
        open(path, "w").write(src.replace(OLD, NEW, 1))
        print("patch_bno08x.py: shtp.c patched (request SHTP advertisement)")
