#!/usr/bin/env python3
"""Update the head tracker's firmware over Bluetooth.

    pip install bleak        (inside a venv, or: pipx run --spec bleak ...; see README)
    python3 ota_update.py ../firmware/.pio/build/esp32c3/firmware.bin

X-Plane can keep running; tracking pauses during the update and the plugin
reconnects by itself afterwards. If anything goes wrong the tracker keeps (or
goes back to) its old firmware, so it's safe to retry.
"""

import argparse
import asyncio
import hashlib
import struct
import sys
import time

try:
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakCharacteristicNotFoundError, BleakError
except ImportError:
    sys.exit("This needs the 'bleak' Python package. See the README, section 5, for how to install it.")

CTRL_UUID = "6e7a0011-3c1b-4b8a-9f2e-6b1d5a7c0de1"
DATA_UUID = "6e7a0012-3c1b-4b8a-9f2e-6b1d5a7c0de1"
INFO_UUID = "6e7a0013-3c1b-4b8a-9f2e-6b1d5a7c0de1"

CMD_BEGIN, CMD_END, CMD_ABORT = 0x01, 0x02, 0x03
ST_READY, ST_PROGRESS, ST_DONE, ST_ERROR = 0x10, 0x11, 0x12, 0x1F
ERRORS = {
    1: "bad command",
    2: "image too big for the update slot",
    3: "couldn't prepare the update slot",
    4: "flash write failed",
    5: "size mismatch (data lost in transit?)",
    6: "SHA-256 mismatch (data corrupted in transit?)",
    7: "not a valid ESP32 firmware image",
    8: "tracker wasn't expecting data (update not started)",
    9: "couldn't switch to the new slot",
    10: "the tracker is still checking its last update; try again in a minute",
    11: "the update stalled (no data for 10 s) and was cancelled",
}

ESP_IMAGE_MAGIC = 0xE9
ESP32C3_CHIP_ID = 5


class OtaError(Exception):
    pass


def check_image(image: bytes) -> None:
    """Refuse files that aren't an app image for the ESP32-C3."""
    if len(image) < 24 or image[0] != ESP_IMAGE_MAGIC:
        raise OtaError("this isn't an ESP32 app image (use .pio/build/esp32c3/firmware.bin)")
    chip_id = struct.unpack_from("<H", image, 12)[0]
    if chip_id != ESP32C3_CHIP_ID:
        raise OtaError(f"this image is for a different chip (chip id {chip_id}, need 5 = ESP32-C3)")


async def find_tracker(name: str, address: str | None, timeout: float):
    if address:
        dev = await BleakScanner.find_device_by_address(address, timeout=timeout)
    else:
        dev = await BleakScanner.find_device_by_name(name, timeout=timeout)
    if dev is None:
        raise OtaError(f"couldn't find '{address or name}'. Is it powered and in range?")
    return dev


async def read_info(client) -> dict:
    raw = (await client.read_gatt_char(INFO_UUID)).decode(errors="replace")
    return dict(kv.split("=", 1) for kv in raw.split(";") if "=" in kv)


async def chunk_size(client) -> int:
    # On Linux, BlueZ only reports the real MTU after this call.
    backend = getattr(client, "_backend", None)
    if hasattr(backend, "_acquire_mtu"):
        try:
            await backend._acquire_mtu()
        except Exception:
            pass
    mtu = client.mtu_size
    # 512 is the largest single write. With a small MTU, BlueZ splits a
    # 512-byte write into a "long write" itself, which is still correct.
    return 512 if mtu <= 23 else max(20, min(512, mtu - 3))


async def send_image(client, image: bytes, log=print) -> None:
    status_q: asyncio.Queue = asyncio.Queue()

    def on_status(_, data: bytearray):
        if len(data) >= 6:
            code, err = data[0], data[1]
            received = struct.unpack_from("<I", data, 2)[0]
            status_q.put_nowait((code, err, received))

    async def wait_for(code_wanted: int, timeout: float):
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise OtaError("timed out waiting for the tracker")
            try:
                code, err, received = await asyncio.wait_for(status_q.get(), left)
            except asyncio.TimeoutError:
                raise OtaError("timed out waiting for the tracker") from None
            if code == ST_ERROR:
                raise OtaError(f"tracker reported: {ERRORS.get(err, f'error {err}')}")
            if code == code_wanted:
                return received

    def check_errors():
        while not status_q.empty():
            code, err, _ = status_q.get_nowait()
            if code == ST_ERROR:
                raise OtaError(f"tracker reported: {ERRORS.get(err, f'error {err}')}")

    await client.start_notify(CTRL_UUID, on_status)
    sha = hashlib.sha256(image).digest()
    try:
        # Inside the try, so a timeout or Ctrl-C while the slot is being
        # prepared still tells the tracker to stop (and let it sleep again).
        await client.write_gatt_char(CTRL_UUID, struct.pack("<BI", CMD_BEGIN, len(image)) + sha, response=True)
        log("Preparing the update slot...")
        await wait_for(ST_READY, 30)  # erasing flash takes a few seconds

        size = await chunk_size(client)
        start = time.monotonic()
        sent = 0
        while sent < len(image):
            chunk = image[sent:sent + size]
            await client.write_gatt_char(DATA_UUID, chunk, response=True)
            sent += len(chunk)
            check_errors()
            pct = 100 * sent // len(image)
            rate = sent / max(0.001, time.monotonic() - start) / 1024
            log(f"\rSending {pct:3d}%  {sent // 1024} / {len(image) // 1024} KB  {rate:.1f} KB/s", end="")
        log("")
        await client.write_gatt_char(CTRL_UUID, bytes([CMD_END]), response=True)
        log("Verifying...")
        await wait_for(ST_DONE, 30)
    except BaseException:
        try:
            await client.write_gatt_char(CTRL_UUID, bytes([CMD_ABORT]), response=True)
        except Exception:
            pass
        raise


async def confirm_new_firmware(args, log=print) -> dict:
    """Reconnect after the reboot and wait for the new firmware to confirm itself."""
    log("Tracker is rebooting into the new firmware...")
    await asyncio.sleep(3)
    deadline = time.monotonic() + 90
    info = {}
    while time.monotonic() < deadline:
        try:
            dev = await find_tracker(args.name, args.address, 10)
            async with BleakClient(dev) as client:
                while time.monotonic() < deadline:
                    info = await read_info(client)
                    if info.get("state") == "valid":
                        return info
                    await asyncio.sleep(2)
        except Exception:
            await asyncio.sleep(2)
    return info


async def main_async(args) -> int:
    try:
        with open(args.firmware, "rb") as f:
            image = f.read()
    except OSError as e:
        raise OtaError(f"can't read {args.firmware}: {e.strerror}")
    check_image(image)
    print(f"Firmware: {args.firmware} ({len(image) // 1024} KB)")

    dev = await find_tracker(args.name, args.address, 15)
    print(f"Found {dev.name or args.name} ({dev.address})")
    async with BleakClient(dev) as client:
        try:
            before = await read_info(client)
        except BleakCharacteristicNotFoundError:
            raise OtaError("this tracker's firmware doesn't support Bluetooth updates yet. "
                           "Flash it once over USB (pio run -t upload).")
        print(f"Running: version {before.get('version')} in {before.get('slot')}")
        if before.get("state") == "probation":
            raise OtaError("the tracker is still checking its last update. Wait a minute and try again.")
        await send_image(client, image)
        # The tracker reboots now. If that drops the link before we let go of it
        # (or the plugin shares the same connection), disconnecting may fail.
        try:
            await client.disconnect()
        except Exception:
            pass

    info = await confirm_new_firmware(args)
    if info.get("slot") and info.get("slot") == before.get("slot"):
        # Booting the new slot failed (crash, hang, or not healthy in time), and
        # the bootloader went back to the old one, which reports "valid".
        print("The new firmware didn't start properly, so the tracker went back to the old one.")
        return 1
    if info.get("state") == "valid":
        print(f"Done. Now running version {info.get('version')} in {info.get('slot')}.")
        return 0
    print("Couldn't confirm the new firmware. If the tracker misbehaves, power-cycle it: "
          "unconfirmed firmware is rolled back automatically.")
    return 1


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("firmware", help="firmware.bin from .pio/build/esp32c3/")
    p.add_argument("--name", default="HeadTrackerC3", help="Bluetooth name (default HeadTrackerC3)")
    p.add_argument("--address", help="Bluetooth address, if you have several trackers")
    args = p.parse_args()
    try:
        return asyncio.run(main_async(args))
    except OtaError as e:
        print(f"\nUpdate failed: {e}", file=sys.stderr)
        return 1
    except BleakError as e:
        print(f"\nUpdate failed: Bluetooth error: {e}\n"
              "Check Bluetooth is on (bluetoothctl show) and the tracker is in range, then try again. "
              "The tracker keeps its current firmware.", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nCancelled. The tracker keeps its current firmware.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
