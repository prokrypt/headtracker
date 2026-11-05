# Bench test over BLE without X-Plane: reads the info string, collects 5 s of head
# packets (rate, lost packets, angle range) and checks a recenter round-trip.
# Usage: python bletest.py   (needs bleak, e.g. the venv from README section 5)
import asyncio, struct, sys, time
from bleak import BleakScanner, BleakClient
HEAD = "6e7a0002-3c1b-4b8a-9f2e-6b1d5a7c0de1"
CTRL = "6e7a0003-3c1b-4b8a-9f2e-6b1d5a7c0de1"
INFO = "6e7a0013-3c1b-4b8a-9f2e-6b1d5a7c0de1"
async def main():
    t0 = time.time()
    dev = await BleakScanner.find_device_by_name("HeadTrackerC3", timeout=10)
    if not dev: sys.exit("HeadTrackerC3 not found")
    print(f"found {dev.address} in {time.time()-t0:.1f}s")
    async with BleakClient(dev) as c:
        print(f"connected, MTU {c.mtu_size}")
        print("info:", (await c.read_gatt_char(INFO)).decode(errors="replace"))
        pk = []; replies = []
        def on_head(_, d):
            if len(d) != 14: print("bad length", len(d)); return
            y, p, r = struct.unpack("<3f", d[:12]); pk.append((time.time(), y, p, r, d[12], d[13]))
        await c.start_notify(HEAD, on_head)
        await c.start_notify(CTRL, lambda _, d: replies.append(bytes(d)))
        await asyncio.sleep(5)
        n = len(pk); dur = pk[-1][0] - pk[0][0]
        seqs = [x[5] for x in pk]; lost = sum(((b - a) % 256) - 1 for a, b in zip(seqs, seqs[1:]))
        gaps = sorted(b[0] - a[0] for a, b in zip(pk, pk[1:]))
        print(f"{n} packets in {dur:.2f}s = {n/dur:.1f}/s, seq gaps (lost) {lost}, "
              f"interval median {gaps[len(gaps)//2]*1000:.1f}ms max {gaps[-1]*1000:.1f}ms")
        ys = [x[1] for x in pk]; ps = [x[2] for x in pk]; rs = [x[3] for x in pk]
        print(f"yaw {min(ys):.1f}..{max(ys):.1f}  pitch {min(ps):.1f}..{max(ps):.1f}  roll {min(rs):.1f}..{max(rs):.1f}")
        f = pk[-1][4]; print(f"flags 0x{f:02x}: button={f&1} calibrated={(f>>1)&1} recenters={f>>4}")
        before = pk[-1][4] >> 4
        await c.write_gatt_char(CTRL, b"\x01", response=True)
        await asyncio.sleep(0.3)
        after = pk[-1]
        print(f"recenter: reply {[r.hex() for r in replies]}, count {before} -> {after[4]>>4}, "
              f"angles right after {after[1]:.2f} {after[2]:.2f} {after[3]:.2f}")
asyncio.run(main())
