# Connection-event timing: notifications arriving within 3 ms count as one event.
import asyncio, struct, sys, time
from bleak import BleakScanner, BleakClient
HEAD = "6e7a0002-3c1b-4b8a-9f2e-6b1d5a7c0de1"
async def main():
    dev = await BleakScanner.find_device_by_filter(lambda d, a: "6e7a0001-3c1b-4b8a-9f2e-6b1d5a7c0de1" in (a.service_uuids or []), timeout=10)
    async with BleakClient(dev) as c:
        ts = []
        await c.start_notify(HEAD, lambda _, d: ts.append(time.perf_counter()))
        await asyncio.sleep(1.0); ts.clear()   # let any conn-param update settle
        await asyncio.sleep(6.0)
    ev = [[ts[0]]]
    for t in ts[1:]:
        (ev[-1].append(t) if t - ev[-1][-1] < 0.003 else ev.append([t]))
    gaps = sorted((b[0] - a[0]) * 1000 for a, b in zip(ev, ev[1:]))
    sizes = sorted(len(e) for e in ev)
    pct = lambda l, q: l[min(len(l) - 1, int(q * len(l)))]
    print(f"{len(ts)} packets ({len(ts)/6:.1f}/s) in {len(ev)} bursts")
    print(f"burst spacing ms: min {gaps[0]:.1f} p10 {pct(gaps,.1):.1f} median {pct(gaps,.5):.1f} p90 {pct(gaps,.9):.1f} max {gaps[-1]:.1f}")
    print(f"packets per burst: median {pct(sizes,.5)} max {sizes[-1]}")
asyncio.run(main())
