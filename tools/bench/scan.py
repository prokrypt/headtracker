import asyncio
from bleak import BleakScanner
SVC = "6e7a0001-3c1b-4b8a-9f2e-6b1d5a7c0de1"
async def main():
    found = await BleakScanner.discover(timeout=6.0, return_adv=True)
    hits = [(d, a) for d, a in found.values() if (d.name or a.local_name or "").startswith("HeadTracker") or SVC in a.service_uuids]
    for d, a in hits:
        print(f"{d.address}  name={a.local_name or d.name}  rssi={a.rssi}  services={a.service_uuids}")
    print(f"{len(found)} devices seen, {len(hits)} tracker(s)")
asyncio.run(main())
