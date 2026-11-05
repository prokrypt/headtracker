// Firmware updates over Bluetooth LE, with automatic rollback.
//
// Service 6e7a0010-..., three characteristics:
//   0011 control  write + notify   commands from the PC, status back to it
//   0012 data     write, write without response
//                                  raw image bytes, in order. Without response
//                                  is the fast path: keep at most ~8 KB beyond
//                                  the last PROGRESS count in flight.
//   0013 info     read             "version=...;slot=...;state=..."
//
// Control commands (PC -> tracker):
//   01 <u32 size> <32-byte sha256>   begin (tracker erases the slot, then says READY)
//   02                               end   (verify, switch slots, reboot)
//   03                               abort
// Status notifications (tracker -> PC): <u8 code> <u8 error> <u32 bytes received>
//   10 READY, 11 PROGRESS (every 1 KB written to flash, and at the last byte),
//   12 DONE (rebooting), 1F ERROR
//
// After any firmware change, the first PC to connect gets a GATT "Service
// Changed" indication, so BlueZ drops its cached copy of the services.
//
// The new image boots "on probation". If it crashes, resets, or does not reach
// a healthy state (running for 15 s; Bluetooth starts first thing) within 60 s, the bootloader
// goes back to the previous firmware. tools/ota_update.py is the PC side.
#pragma once
#include <NimBLEDevice.h>

void otaSetup(NimBLEServer *server);
void otaLoop();                         // call from loop(); confirms a new image once healthy
void otaOnDisconnect(uint16_t connHandle);
bool otaInProgress();
