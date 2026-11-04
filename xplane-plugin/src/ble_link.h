// Background Bluetooth LE link to the ESP32-C3 head tracker.
#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "head_math.h"

namespace ht {

// Head packet from the firmware. The tracker does the mount correction and
// recentering itself, so these are already the head's angles away from
// "straight ahead" (degrees, X-Plane's conventions).
struct Sample {
  Angles head;
  uint8_t flags = 0;                              // bit1 = mount calibrated, bits 4-7 = recenter count
  uint32_t count = 0;                             // total packets received
  uint32_t connection = 0;                        // bumps on every new connection
  std::chrono::steady_clock::time_point time{};   // when it arrived
};

class BleLink {
 public:
  explicit BleLink(std::string deviceName) : deviceName_(std::move(deviceName)) {}
  ~BleLink() { stop(); }

  void start();
  void stop();  // waits for the Bluetooth thread, however long it takes
  // Asks the thread to stop and waits at most `timeout`. False if it's still
  // busy (stuck in a Bluetooth connect, which can take tens of seconds when the
  // tracker vanished in the middle of it): keep this object alive until
  // finished() says so, then destroy it.
  bool stopFor(std::chrono::milliseconds timeout);
  bool finished() const { return finished_; }

  Sample latest();
  bool connected() const { return connected_; }
  // False when the connected tracker runs firmware from before 1.2.0, which
  // can't take commands (recenter, calibration).
  bool canCommand() const { return hasControl_; }

  // Commands to the tracker (see the firmware's main.cpp). Sent from the
  // Bluetooth thread; false if not connected or the firmware is too old.
  enum : uint8_t { kRecenter = 0x01, kCalAhead = 0x02, kCalNod = 0x03, kCalReset = 0x04 };
  bool send(uint8_t command);
  // Replies from the tracker: the command and its result (0 = OK).
  bool popReply(uint8_t &command, uint8_t &result);
  bool popLog(std::string &line);  // logs are drained on X-Plane's main thread

 private:
  void run();
  void log(const std::string &line);
  void sleepFor(std::chrono::milliseconds ms);

  // Written by SimpleBLE's own thread. Held through a shared_ptr so a late
  // notification can never touch a BleLink that has already been destroyed.
  struct Shared {
    std::mutex mutex;
    Sample sample;
    std::deque<std::pair<uint8_t, uint8_t>> replies;
  };

  std::mutex cmdMutex_;
  std::condition_variable cmdCv_;
  std::deque<uint8_t> commands_;

  std::string deviceName_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> hasControl_{false};
  std::atomic<bool> finished_{false};  // run() has returned
  std::mutex doneMutex_;
  std::condition_variable doneCv_;
  uint32_t connections_ = 0;  // only touched by the BLE thread

  std::shared_ptr<Shared> shared_ = std::make_shared<Shared>();

  std::mutex logMutex_;
  std::deque<std::string> logs_;
};

}  // namespace ht
