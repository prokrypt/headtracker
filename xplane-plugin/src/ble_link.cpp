#include "ble_link.h"

#include <cctype>
#include <cmath>
#include <cstring>
#include <optional>
#include <simpleble/SimpleBLE.h>

namespace ht {

namespace {
// Must match the firmware.
const char *kServiceUuid = "6e7a0001-3c1b-4b8a-9f2e-6b1d5a7c0de1";
const char *kQuatCharUuid = "6e7a0002-3c1b-4b8a-9f2e-6b1d5a7c0de1";
const char *kCtrlCharUuid = "6e7a0003-3c1b-4b8a-9f2e-6b1d5a7c0de1";
const size_t kPacketSize = 14;  // firmware 1.2.0+; older firmware sent 18 bytes

bool sameUuid(std::string a, std::string b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if (std::tolower((unsigned char)a[i]) != std::tolower((unsigned char)b[i])) return false;
  return true;
}

// A tracker advertises our service UUID. With a name set, only that one.
bool isTracker(SimpleBLE::Peripheral &p, const std::string &name) {
  try {
    if (!name.empty()) return p.identifier() == name;
    for (auto &s : p.services())  // before connecting: the advertised UUIDs
      if (sameUuid(s.uuid(), kServiceUuid)) return true;
  } catch (...) {}
  return false;
}
}  // namespace

std::string BleLink::trackerName() {
  std::lock_guard<std::mutex> lock(nameMutex_);
  return connected_ ? trackerName_ : "";
}

void BleLink::start() {
  if (thread_.joinable()) return;
  stop_ = false;
  finished_ = false;
  thread_ = std::thread([this] {
    run();
    std::lock_guard<std::mutex> lock(doneMutex_);
    finished_ = true;
    doneCv_.notify_all();
  });
}

bool BleLink::stopFor(std::chrono::milliseconds timeout) {
  stop_ = true;
  cmdCv_.notify_all();
  if (!thread_.joinable()) return true;
  {
    std::unique_lock<std::mutex> lock(doneMutex_);
    if (!doneCv_.wait_for(lock, timeout, [this] { return finished_.load(); })) return false;
  }
  thread_.join();
  return true;
}

void BleLink::stop() {
  stop_ = true;
  cmdCv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

Sample BleLink::latest() {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  return shared_->sample;
}

bool BleLink::send(uint8_t command) {
  if (!connected_ || !hasControl_) return false;
  {
    std::lock_guard<std::mutex> lock(cmdMutex_);
    if (commands_.size() < 16) commands_.push_back(command);
  }
  cmdCv_.notify_all();
  return true;
}

bool BleLink::popReply(uint8_t &command, uint8_t &result) {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  if (shared_->replies.empty()) return false;
  command = shared_->replies.front().first;
  result = shared_->replies.front().second;
  shared_->replies.pop_front();
  return true;
}

bool BleLink::popLog(std::string &line) {
  std::lock_guard<std::mutex> lock(logMutex_);
  if (logs_.empty()) return false;
  line = std::move(logs_.front());
  logs_.pop_front();
  return true;
}

void BleLink::log(const std::string &line) {
  std::lock_guard<std::mutex> lock(logMutex_);
  if (logs_.size() < 100) logs_.push_back(line);
}

void BleLink::sleepFor(std::chrono::milliseconds ms) {
  auto until = std::chrono::steady_clock::now() + ms;
  while (!stop_ && std::chrono::steady_clock::now() < until)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void BleLink::run() {
  bool warnedNoAdapter = false;
  std::optional<SimpleBLE::Peripheral> current;
  auto dropConnection = [&current]() {
    if (!current) return;
    try {
      if (current->is_connected()) {
        try { current->unsubscribe(kServiceUuid, kQuatCharUuid); } catch (...) {}
        try { current->unsubscribe(kServiceUuid, kCtrlCharUuid); } catch (...) {}
        current->disconnect();
      }
    } catch (...) {}
    current.reset();
  };
  while (!stop_) {
    try {
      if (!SimpleBLE::Adapter::bluetooth_enabled()) {
        if (!warnedNoAdapter) log("Bluetooth is off or unavailable, waiting");
        warnedNoAdapter = true;
        sleepFor(std::chrono::seconds(3));
        continue;
      }
      auto adapters = SimpleBLE::Adapter::get_adapters();
      if (adapters.empty()) {
        if (!warnedNoAdapter) log("No Bluetooth adapter found, waiting");
        warnedNoAdapter = true;
        sleepFor(std::chrono::seconds(3));
        continue;
      }
      warnedNoAdapter = false;
      SimpleBLE::Adapter adapter = adapters.front();

      // Scan until the tracker shows up (or a few seconds pass).
      std::mutex foundMutex;
      std::optional<SimpleBLE::Peripheral> found;
      // "Updated" too: the scan response (name) and UUIDs can come after the
      // first sighting.
      auto onSeen = [&](SimpleBLE::Peripheral p) {
        if (isTracker(p, deviceName_)) {
          std::lock_guard<std::mutex> lock(foundMutex);
          if (!found) found = p;
        }
      };
      adapter.set_callback_on_scan_found(onSeen);
      adapter.set_callback_on_scan_updated(onSeen);
      adapter.scan_start();
      auto scanEnd = std::chrono::steady_clock::now() + std::chrono::seconds(4);
      while (!stop_ && std::chrono::steady_clock::now() < scanEnd) {
        {
          std::lock_guard<std::mutex> lock(foundMutex);
          if (found) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      adapter.scan_stop();
      adapter.set_callback_on_scan_found([](SimpleBLE::Peripheral) {});
      adapter.set_callback_on_scan_updated([](SimpleBLE::Peripheral) {});

      std::optional<SimpleBLE::Peripheral> target;
      {
        std::lock_guard<std::mutex> lock(foundMutex);
        target = found;
      }
      if (!target || stop_) continue;

      current = *target;
      SimpleBLE::Peripheral &peripheral = *current;
      std::string name = peripheral.identifier();
      if (name.empty()) name = "tracker";
      {
        std::lock_guard<std::mutex> lock(nameMutex_);
        trackerName_ = name;
      }
      log("Found " + name + " (" + peripheral.address() + "), connecting");
      peripheral.connect();
      if (stop_) break;  // the plugin was disabled while connecting

      uint32_t connectionId = ++connections_;
      std::weak_ptr<Shared> weak = shared_;
      peripheral.notify(kServiceUuid, kQuatCharUuid, [weak, connectionId](SimpleBLE::ByteArray payload) {
        if (payload.size() != kPacketSize) return;
        auto sh = weak.lock();
        if (!sh) return;
        const uint8_t *p = reinterpret_cast<const uint8_t *>(payload.data());
        float f[3];
        std::memcpy(f, p, sizeof(f));  // little-endian floats, same as x86_64
        if (!std::isfinite(f[0]) || !std::isfinite(f[1]) || !std::isfinite(f[2])) return;
        std::lock_guard<std::mutex> lock(sh->mutex);
        sh->sample.head = {f[0], f[1], f[2]};
        sh->sample.flags = p[12];
        sh->sample.count++;
        sh->sample.connection = connectionId;
        sh->sample.time = std::chrono::steady_clock::now();
      });
      // Firmware 1.2.0+ has a control characteristic for recenter and calibration.
      bool hasControl = false;
      for (auto &service : peripheral.services()) {
        if (service.uuid() != kServiceUuid) continue;
        for (auto &c : service.characteristics())
          if (c.uuid() == kCtrlCharUuid) hasControl = true;
      }
      if (hasControl) {
        peripheral.notify(kServiceUuid, kCtrlCharUuid, [weak](SimpleBLE::ByteArray payload) {
          if (payload.size() < 2) return;
          auto sh = weak.lock();
          if (!sh) return;
          std::lock_guard<std::mutex> lock(sh->mutex);
          if (sh->replies.size() < 16) sh->replies.emplace_back(uint8_t(payload[0]), uint8_t(payload[1]));
        });
      } else {
        log("The tracker's firmware is too old for this plugin (needs 1.2.0 or newer, with "
            "recentering and calibration on the tracker). Update it, see the README.");
      }
      {
        std::lock_guard<std::mutex> lock(cmdMutex_);
        commands_.clear();  // anything queued was meant for an older connection
      }
      hasControl_ = hasControl;
      connected_ = true;
      log("Connected, receiving head data");

      while (!stop_ && peripheral.is_connected()) {
        std::deque<uint8_t> todo;
        {
          std::unique_lock<std::mutex> lock(cmdMutex_);
          cmdCv_.wait_for(lock, std::chrono::milliseconds(200),
                          [this] { return stop_ || !commands_.empty(); });
          todo.swap(commands_);
        }
        for (uint8_t cmd : todo) {
          if (stop_) break;
          SimpleBLE::ByteArray data(&cmd, 1);
          peripheral.write_request(kServiceUuid, kCtrlCharUuid, data);
        }
      }

      hasControl_ = false;
      connected_ = false;
      dropConnection();
      if (!stop_) log("Tracker disconnected, searching again");
    } catch (const std::exception &e) {
      connected_ = false;
      hasControl_ = false;
      // Don't leave a half-set-up link behind (e.g. connected but subscribe failed).
      dropConnection();
      log(std::string("Bluetooth error: ") + e.what());
      sleepFor(std::chrono::seconds(2));
    } catch (...) {
      // Anything else escaping this thread would take X-Plane down with it.
      connected_ = false;
      hasControl_ = false;
      dropConnection();
      log("Bluetooth error (unknown)");
      sleepFor(std::chrono::seconds(2));
    }
  }
  dropConnection();
}

}  // namespace ht
