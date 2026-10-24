// Copyright 2026 Wuji Technology
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef HARDWARE__READ_ONLY_STREAM_PROBE_HPP_
#define HARDWARE__READ_ONLY_STREAM_PROBE_HPP_

#include <wuji_sdk.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <iostream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "hardware_test_validation.hpp"

#include "wuji_glove_driver/sdk/glove_sdk_api.hpp"
#include "wuji_glove_driver/sdk/sdk_status.hpp"

namespace wuji_glove_driver::hardware_test {

class SubscriptionSet {
 public:
  explicit SubscriptionSet(sdk::GloveSdkApi& api) : api_(api) {}
  ~SubscriptionSet() noexcept {
    close_all();
  }

  void close_all() noexcept {
    for (auto iterator = subscriptions_.rbegin(); iterator != subscriptions_.rend(); ++iterator) {
      iterator->begin_close(iterator->probe);
      api_.sub_close(iterator->subscription);
    }
    subscriptions_.clear();
  }

  template <typename Probe>
  void add(WujiSub* subscription, Probe& probe) {
    try {
      subscriptions_.push_back(Entry{
          subscription,
          [](void* target) noexcept { static_cast<Probe*>(target)->begin_close(); },
          &probe,
      });
    } catch (...) {
      api_.sub_close(subscription);
      throw;
    }
  }

 private:
  using BeginClose = void (*)(void*) noexcept;  // NOLINT(readability/casting)

  struct Entry {
    WujiSub* subscription;
    BeginClose begin_close;
    void* probe;
  };

  sdk::GloveSdkApi& api_;
  std::vector<Entry> subscriptions_;
};

template <typename Probe, typename Subscribe>
bool start_stream(const char* name, bool required, bool enabled, WujiDevice* device,
                  sdk::GloveSdkApi& api, SubscriptionSet& subscriptions, Probe& probe,
                  Subscribe subscribe, std::vector<std::pair<const char*, Probe*>>& active,
                  std::string& error) {
  if (!enabled) {
    std::cout << "SKIP: " << name << " disabled by publish flag\n";
    return true;
  }
  WujiSub* subscription = nullptr;
  const WujiStatus result = subscribe(device, &Probe::callback, &probe, &subscription);
  if (result != WUJI_STATUS_OK) {
    const auto subscribe_status = sdk::sdk_status(api, std::string{"subscribe "} + name, result);
    if (subscription != nullptr) {
      probe.begin_close();
      api.sub_close(subscription);
    }
    if (!required && is_optional_stream_unavailable(result)) {
      std::cout << "SKIP: " << name << " unavailable: " << subscribe_status.message << '\n';
      return true;
    }
    error = subscribe_status.message;
    return false;
  }
  if (subscription == nullptr) {
    error = std::string{"subscribe "} + name + " succeeded without a subscription handle";
    return false;
  }
  subscriptions.add(subscription, probe);
  active.emplace_back(name, &probe);
  return true;
}

template <typename Payload, typename Message, typename Converter>
class StreamProbe {
 public:
  StreamProbe(std::string name, const std::atomic_bool& cancelled)
      : name_(std::move(name)), cancelled_(cancelled) {}

  static void callback(WujiFrameKind kind, const Payload* frame, void* user_data) noexcept {
    if (user_data == nullptr) {
      return;
    }
    auto& self = *static_cast<StreamProbe*>(user_data);
    try {
      self.receive(kind, frame);
    } catch (const std::exception& exception) {
      self.fail(std::string{"conversion exception: "} + exception.what());
    } catch (...) {
      self.fail("unknown conversion exception");
    }
  }

  bool wait_until(std::chrono::steady_clock::time_point deadline, std::string& error) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!valid_ && !terminal_ && !cancelled_.load()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        error = name_ + ": timed out waiting for an OK frame";
        return false;
      }
      condition_.wait_until(lock, std::min(deadline, now + std::chrono::milliseconds{100}));
    }
    if (cancelled_.load()) {
      error = name_ + ": interrupted";
      return false;
    }
    return state_is_success_locked(error);
  }

  bool snapshot_success(std::string& error) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_is_success_locked(error);
  }

  void begin_close() {
    std::lock_guard<std::mutex> lock(mutex_);
    closing_ = true;
  }

  bool check_after_close(std::string& error) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_is_success_locked(error);
  }

 private:
  bool state_is_success_locked(std::string& error) const {
    if (terminal_) {
      error = name_ + ": " + error_;
      return false;
    }
    if (!valid_) {
      error = name_ + ": no convertible OK frame received";
      return false;
    }
    return true;
  }

  void receive(WujiFrameKind kind, const Payload* frame) {
    if (kind == WUJI_FRAME_KIND_END) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!closing_) {
        fail_locked("received terminal frame kind " +
                    std::to_string(static_cast<unsigned int>(kind)));
      }
      return;
    }
    if (kind == WUJI_FRAME_KIND_ERROR) {
      fail("received terminal frame kind " + std::to_string(static_cast<unsigned int>(kind)));
      return;
    }
    if (kind != WUJI_FRAME_KIND_OK) {
      return;
    }
    if (frame == nullptr) {
      fail("received an OK frame with null payload");
      return;
    }
    Message output;
    const auto status = Converter{}(*frame, output);
    if (!status) {
      fail("message conversion failed: " + status.message);
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    valid_ = true;
    condition_.notify_all();
  }

  void fail(std::string error) {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_locked(std::move(error));
  }

  void fail_locked(std::string error) {
    terminal_ = true;
    error_ = std::move(error);
    condition_.notify_all();
  }

  std::string name_;
  const std::atomic_bool& cancelled_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::string error_;
  bool valid_{false};
  bool terminal_{false};
  bool closing_{false};
};

}  // namespace wuji_glove_driver::hardware_test

#endif  // HARDWARE__READ_ONLY_STREAM_PROBE_HPP_
