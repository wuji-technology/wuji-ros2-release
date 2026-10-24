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

#include "wuji_glove_driver/sdk/glove_session.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace wuji_glove_driver::sdk {
namespace {

constexpr char kDeviceAlias[] = "wuji_glove";
constexpr char kGloveModel[] = "0x0001";
constexpr std::size_t kIdentityReadAttempts = 3;
constexpr std::chrono::milliseconds kRetryDelay{50};

struct RuntimeState {
  std::mutex mutex;
  decltype(&wuji_init) init{nullptr};
  decltype(&wuji_shutdown) shutdown{nullptr};
  std::size_t users{0};
};

RuntimeState& runtime_state() {
  static RuntimeState state;
  return state;
}

struct Candidate {
  std::string serial_number;
  std::string address;
  WujiTransportType transport;
  WujiDeviceType device_type;
};

class DiscoveredGuard {
 public:
  DiscoveredGuard(GloveSdkApi& sdk, WujiDiscovered* devices, std::size_t count) noexcept
      : sdk_(sdk), devices_(devices), count_(count) {}

  ~DiscoveredGuard() noexcept {
    sdk_.discovered_free(devices_, count_);
  }

 private:
  GloveSdkApi& sdk_;
  WujiDiscovered* devices_;
  std::size_t count_;
};

template <std::size_t Size>
Status copy_fixed_string(const char (&source)[Size], const char* field, std::string& output) {
  const auto terminator = std::find(std::begin(source), std::end(source), '\0');
  if (terminator == std::end(source)) {
    return Status::failure(std::string{"wuji_scan returned non-terminated "} + field);
  }
  output.assign(std::begin(source), terminator);
  return Status::success();
}

std::string fallback_identity(const Candidate& candidate) {
  return "fallback:" + std::to_string(static_cast<unsigned int>(candidate.transport)) + ":" +
         candidate.address + ":" + std::to_string(static_cast<unsigned int>(candidate.device_type));
}

bool side_matches_serial(const std::string& serial_number, const std::string& hand_side) {
  if (serial_number.size() < 4) {
    return false;
  }
  if (serial_number[3] == 'J') {
    return hand_side == "left";
  }
  if (serial_number[3] == 'K') {
    return hand_side == "right";
  }
  return false;
}

}  // namespace

GloveSession::GloveSession(GloveSdkApi& sdk, GloveSessionHooks hooks)
    : sdk_(sdk), hooks_(std::move(hooks)) {
  if (!hooks_.now) {
    hooks_.now = []() { return std::chrono::steady_clock::now(); };
  }
  if (!hooks_.sleep_for) {
    hooks_.sleep_for = [](std::chrono::steady_clock::duration duration) {
      std::this_thread::sleep_for(duration);
    };
  }
  if (!hooks_.cancelled) {
    hooks_.cancelled = []() { return false; };
  }
}

GloveSession::~GloveSession() noexcept {
  disconnect();
}

Status GloveSession::acquire_runtime() {
  auto& runtime = runtime_state();
  const std::lock_guard<std::mutex> lock(runtime.mutex);
  if (runtime.users != 0) {
    if (runtime.init != sdk_.init || runtime.shutdown != sdk_.shutdown) {
      return Status::failure("another SDK function table owns the process runtime");
    }
    ++runtime.users;
    runtime_acquired_ = true;
    return Status::success();
  }

  const auto result = sdk_.init(nullptr);
  const auto status = sdk_status(sdk_, "wuji_init", result);
  if (!status) {
    return status;
  }
  runtime.init = sdk_.init;
  runtime.shutdown = sdk_.shutdown;
  runtime.users = 1;
  runtime_acquired_ = true;
  return Status::success();
}

void GloveSession::release_runtime() noexcept {
  if (!runtime_acquired_) {
    return;
  }
  runtime_acquired_ = false;

  auto& runtime = runtime_state();
  const std::lock_guard<std::mutex> lock(runtime.mutex);
  if (runtime.users == 0) {
    return;
  }
  --runtime.users;
  if (runtime.users == 0) {
    const auto shutdown = runtime.shutdown;
    runtime.init = nullptr;
    runtime.shutdown = nullptr;
    shutdown();
  }
}

Status GloveSession::check_wait_state(const std::optional<TimePoint>& deadline) const {
  if (hooks_.cancelled()) {
    return Status::failure("device connection cancelled");
  }
  if (deadline && hooks_.now() >= *deadline) {
    return Status::failure("connect_timeout expired while connecting to a Wuji Glove");
  }
  return Status::success();
}

Status GloveSession::read_device_string(decltype(&wuji_dev_serial_number) getter,
                                        const char* operation, std::string& output,
                                        const std::optional<TimePoint>& deadline) {
  for (std::size_t attempt = 0; attempt < kIdentityReadAttempts; ++attempt) {
    auto status = check_wait_state(deadline);
    if (!status) {
      return status;
    }
    const auto required = getter(device_, nullptr, 0);
    if (required < 0) {
      return sdk_status(sdk_, operation, static_cast<WujiStatus>(required));
    }
    std::vector<char> buffer(static_cast<std::size_t>(required) + 2, '\0');
    status = check_wait_state(deadline);
    if (!status) {
      return status;
    }
    const auto written = getter(device_, buffer.data(), buffer.size());
    if (written < 0) {
      return sdk_status(sdk_, operation, static_cast<WujiStatus>(written));
    }
    status = check_wait_state(deadline);
    if (!status) {
      return status;
    }
    const auto confirmed = getter(device_, nullptr, 0);
    if (confirmed < 0) {
      return sdk_status(sdk_, operation, static_cast<WujiStatus>(confirmed));
    }
    if (written <= required && confirmed == written) {
      output.assign(buffer.data(), static_cast<std::size_t>(written));
      return Status::success();
    }
  }
  return Status::failure(std::string{operation} + " did not stabilize after " +
                         std::to_string(kIdentityReadAttempts) + " attempts");
}

Status GloveSession::read_hand_side(std::string& output, const std::optional<TimePoint>& deadline) {
  auto status = check_wait_state(deadline);
  if (!status) {
    return status;
  }
  std::size_t needed = 0;
  const auto probe = sdk_.get_hand_side(device_, nullptr, 0, &needed);
  if (probe != WUJI_STATUS_ERR_BUFFER_TOO_SMALL) {
    return sdk_status(sdk_, "wuji_glove_get_hand_side", probe);
  }
  if (needed == 0) {
    return Status::failure("wuji_glove_get_hand_side returned an empty size");
  }
  status = check_wait_state(deadline);
  if (!status) {
    return status;
  }
  std::vector<char> buffer(needed, '\0');
  const auto result = sdk_.get_hand_side(device_, buffer.data(), buffer.size(), &needed);
  if (result != WUJI_STATUS_OK) {
    return sdk_status(sdk_, "wuji_glove_get_hand_side", result);
  }
  const auto terminator = std::find(buffer.begin(), buffer.end(), '\0');
  if (terminator == buffer.end()) {
    return Status::failure("wuji_glove_get_hand_side returned a non-terminated value");
  }
  output.assign(buffer.begin(), terminator);
  return Status::success();
}

Status GloveSession::connect(const GloveConnectConfig& config) {
  if (device_ != nullptr || runtime_acquired_) {
    return Status::failure("session is already connected");
  }
  if (!config.hand_side.empty() && config.hand_side != "left" && config.hand_side != "right") {
    return Status::failure("hand_side must be left, right, or empty");
  }
  constexpr double kMaxTimeoutSeconds =
      static_cast<double>(std::numeric_limits<std::uint32_t>::max()) / 1000.0;
  if (!std::isfinite(config.connect_timeout) || config.connect_timeout < 0.0 ||
      config.connect_timeout > kMaxTimeoutSeconds) {
    return Status::failure("connect_timeout is outside the SDK millisecond range");
  }

  const auto start = hooks_.now();
  const std::optional<TimePoint> deadline =
      config.connect_timeout == 0.0
          ? std::nullopt
          : std::optional<TimePoint>{start +
                                     std::chrono::duration_cast<TimePoint::duration>(
                                         std::chrono::duration<double>{config.connect_timeout})};

  auto status = acquire_runtime();
  if (!status) {
    return status;
  }
  bool rollback = true;
  const auto rollback_session = [this, &rollback]() {
    if (rollback) {
      disconnect();
    }
  };
  struct RollbackGuard {
    decltype(rollback_session)& action;
    ~RollbackGuard() {
      action();
    }
  } guard{rollback_session};

  std::string target_value;
  std::uint8_t target_kind = WUJI_CONNECT_TARGET_KIND_SN;

  if (!config.endpoint.empty()) {
    target_kind = WUJI_CONNECT_TARGET_KIND_ADDR;
    target_value = config.endpoint;
  } else if (!config.serial_number.empty()) {
    target_kind = WUJI_CONNECT_TARGET_KIND_SN;
    target_value = config.serial_number;
  } else if (!config.hand_side.empty()) {
    while (true) {
      status = check_wait_state(deadline);
      if (!status) {
        return status;
      }
      WujiDiscovered* discovered = nullptr;
      std::size_t discovered_count = 0;
      const auto scan_result = sdk_.scan(&discovered, &discovered_count);
      DiscoveredGuard discovered_guard(sdk_, discovered, discovered_count);
      if (scan_result != WUJI_STATUS_OK) {
        return sdk_status(sdk_, "wuji_scan", scan_result);
      }
      if (discovered == nullptr && discovered_count != 0) {
        return Status::failure("wuji_scan returned a null array with nonzero count");
      }

      std::vector<Candidate> candidates;
      std::set<std::string> identities;
      for (std::size_t index = 0; index < discovered_count; ++index) {
        if (discovered[index].device_id != WUJI_DEVICE_TYPE_WUJI_GLOVE) {
          continue;
        }
        Candidate candidate;
        status = copy_fixed_string(discovered[index].serial_number, "serial_number",
                                   candidate.serial_number);
        if (!status) {
          return status;
        }
        status = copy_fixed_string(discovered[index].address, "address", candidate.address);
        if (!status) {
          return status;
        }
        candidate.transport = discovered[index].transport;
        candidate.device_type = discovered[index].device_id;
        if (!side_matches_serial(candidate.serial_number, config.hand_side)) {
          continue;
        }
        const std::string identity = candidate.serial_number.empty()
                                         ? fallback_identity(candidate)
                                         : "serial:" + candidate.serial_number;
        if (identities.insert(identity).second) {
          candidates.push_back(std::move(candidate));
        }
      }
      if (candidates.size() > 1) {
        return Status::failure("multiple matching Wuji Gloves found; set serial_number");
      }
      if (candidates.size() == 1) {
        target_kind = candidates.front().serial_number.empty() ? WUJI_CONNECT_TARGET_KIND_ADDR
                                                               : WUJI_CONNECT_TARGET_KIND_SN;
        target_value = candidates.front().serial_number.empty() ? candidates.front().address
                                                                : candidates.front().serial_number;
        break;
      }
      auto delay = std::chrono::duration_cast<std::chrono::steady_clock::duration>(kRetryDelay);
      if (deadline) {
        delay = std::min(delay, *deadline - hooks_.now());
      }
      if (delay > std::chrono::steady_clock::duration::zero()) {
        hooks_.sleep_for(delay);
      }
    }
  } else {
    while (true) {
      status = check_wait_state(deadline);
      if (!status) {
        return status;
      }
      WujiDiscovered* discovered = nullptr;
      std::size_t discovered_count = 0;
      const auto scan_result = sdk_.scan(&discovered, &discovered_count);
      DiscoveredGuard discovered_guard(sdk_, discovered, discovered_count);
      if (scan_result != WUJI_STATUS_OK) {
        return sdk_status(sdk_, "wuji_scan", scan_result);
      }
      if (discovered == nullptr && discovered_count != 0) {
        return Status::failure("wuji_scan returned a null array with nonzero count");
      }

      std::vector<Candidate> candidates;
      std::set<std::string> identities;
      for (std::size_t index = 0; index < discovered_count; ++index) {
        if (discovered[index].device_id != WUJI_DEVICE_TYPE_WUJI_GLOVE) {
          continue;
        }
        Candidate candidate;
        status = copy_fixed_string(discovered[index].serial_number, "serial_number",
                                   candidate.serial_number);
        if (!status) {
          return status;
        }
        status = copy_fixed_string(discovered[index].address, "address", candidate.address);
        if (!status) {
          return status;
        }
        candidate.transport = discovered[index].transport;
        candidate.device_type = discovered[index].device_id;
        const std::string identity = candidate.serial_number.empty()
                                         ? fallback_identity(candidate)
                                         : "serial:" + candidate.serial_number;
        if (identities.insert(identity).second) {
          candidates.push_back(std::move(candidate));
        }
      }
      if (candidates.size() > 1) {
        return Status::failure("multiple Wuji Gloves found; set serial_number");
      }
      if (candidates.size() == 1) {
        target_kind = candidates.front().serial_number.empty() ? WUJI_CONNECT_TARGET_KIND_ADDR
                                                               : WUJI_CONNECT_TARGET_KIND_SN;
        target_value = candidates.front().serial_number.empty() ? candidates.front().address
                                                                : candidates.front().serial_number;
        break;
      }
      auto delay = std::chrono::duration_cast<std::chrono::steady_clock::duration>(kRetryDelay);
      if (deadline) {
        delay = std::min(delay, *deadline - hooks_.now());
      }
      if (delay > std::chrono::steady_clock::duration::zero()) {
        hooks_.sleep_for(delay);
      }
    }
  }

  status = check_wait_state(deadline);
  if (!status) {
    return status;
  }
  auto options = sdk_.connect_options_default();
  options.retry_count = 0;
  // The driver owns its device connection. Without the Bridge the SDK never falls back
  // to Zenoh, so wuji_connect returns once device discovery has answered.
  options.enable_bridge = false;
  status = check_wait_state(deadline);
  if (!status) {
    return status;
  }
  if (deadline) {
    const auto remaining = *deadline - hooks_.now();
    const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
    if (remaining <= std::chrono::steady_clock::duration::zero() || remaining_ms.count() <= 0) {
      return Status::failure("connect_timeout expired while connecting to a Wuji Glove");
    }
    options.timeout_ms = static_cast<std::uint32_t>(
        std::min<std::int64_t>(remaining_ms.count(), std::numeric_limits<std::uint32_t>::max()));
  }
  status = check_wait_state(deadline);
  if (!status) {
    return status;
  }
  const WujiConnectTarget target{target_kind, target_value.c_str()};
  while (true) {
    WujiDevice* connected_device = nullptr;
    const auto connect_result = sdk_.connect(&target, kDeviceAlias, &options, &connected_device);
    if (connect_result == WUJI_STATUS_OK) {
      device_ = connected_device;
      break;
    }
    const auto connect_status = sdk_status(sdk_, "wuji_connect", connect_result);
    if (connected_device != nullptr) {
      static_cast<void>(sdk_.disconnect(connected_device));
      sdk_.release(connected_device);
    }
    if (deadline) {
      return connect_status;
    }
    status = check_wait_state(deadline);
    if (!status) {
      return status;
    }
    hooks_.sleep_for(std::chrono::duration_cast<std::chrono::steady_clock::duration>(kRetryDelay));
    status = check_wait_state(deadline);
    if (!status) {
      return status;
    }
  }
  if (device_ == nullptr) {
    return Status::failure("wuji_connect succeeded without a device handle");
  }

  status = check_wait_state(deadline);
  if (!status) {
    return status;
  }
  status = read_device_string(sdk_.device_serial_number, "wuji_dev_serial_number", serial_number_,
                              deadline);
  if (!status) {
    return status;
  }
  if (serial_number_.empty()) {
    return Status::failure("wuji_dev_serial_number returned an empty value");
  }
  status = check_wait_state(deadline);
  if (!status) {
    return status;
  }
  status = read_device_string(sdk_.device_model, "wuji_dev_model", model_, deadline);
  if (!status) {
    return status;
  }
  status = check_wait_state(deadline);
  if (!status) {
    return status;
  }
  status = read_hand_side(hand_side_, deadline);
  if (!status) {
    return status;
  }
  status = check_wait_state(deadline);
  if (!status) {
    return status;
  }

  if (!config.serial_number.empty() && serial_number_ != config.serial_number) {
    return Status::failure("connected serial_number is " + serial_number_ + ", expected " +
                           config.serial_number);
  }
  if (model_ != kGloveModel) {
    return Status::failure("connected model is " + model_ + ", expected " + kGloveModel);
  }
  if (hand_side_ != "left" && hand_side_ != "right") {
    return Status::failure("wuji_glove_get_hand_side returned unsupported value " + hand_side_);
  }
  if (!config.hand_side.empty() && hand_side_ != config.hand_side) {
    return Status::failure("connected hand_side is " + hand_side_ + ", expected " +
                           config.hand_side);
  }
  status = check_wait_state(deadline);
  if (!status) {
    return status;
  }
  if (sdk_.device_is_connected(device_) == 0) {
    return Status::failure("wuji_dev_is_connected reported a disconnected device");
  }
  status = check_wait_state(deadline);
  if (!status) {
    return status;
  }

  rollback = false;
  return Status::success();
}

WujiDevice* GloveSession::device() const noexcept {
  return device_;
}

std::string GloveSession::serial_number() const {
  return serial_number_;
}

std::string GloveSession::model() const {
  return model_;
}

std::string GloveSession::hand_side() const {
  return hand_side_;
}

void GloveSession::disconnect() noexcept {
  if (device_ != nullptr) {
    auto* device = std::exchange(device_, nullptr);
    static_cast<void>(sdk_.disconnect(device));
    sdk_.release(device);
  }
  serial_number_.clear();
  model_.clear();
  hand_side_.clear();
  release_runtime();
}

}  // namespace wuji_glove_driver::sdk
