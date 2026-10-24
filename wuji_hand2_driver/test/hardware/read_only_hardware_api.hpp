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

#ifndef HARDWARE__READ_ONLY_HARDWARE_API_HPP_
#define HARDWARE__READ_ONLY_HARDWARE_API_HPP_

#include <wuji_sdk.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wuji_driver_common/stream/status.hpp"
#include "wuji_hand2_driver/sdk/hand2_sdk_api.hpp"

namespace wuji_hand2_driver::hardware_test {

using HardwareStatus = wuji_driver_common::stream::Status;

struct ReadOnlyHardwareApi {
  using FingertipSubscribeFn = decltype(&wuji_hand_2_subscribe_fingertip_thumb_data);

  decltype(&wuji_init) init{nullptr};
  decltype(&wuji_shutdown) shutdown{nullptr};
  decltype(&wuji_last_error) last_error{nullptr};
  decltype(&wuji_connect_options_default) connect_options_default{nullptr};
  decltype(&wuji_scan) scan{nullptr};
  decltype(&wuji_discovered_free) discovered_free{nullptr};
  decltype(&wuji_connect) connect{nullptr};
  decltype(&wuji_dev_disconnect) disconnect{nullptr};
  decltype(&wuji_dev_release) release{nullptr};
  decltype(&wuji_dev_serial_number) device_serial_number{nullptr};
  decltype(&wuji_dev_model) device_model{nullptr};
  decltype(&wuji_dev_is_connected) device_is_connected{nullptr};
  decltype(&wuji_sub_close) sub_close{nullptr};
  decltype(&wuji_hand_2_subscribe_joint_states) subscribe_joint_states{nullptr};
  decltype(&wuji_hand_2_subscribe_joint_diagnostics) subscribe_joint_diagnostics{nullptr};
  decltype(&wuji_hand_2_subscribe_imu) subscribe_imu{nullptr};
  std::array<FingertipSubscribeFn, 5> subscribe_fingertip{};
  decltype(&wuji_hand_2_get_handedness) get_handedness{nullptr};
  decltype(&wuji_hand_2_get_hw_version) get_hw_version{nullptr};

  static ReadOnlyHardwareApi from(const sdk::Hand2SdkApi& source) noexcept {
    ReadOnlyHardwareApi result;
    result.init = source.init;
    result.shutdown = source.shutdown;
    result.last_error = source.last_error;
    result.connect_options_default = source.connect_options_default;
    result.scan = source.scan;
    result.discovered_free = source.discovered_free;
    result.connect = source.connect;
    result.disconnect = source.disconnect;
    result.release = source.release;
    result.device_serial_number = source.device_serial_number;
    result.device_model = source.device_model;
    result.device_is_connected = source.device_is_connected;
    result.sub_close = source.sub_close;
    result.subscribe_joint_states = source.subscribe_joint_states;
    result.subscribe_joint_diagnostics = source.subscribe_joint_diagnostics;
    result.subscribe_imu = source.subscribe_imu;
    result.subscribe_fingertip = source.subscribe_fingertip;
    result.get_handedness = source.get_handedness;
    result.get_hw_version = source.get_hw_version;
    return result;
  }

  static ReadOnlyHardwareApi production() noexcept {
    return from(sdk::Hand2SdkApi::production());
  }
};

inline HardwareStatus read_only_sdk_status(ReadOnlyHardwareApi& api, std::string_view operation,
                                           WujiStatus status) {
  if (status == WUJI_STATUS_OK) {
    return HardwareStatus::success();
  }
  const char* raw_error = api.last_error();
  const std::string error = raw_error == nullptr ? std::string{} : std::string{raw_error};
  std::string message{operation};
  message += " failed (status ";
  message += std::to_string(static_cast<int>(status));
  message += ")";
  if (!error.empty()) {
    message += ": ";
    message += error;
  }
  return HardwareStatus::failure(std::move(message));
}

struct ReadOnlyConnectConfig {
  std::string endpoint;
  std::string serial_number;
  double connect_timeout{10.0};
};

class ReadOnlyHardwareSession {
 public:
  explicit ReadOnlyHardwareSession(ReadOnlyHardwareApi& api) noexcept : api_(api) {}

  ~ReadOnlyHardwareSession() noexcept {
    disconnect();
  }

  ReadOnlyHardwareSession(const ReadOnlyHardwareSession&) = delete;
  ReadOnlyHardwareSession& operator=(const ReadOnlyHardwareSession&) = delete;

  HardwareStatus connect_device(const ReadOnlyConnectConfig& config) {
    if (device_ != nullptr || runtime_active_) {
      return HardwareStatus::failure("read-only session is already connected");
    }
    if (config.endpoint.empty() && config.serial_number.empty()) {
      return HardwareStatus::failure("read-only connect requires endpoint or serial_number");
    }
    constexpr double kMaximumTimeout =
        static_cast<double>(std::numeric_limits<std::uint16_t>::max()) / 1000.0;
    if (!std::isfinite(config.connect_timeout) || config.connect_timeout <= 0.0 ||
        config.connect_timeout > kMaximumTimeout) {
      return HardwareStatus::failure(
          "read-only connect timeout is outside the SDK millisecond range");
    }

    auto status = read_only_sdk_status(api_, "wuji_init", api_.init(nullptr));
    if (!status) {
      return status;
    }
    runtime_active_ = true;

    auto options = api_.connect_options_default();
    const double timeout_ms = std::ceil(config.connect_timeout * 1000.0);
    options.timeout_ms = static_cast<std::uint32_t>(std::clamp(
        timeout_ms, 1.0, static_cast<double>(std::numeric_limits<std::uint16_t>::max())));
    options.retry_count = 0;

    const std::string& target_value =
        config.endpoint.empty() ? config.serial_number : config.endpoint;
    const WujiConnectTarget target{
        static_cast<std::uint8_t>(config.endpoint.empty() ? WUJI_CONNECT_TARGET_KIND_SN
                                                          : WUJI_CONNECT_TARGET_KIND_ADDR),
        target_value.c_str()};
    WujiDevice* connected_device = nullptr;
    const auto connect_result = api_.connect(&target, "wuji_hand_2", &options, &connected_device);
    device_ = connected_device;
    status = read_only_sdk_status(api_, "wuji_connect", connect_result);
    if (!status) {
      return fail(std::move(status));
    }
    if (device_ == nullptr) {
      return fail(HardwareStatus::failure("wuji_connect succeeded without a device handle"));
    }

    status =
        read_device_string(api_.device_serial_number, "wuji_dev_serial_number", serial_number_);
    if (!status) {
      return fail(std::move(status));
    }
    if (serial_number_.empty()) {
      return fail(HardwareStatus::failure("wuji_dev_serial_number returned an empty value"));
    }
    if (!config.serial_number.empty() && serial_number_ != config.serial_number) {
      return fail(HardwareStatus::failure("connected serial_number is " + serial_number_ +
                                          ", expected " + config.serial_number));
    }

    status = read_device_string(api_.device_model, "wuji_dev_model", model_);
    if (!status) {
      return fail(std::move(status));
    }
    if (model_ != "0x0002") {
      return fail(HardwareStatus::failure("connected model is " + model_ + ", expected 0x0002"));
    }
    if (api_.device_is_connected(device_) == 0) {
      return fail(HardwareStatus::failure("wuji_dev_is_connected reported a disconnected device"));
    }

    WujiHandedness handedness{};
    const auto handedness_result = api_.get_handedness(device_, &handedness);
    status = read_only_sdk_status(api_, "wuji_hand_2_get_handedness", handedness_result);
    if (!status) {
      return fail(std::move(status));
    }
    if (handedness == WUJI_HANDEDNESS_LEFT) {
      hand_side_ = "left";
    } else if (handedness == WUJI_HANDEDNESS_RIGHT) {
      hand_side_ = "right";
    } else {
      return fail(HardwareStatus::failure("wuji_hand_2_get_handedness returned an unknown value"));
    }
    return HardwareStatus::success();
  }

  void disconnect() noexcept {
    if (device_ != nullptr) {
      auto* device = std::exchange(device_, nullptr);
      static_cast<void>(api_.disconnect(device));
      api_.release(device);
    }
    serial_number_.clear();
    model_.clear();
    hand_side_.clear();
    if (runtime_active_) {
      runtime_active_ = false;
      api_.shutdown();
    }
  }

  WujiDevice* device() const noexcept {
    return device_;
  }
  const std::string& serial_number() const noexcept {
    return serial_number_;
  }
  const std::string& model() const noexcept {
    return model_;
  }
  const std::string& hand_side() const noexcept {
    return hand_side_;
  }

 private:
  HardwareStatus read_device_string(decltype(&wuji_dev_serial_number) getter, const char* operation,
                                    std::string& output) {
    constexpr std::size_t kAttempts = 3;
    constexpr std::size_t kTerminatorAndProbeBytes = 2;
    for (std::size_t attempt = 0; attempt < kAttempts; ++attempt) {
      const auto required = getter(device_, nullptr, 0);
      if (required < 0) {
        return read_only_sdk_status(api_, operation, static_cast<WujiStatus>(required));
      }
      std::vector<char> buffer(static_cast<std::size_t>(required) + kTerminatorAndProbeBytes, '\0');
      const auto written = getter(device_, buffer.data(), buffer.size());
      if (written < 0) {
        return read_only_sdk_status(api_, operation, static_cast<WujiStatus>(written));
      }
      const auto confirmed = getter(device_, nullptr, 0);
      if (confirmed < 0) {
        return read_only_sdk_status(api_, operation, static_cast<WujiStatus>(confirmed));
      }
      if (written <= required && confirmed == written) {
        output.assign(buffer.data(), static_cast<std::size_t>(written));
        return HardwareStatus::success();
      }
    }
    return HardwareStatus::failure(std::string{operation} + " did not stabilize after 3 attempts");
  }

  HardwareStatus fail(HardwareStatus status) noexcept {
    disconnect();
    return status;
  }

  ReadOnlyHardwareApi& api_;
  WujiDevice* device_{nullptr};
  std::string serial_number_;
  std::string model_;
  std::string hand_side_;
  bool runtime_active_{false};
};

}  // namespace wuji_hand2_driver::hardware_test

#endif  // HARDWARE__READ_ONLY_HARDWARE_API_HPP_
