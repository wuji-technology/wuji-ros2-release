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

#include <wuji_sdk.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <mutex>
#include <string>
#include <utility>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>
#include <wuji_msgs/msg/joint_diagnostics_array.hpp>
#include "hardware_test_config.hpp"
#include "hardware_test_validation.hpp"
#include "read_only_hardware_api.hpp"

#include "wuji_hand2_driver/conversion/imu_conversion.hpp"
#include "wuji_hand2_driver/conversion/joint_diagnostics_conversion.hpp"
#include "wuji_hand2_driver/conversion/joint_mapping.hpp"
#include "wuji_hand2_driver/conversion/joint_state_conversion.hpp"
#include "wuji_hand2_driver/conversion/ros_header_conversion.hpp"

namespace wuji_hand2_driver::hardware_test {
namespace {

constexpr std::size_t kFingertipCount = 5;
constexpr std::size_t kMaximumFingertipPayload = 256;
constexpr const char* kFingertipNames[kFingertipCount] = {"thumb", "index", "middle", "ring",
                                                          "pinky"};

// One machine-readable line per check: RESULT<TAB>PASS|FAIL|SKIP<TAB>check<TAB>message.
void report(const char* status, const std::string& check, std::string message) {
  std::replace_if(
      message.begin(), message.end(),
      [](const char character) { return character == '\t' || character == '\n'; }, ' ');
  std::cout << "RESULT\t" << status << '\t' << check << '\t' << message << std::endl;
}

class RuntimeGuard {
 public:
  explicit RuntimeGuard(ReadOnlyHardwareApi& api) noexcept : api_(api) {}

  ~RuntimeGuard() noexcept {
    if (active_) {
      api_.shutdown();
    }
  }

  void activate() noexcept {
    active_ = true;
  }

 private:
  ReadOnlyHardwareApi& api_;
  bool active_{false};
};

class DiscoveryGuard {
 public:
  explicit DiscoveryGuard(ReadOnlyHardwareApi& api) noexcept : api_(api) {}

  ~DiscoveryGuard() noexcept {
    api_.discovered_free(devices_, count_);
  }

  WujiDiscovered** output() noexcept {
    return &devices_;
  }
  std::size_t* count_output() noexcept {
    return &count_;
  }
  const WujiDiscovered* devices() const noexcept {
    return devices_;
  }
  std::size_t count() const noexcept {
    return count_;
  }

 private:
  ReadOnlyHardwareApi& api_;
  WujiDiscovered* devices_{nullptr};
  std::size_t count_{0};
};

class SubscriptionGuard {
 public:
  explicit SubscriptionGuard(ReadOnlyHardwareApi& api) noexcept : api_(api) {}

  ~SubscriptionGuard() noexcept {
    if (subscription_ != nullptr) {
      api_.sub_close(subscription_);
    }
  }

  WujiSub** output() noexcept {
    return &subscription_;
  }
  bool valid() const noexcept {
    return subscription_ != nullptr;
  }

 private:
  ReadOnlyHardwareApi& api_;
  WujiSub* subscription_{nullptr};
};

class FrameWait {
 public:
  void record_valid() {
    std::lock_guard<std::mutex> lock(mutex_);
    valid_ = true;
    condition_.notify_all();
  }

  void record_invalid(std::string message) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = std::move(message);
  }

  void record_terminal(std::string message) {
    std::lock_guard<std::mutex> lock(mutex_);
    terminal_ = true;
    last_error_ = std::move(message);
    condition_.notify_all();
  }

  bool wait_for(double timeout_seconds, std::string& error) {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto timeout = std::chrono::duration<double>{timeout_seconds};
    if (!condition_.wait_for(lock, timeout, [this]() { return valid_ || terminal_; })) {
      error = last_error_.empty() ? "timed out waiting for a valid frame" : last_error_;
      return false;
    }
    if (!valid_) {
      error = last_error_.empty() ? "stream terminated before a valid frame" : last_error_;
      return false;
    }
    return true;
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::string last_error_;
  bool valid_{false};
  bool terminal_{false};
};

std::string terminal_message(const char* stream_name, WujiFrameKind kind) {
  return std::string{stream_name} + " received terminal frame kind " +
         std::to_string(static_cast<unsigned int>(kind));
}

HardwareStatus scan_for_selected_device(ReadOnlyHardwareApi& api,
                                        const HardwareTestConfig& config) {
  RuntimeGuard runtime(api);
  const auto init_result = api.init(nullptr);
  auto status = read_only_sdk_status(api, "wuji_init", init_result);
  if (!status) {
    return status;
  }
  runtime.activate();

  DiscoveryGuard discovery(api);
  const auto scan_result = api.scan(discovery.output(), discovery.count_output());
  status = read_only_sdk_status(api, "wuji_scan", scan_result);
  if (!status) {
    return status;
  }
  if (discovery.count() != 0 && discovery.devices() == nullptr) {
    return HardwareStatus::failure("wuji_scan returned a count without a device array");
  }

  std::cout << "scan: discovered " << discovery.count() << " device(s)" << std::endl;
  if (config.serial_number.empty()) {
    std::cout << "scan: endpoint mode will connect only " << config.endpoint << std::endl;
    return HardwareStatus::success();
  }

  bool selected_hand_found = false;
  for (std::size_t index = 0; index < discovery.count(); ++index) {
    const auto& device = discovery.devices()[index];
    if (config.serial_number == device.serial_number &&
        device.device_id == WUJI_DEVICE_TYPE_WUJI_HAND_2) {
      selected_hand_found = true;
      break;
    }
  }
  if (!selected_hand_found) {
    return HardwareStatus::failure("scan did not report selected Wuji Hand 2 serial " +
                                   config.serial_number);
  }
  std::cout << "scan: found selected serial " << config.serial_number << std::endl;
  return HardwareStatus::success();
}

ReadOnlyConnectConfig connect_config(const HardwareTestConfig& config) {
  ReadOnlyConnectConfig result;
  result.endpoint = config.endpoint;
  result.serial_number = config.serial_number;
  result.connect_timeout = config.connect_timeout_seconds;
  return result;
}

struct JointStateContext {
  explicit JointStateContext(const conversion::JointMapping& input_mapping)
      : mapping(input_mapping) {}

  FrameWait wait;
  const conversion::JointMapping& mapping;
};

void joint_state_callback(WujiFrameKind kind, const WujiJointStateFrame* frame,
                          void* user_data) noexcept {
  auto& context = *static_cast<JointStateContext*>(user_data);
  if (kind == WUJI_FRAME_KIND_LAG) {
    return;
  }
  if (kind != WUJI_FRAME_KIND_OK || frame == nullptr) {
    context.wait.record_terminal(terminal_message("joint state", kind));
    return;
  }
  try {
    sensor_msgs::msg::JointState output;
    const auto status = conversion::to_joint_state(*frame, context.mapping, output);
    if (status) {
      context.wait.record_valid();
    } else {
      context.wait.record_invalid(status.message);
    }
  } catch (const std::exception& error) {
    context.wait.record_terminal(std::string{"joint state callback exception: "} + error.what());
  } catch (...) {
    context.wait.record_terminal("joint state callback unknown exception");
  }
}

struct JointDiagnosticsContext {
  explicit JointDiagnosticsContext(const conversion::JointMapping& input_mapping)
      : mapping(input_mapping) {}

  FrameWait wait;
  const conversion::JointMapping& mapping;
};

void joint_diagnostics_callback(WujiFrameKind kind, const WujiJointDiagnosticsFrame* frame,
                                void* user_data) noexcept {
  auto& context = *static_cast<JointDiagnosticsContext*>(user_data);
  if (kind == WUJI_FRAME_KIND_LAG) {
    return;
  }
  if (kind != WUJI_FRAME_KIND_OK || frame == nullptr) {
    context.wait.record_terminal(terminal_message("joint diagnostics", kind));
    return;
  }
  try {
    wuji_msgs::msg::JointDiagnosticsArray output;
    const auto status = conversion::to_joint_diagnostics(*frame, context.mapping, output);
    if (status) {
      context.wait.record_valid();
    } else {
      context.wait.record_invalid(status.message);
    }
  } catch (const std::exception& error) {
    context.wait.record_terminal(std::string{"joint diagnostics callback exception: "} +
                                 error.what());
  } catch (...) {
    context.wait.record_terminal("joint diagnostics callback unknown exception");
  }
}

void imu_callback(WujiFrameKind kind, const WujiImuData* frame, void* user_data) noexcept {
  auto& wait = *static_cast<FrameWait*>(user_data);
  if (kind == WUJI_FRAME_KIND_LAG) {
    return;
  }
  if (kind != WUJI_FRAME_KIND_OK || frame == nullptr) {
    wait.record_terminal(terminal_message("IMU", kind));
    return;
  }
  try {
    sensor_msgs::msg::Imu output;
    const auto status = conversion::to_imu(*frame, output);
    if (status) {
      wait.record_valid();
    } else {
      wait.record_invalid(status.message);
    }
  } catch (const std::exception& error) {
    wait.record_terminal(std::string{"IMU callback exception: "} + error.what());
  } catch (...) {
    wait.record_terminal("IMU callback unknown exception");
  }
}

struct FingertipContext {
  FrameWait wait;
};

void fingertip_callback(WujiFrameKind kind, const WujiFingertipSensorData* frame,
                        void* user_data) noexcept {
  auto& context = *static_cast<FingertipContext*>(user_data);
  if (kind == WUJI_FRAME_KIND_LAG) {
    return;
  }
  if (kind != WUJI_FRAME_KIND_OK || frame == nullptr) {
    context.wait.record_terminal(terminal_message("fingertip", kind));
    return;
  }
  try {
    if (frame->info_digest == 0U) {
      context.wait.record_invalid("fingertip metadata digest must be nonzero");
      return;
    }
    if (frame->data_len == 0U || frame->data_len > kMaximumFingertipPayload) {
      context.wait.record_invalid("fingertip payload length must be from 1 through 256 bytes");
      return;
    }
    if (frame->data == nullptr) {
      context.wait.record_invalid("fingertip payload pointer is null");
      return;
    }
    std_msgs::msg::Header header;
    const auto status = conversion::to_ros_header(frame->header, header);
    if (!status) {
      context.wait.record_invalid(status.message);
      return;
    }
    context.wait.record_valid();
  } catch (const std::exception& error) {
    context.wait.record_terminal(std::string{"fingertip callback exception: "} + error.what());
  } catch (...) {
    context.wait.record_terminal("fingertip callback unknown exception");
  }
}

bool wait_for_stream(const char* name, const std::string& check, FrameWait& wait,
                     double timeout_seconds) {
  std::string error;
  if (!wait.wait_for(timeout_seconds, error)) {
    std::cerr << "FAIL: " << name << ": " << error << std::endl;
    report("FAIL", check, error);
    return false;
  }
  std::cout << "PASS: " << name << " delivered a valid frame" << std::endl;
  report("PASS", check, "delivered a valid frame");
  return true;
}

bool check_required_joint_state(ReadOnlyHardwareApi& api, WujiDevice* device,
                                const conversion::JointMapping& mapping, double timeout_seconds) {
  JointStateContext context(mapping);
  SubscriptionGuard subscription(api);
  const auto result =
      api.subscribe_joint_states(device, &joint_state_callback, &context, subscription.output());
  const auto status = read_only_sdk_status(api, "wuji_hand_2_subscribe_joint_states", result);
  if (!status || !subscription.valid()) {
    const std::string message = status ? "SDK returned no subscription handle" : status.message;
    std::cerr << "FAIL: required joint state subscription: " << message << std::endl;
    report("FAIL", "joint_state", message);
    return false;
  }
  return wait_for_stream("joint state", "joint_state", context.wait, timeout_seconds);
}

bool check_optional_joint_diagnostics(ReadOnlyHardwareApi& api, WujiDevice* device,
                                      const conversion::JointMapping& mapping,
                                      double timeout_seconds) {
  JointDiagnosticsContext context(mapping);
  SubscriptionGuard subscription(api);
  const auto result = api.subscribe_joint_diagnostics(device, &joint_diagnostics_callback, &context,
                                                      subscription.output());
  const auto status = read_only_sdk_status(api, "wuji_hand_2_subscribe_joint_diagnostics", result);
  if (!status) {
    if (is_optional_stream_unavailable(result)) {
      std::cout << "SKIP: optional joint diagnostics unavailable: " << status.message << std::endl;
      report("SKIP", "joint_diagnostics", status.message);
      return true;
    }
    std::cerr << "FAIL: optional joint diagnostics: " << status.message << std::endl;
    report("FAIL", "joint_diagnostics", status.message);
    return false;
  }
  if (!subscription.valid()) {
    std::cerr << "FAIL: joint diagnostics subscribe returned no handle" << std::endl;
    report("FAIL", "joint_diagnostics", "subscribe returned no handle");
    return false;
  }
  return wait_for_stream("joint diagnostics", "joint_diagnostics", context.wait, timeout_seconds);
}

bool check_optional_imu(ReadOnlyHardwareApi& api, WujiDevice* device, double timeout_seconds) {
  FrameWait wait;
  SubscriptionGuard subscription(api);
  const auto result = api.subscribe_imu(device, &imu_callback, &wait, subscription.output());
  const auto status = read_only_sdk_status(api, "wuji_hand_2_subscribe_imu", result);
  if (!status) {
    if (is_optional_stream_unavailable(result)) {
      std::cout << "SKIP: optional IMU unavailable: " << status.message << std::endl;
      report("SKIP", "imu", status.message);
      return true;
    }
    std::cerr << "FAIL: optional IMU: " << status.message << std::endl;
    report("FAIL", "imu", status.message);
    return false;
  }
  if (!subscription.valid()) {
    std::cerr << "FAIL: IMU subscribe returned no handle" << std::endl;
    report("FAIL", "imu", "subscribe returned no handle");
    return false;
  }
  return wait_for_stream("IMU", "imu", wait, timeout_seconds);
}

bool check_fingertips(ReadOnlyHardwareApi& api, WujiDevice* device, double timeout_seconds) {
  bool passed = true;
  for (std::size_t finger = 0; finger < kFingertipCount; ++finger) {
    FingertipContext context;
    const std::string check = std::string{"fingertip_"} + kFingertipNames[finger];

    SubscriptionGuard subscription(api);
    const auto result = api.subscribe_fingertip[finger](device, &fingertip_callback, &context,
                                                        subscription.output());
    const auto status = read_only_sdk_status(api, "wuji_hand_2_subscribe_fingertip_data", result);
    if (!status) {
      if (is_optional_stream_unavailable(result)) {
        std::cout << "SKIP: optional " << kFingertipNames[finger]
                  << " fingertip unavailable: " << status.message << std::endl;
        report("SKIP", check, status.message);
        continue;
      }
      std::cerr << "FAIL: " << kFingertipNames[finger]
                << " fingertip subscription: " << status.message << std::endl;
      report("FAIL", check, status.message);
      passed = false;
      continue;
    }
    if (!subscription.valid()) {
      std::cerr << "FAIL: " << kFingertipNames[finger] << " fingertip subscribe returned no handle"
                << std::endl;
      report("FAIL", check, "subscribe returned no handle");
      passed = false;
      continue;
    }
    const std::string stream_name = std::string{kFingertipNames[finger]} + " fingertip";
    if (!wait_for_stream(stream_name.c_str(), check, context.wait, timeout_seconds)) {
      passed = false;
    }
  }
  return passed;
}

int run_read_only_smoke(const HardwareTestConfig& config) {
  ReadOnlyHardwareApi api = ReadOnlyHardwareApi::production();
  auto status = scan_for_selected_device(api, config);
  if (!status) {
    std::cerr << "FAIL: " << status.message << std::endl;
    report("FAIL", "connect", status.message);
    return 1;
  }

  ReadOnlyHardwareSession session(api);
  status = session.connect_device(connect_config(config));
  if (!status) {
    std::cerr << "FAIL: selected-device connection: " << status.message << std::endl;
    report("FAIL", "connect", status.message);
    return 1;
  }

  WujiHwVersion hardware_version{};
  const auto version_result = api.get_hw_version(session.device(), &hardware_version);
  status = read_only_sdk_status(api, "wuji_hand_2_get_hw_version", version_result);
  if (!status) {
    std::cerr << "FAIL: identity hardware version: " << status.message << std::endl;
    report("FAIL", "connect", "identity hardware version: " + status.message);
    return 1;
  }
  wuji_hw_version_free(&hardware_version);
  std::cout << "PASS: connected identity serial=" << session.serial_number()
            << " model=" << session.model() << " hand_side=" << session.hand_side() << std::endl;
  report("PASS", "connect", "serial=" + session.serial_number());

  const auto side =
      session.hand_side() == "left" ? conversion::HandSide::kLeft : conversion::HandSide::kRight;
  const conversion::JointMapping mapping(side);
  // Every check runs even after a failure, so one report covers the whole device.
  bool passed =
      check_required_joint_state(api, session.device(), mapping, config.stream_timeout_seconds);
  passed = check_optional_joint_diagnostics(api, session.device(), mapping,
                                            config.stream_timeout_seconds) &&
           passed;
  passed = check_optional_imu(api, session.device(), config.stream_timeout_seconds) && passed;
  if (config.fingertip_tactile) {
    passed = check_fingertips(api, session.device(), config.stream_timeout_seconds) && passed;
  }
  if (!passed) {
    return 1;
  }

  std::cout << "PASS: read-only Hand 2 hardware smoke completed" << std::endl;
  return 0;
}

}  // namespace
}  // namespace wuji_hand2_driver::hardware_test

int main() {
  return wuji_hand2_driver::hardware_test::run_hardware_exception_boundary(
      "read-only Hand 2 hardware test", std::cerr, []() {
        const auto config = wuji_hand2_driver::hardware_test::load_hardware_test_config(false);
        if (!config.config) {
          std::cerr << config.message << std::endl;
          return config.exit_code;
        }
        return wuji_hand2_driver::hardware_test::run_read_only_smoke(*config.config);
      });
}
