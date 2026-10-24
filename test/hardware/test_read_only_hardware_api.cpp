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

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <type_traits>

#include "read_only_hardware_api.hpp"

namespace wuji_hand2_driver::hardware_test {
namespace {

std::vector<std::string> recording_calls;
int recording_device_storage = 0;
bool recording_target_valid = false;

WujiDevice* recording_device() noexcept {
  return reinterpret_cast<WujiDevice*>(&recording_device_storage);
}

WujiStatus recording_init(const WujiInitOptions*) {
  recording_calls.emplace_back("init");
  return WUJI_STATUS_OK;
}

void recording_shutdown() {
  recording_calls.emplace_back("shutdown");
}

const char* recording_last_error() {
  return "recording fake error";
}

WujiConnectOptions recording_connect_options_default() {
  return WujiConnectOptions{};
}

WujiStatus recording_connect(const WujiConnectTarget* target, const char* alias,
                             const WujiConnectOptions* options, WujiDevice** output) {
  recording_calls.emplace_back("connect");
  recording_target_valid = target != nullptr && target->kind == WUJI_CONNECT_TARGET_KIND_SN &&
                           std::string{target->value} == "SN123" &&
                           std::string{alias} == "wuji_hand_2" && options != nullptr &&
                           options->retry_count == 0 && output != nullptr;
  *output = recording_device();
  return WUJI_STATUS_OK;
}

WujiStatus recording_disconnect(WujiDevice* device) {
  recording_calls.emplace_back("disconnect");
  return device == recording_device() ? WUJI_STATUS_OK : WUJI_STATUS_ERR_INVALID_ARG;
}

void recording_release(WujiDevice* device) {
  if (device == recording_device()) {
    recording_calls.emplace_back("release");
  }
}

std::int32_t recording_string(const char* name, const std::string& value, WujiDevice* device,
                              char* buffer, std::size_t buffer_size) {
  recording_calls.emplace_back(name);
  if (device != recording_device()) {
    return static_cast<std::int32_t>(WUJI_STATUS_ERR_INVALID_ARG);
  }
  if (buffer == nullptr) {
    return static_cast<std::int32_t>(value.size());
  }
  if (buffer_size <= value.size()) {
    return static_cast<std::int32_t>(WUJI_STATUS_ERR_INVALID_ARG);
  }
  std::memcpy(buffer, value.c_str(), value.size() + 1);
  return static_cast<std::int32_t>(value.size());
}

std::int32_t recording_serial(WujiDevice* device, char* buffer, std::size_t buffer_size) {
  return recording_string("serial", "SN123", device, buffer, buffer_size);
}

std::int32_t recording_model(WujiDevice* device, char* buffer, std::size_t buffer_size) {
  return recording_string("model", "0x0002", device, buffer, buffer_size);
}

std::uint8_t recording_is_connected(WujiDevice* device) {
  recording_calls.emplace_back("is_connected");
  return device == recording_device() ? 1U : 0U;
}

WujiStatus recording_handedness(WujiDevice* device, WujiHandedness* output) {
  recording_calls.emplace_back("handedness");
  if (device != recording_device() || output == nullptr) {
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  *output = WUJI_HANDEDNESS_RIGHT;
  return WUJI_STATUS_OK;
}

#define WUJI_DECLARE_HAS_MEMBER(member)     \
  template <typename T, typename = void>    \
  struct Has_##member : std::false_type {}; \
  template <typename T>                     \
  struct Has_##member<T, std::void_t<decltype(&T::member)>> : std::true_type {}

WUJI_DECLARE_HAS_MEMBER(enable);
WUJI_DECLARE_HAS_MEMBER(disable);
WUJI_DECLARE_HAS_MEMBER(clear_fault);
WUJI_DECLARE_HAS_MEMBER(config);
WUJI_DECLARE_HAS_MEMBER(configure);
WUJI_DECLARE_HAS_MEMBER(calibrate);
WUJI_DECLARE_HAS_MEMBER(command);
WUJI_DECLARE_HAS_MEMBER(emergency_stop);
WUJI_DECLARE_HAS_MEMBER(set_effort_limit);
WUJI_DECLARE_HAS_MEMBER(set_effort_limits);
WUJI_DECLARE_HAS_MEMBER(set_mit_params);
WUJI_DECLARE_HAS_MEMBER(calibrate_fingertip);
WUJI_DECLARE_HAS_MEMBER(get_fingertip_status);
WUJI_DECLARE_HAS_MEMBER(get_fingertip_info);
WUJI_DECLARE_HAS_MEMBER(fingertip_online);
WUJI_DECLARE_HAS_MEMBER(open_joint_command_publisher);
WUJI_DECLARE_HAS_MEMBER(send_joint_commands);
WUJI_DECLARE_HAS_MEMBER(close_joint_command_publisher);
WUJI_DECLARE_HAS_MEMBER(open_command_publisher);
WUJI_DECLARE_HAS_MEMBER(send);
WUJI_DECLARE_HAS_MEMBER(disable_all);

#undef WUJI_DECLARE_HAS_MEMBER

#define WUJI_ASSERT_NO_CONTROL_MEMBER(type, member) \
  static_assert(!Has_##member<type>::value, #type " must not expose " #member)

WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, enable);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, disable);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, clear_fault);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, config);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, configure);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, calibrate);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, command);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, emergency_stop);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, set_effort_limit);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, set_effort_limits);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, set_mit_params);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, calibrate_fingertip);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, get_fingertip_status);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, get_fingertip_info);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, fingertip_online);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, open_joint_command_publisher);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, send_joint_commands);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareApi, close_joint_command_publisher);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareSession, open_command_publisher);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareSession, config);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareSession, configure);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareSession, calibrate);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareSession, command);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareSession, send);
WUJI_ASSERT_NO_CONTROL_MEMBER(ReadOnlyHardwareSession, disable_all);

#undef WUJI_ASSERT_NO_CONTROL_MEMBER

TEST(ReadOnlyHardwareApi, ProductionFactoryCopiesOnlyAllowedOperations) {
  const ReadOnlyHardwareApi api = ReadOnlyHardwareApi::production();

  EXPECT_NE(api.init, nullptr);
  EXPECT_NE(api.shutdown, nullptr);
  EXPECT_NE(api.last_error, nullptr);
  EXPECT_NE(api.scan, nullptr);
  EXPECT_NE(api.connect, nullptr);
  EXPECT_NE(api.disconnect, nullptr);
  EXPECT_NE(api.release, nullptr);
  EXPECT_NE(api.device_serial_number, nullptr);
  EXPECT_NE(api.device_model, nullptr);
  EXPECT_NE(api.subscribe_joint_states, nullptr);
  EXPECT_NE(api.subscribe_joint_diagnostics, nullptr);
  EXPECT_NE(api.subscribe_imu, nullptr);
  EXPECT_NE(api.sub_close, nullptr);
  for (const auto subscribe : api.subscribe_fingertip) {
    EXPECT_NE(subscribe, nullptr);
  }
}

TEST(ReadOnlyHardwareSession, RecordingFakeUsesOnlyReadAndLifecycleOperations) {
  recording_calls.clear();
  recording_target_valid = false;
  ReadOnlyHardwareApi api;
  api.init = &recording_init;
  api.shutdown = &recording_shutdown;
  api.last_error = &recording_last_error;
  api.connect_options_default = &recording_connect_options_default;
  api.connect = &recording_connect;
  api.disconnect = &recording_disconnect;
  api.release = &recording_release;
  api.device_serial_number = &recording_serial;
  api.device_model = &recording_model;
  api.device_is_connected = &recording_is_connected;
  api.get_handedness = &recording_handedness;

  {
    ReadOnlyHardwareSession session(api);
    ReadOnlyConnectConfig config;
    config.serial_number = "SN123";
    config.connect_timeout = 1.0;
    const HardwareStatus status = session.connect_device(config);
    ASSERT_TRUE(status) << status.message;
    EXPECT_TRUE(recording_target_valid);
    EXPECT_EQ(session.serial_number(), "SN123");
    EXPECT_EQ(session.model(), "0x0002");
    EXPECT_EQ(session.hand_side(), "right");
  }

  const std::vector<std::string> expected_calls = {
      "init",  "connect",      "serial",     "serial",     "serial",  "model",    "model",
      "model", "is_connected", "handedness", "disconnect", "release", "shutdown",
  };
  EXPECT_EQ(recording_calls, expected_calls);
}

}  // namespace
}  // namespace wuji_hand2_driver::hardware_test
