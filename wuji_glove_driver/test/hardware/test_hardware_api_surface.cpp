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

#include <type_traits>

#include "wuji_glove_driver/sdk/glove_sdk_api.hpp"

namespace wuji_glove_driver::hardware_test {
namespace {

#define WUJI_DECLARE_HAS_MEMBER(member)     \
  template <typename T, typename = void>    \
  struct Has_##member : std::false_type {}; \
  template <typename T>                     \
  struct Has_##member<T, std::void_t<decltype(&T::member)>> : std::true_type {}

WUJI_DECLARE_HAS_MEMBER(reboot);
WUJI_DECLARE_HAS_MEMBER(set);
WUJI_DECLARE_HAS_MEMBER(calibrate);
WUJI_DECLARE_HAS_MEMBER(calibration);
WUJI_DECLARE_HAS_MEMBER(command);
WUJI_DECLARE_HAS_MEMBER(command_publisher);
WUJI_DECLARE_HAS_MEMBER(tactile_sensitivity);
WUJI_DECLARE_HAS_MEMBER(set_tactile_sensitivity);

#undef WUJI_DECLARE_HAS_MEMBER

using Api = sdk::GloveSdkApi;
static_assert(std::is_standard_layout_v<Api>);
static_assert(
    sizeof(Api) ==
        sizeof(decltype(Api::init)) + sizeof(decltype(Api::shutdown)) +
            sizeof(decltype(Api::last_error)) + sizeof(decltype(Api::connect_options_default)) +
            sizeof(decltype(Api::scan)) + sizeof(decltype(Api::discovered_free)) +
            sizeof(decltype(Api::connect)) + sizeof(decltype(Api::disconnect)) +
            sizeof(decltype(Api::release)) + sizeof(decltype(Api::device_serial_number)) +
            sizeof(decltype(Api::device_model)) + sizeof(decltype(Api::device_is_connected)) +
            sizeof(decltype(Api::get_hand_side)) + sizeof(decltype(Api::sub_close)) +
            sizeof(decltype(Api::subscribe_joint_angles)) +
            sizeof(decltype(Api::subscribe_emf_poses)) +
            sizeof(decltype(Api::subscribe_tip_poses)) +
            sizeof(decltype(Api::subscribe_hand_skeleton)) +
            sizeof(decltype(Api::subscribe_imu_palm)) + sizeof(decltype(Api::subscribe_tactile)) +
            sizeof(decltype(Api::subscribe_tactile_residual)) +
            sizeof(decltype(Api::subscribe_tactile_binary)) +
            sizeof(decltype(Api::subscribe_tactile_zones)) +
            sizeof(decltype(Api::subscribe_tactile_point_cloud)),
    "GloveSdkApi must contain exactly the approved 24 read-only function pointers");
static_assert(!Has_reboot<Api>::value);
static_assert(!Has_set<Api>::value);
static_assert(!Has_calibrate<Api>::value);
static_assert(!Has_calibration<Api>::value);
static_assert(!Has_command<Api>::value);
static_assert(!Has_command_publisher<Api>::value);
static_assert(!Has_tactile_sensitivity<Api>::value);
static_assert(!Has_set_tactile_sensitivity<Api>::value);

TEST(GloveHardwareApiSurface, ProductionTableContainsEveryReadOnlyOperation) {
  const Api api = Api::production();
  EXPECT_NE(api.init, nullptr);
  EXPECT_NE(api.shutdown, nullptr);
  EXPECT_NE(api.last_error, nullptr);
  EXPECT_NE(api.connect_options_default, nullptr);
  EXPECT_NE(api.scan, nullptr);
  EXPECT_NE(api.discovered_free, nullptr);
  EXPECT_NE(api.connect, nullptr);
  EXPECT_NE(api.disconnect, nullptr);
  EXPECT_NE(api.release, nullptr);
  EXPECT_NE(api.device_serial_number, nullptr);
  EXPECT_NE(api.device_model, nullptr);
  EXPECT_NE(api.device_is_connected, nullptr);
  EXPECT_NE(api.get_hand_side, nullptr);
  EXPECT_NE(api.sub_close, nullptr);
  EXPECT_NE(api.subscribe_joint_angles, nullptr);
  EXPECT_NE(api.subscribe_emf_poses, nullptr);
  EXPECT_NE(api.subscribe_tip_poses, nullptr);
  EXPECT_NE(api.subscribe_hand_skeleton, nullptr);
  EXPECT_NE(api.subscribe_imu_palm, nullptr);
  EXPECT_NE(api.subscribe_tactile, nullptr);
  EXPECT_NE(api.subscribe_tactile_residual, nullptr);
  EXPECT_NE(api.subscribe_tactile_binary, nullptr);
  EXPECT_NE(api.subscribe_tactile_zones, nullptr);
  EXPECT_NE(api.subscribe_tactile_point_cloud, nullptr);
}

}  // namespace
}  // namespace wuji_glove_driver::hardware_test
