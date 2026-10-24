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
#include <wuji_sdk.h>

#include "wuji_glove_driver/sdk/glove_sdk_api.hpp"
#include "wuji_glove_driver/sdk/sdk_status.hpp"

namespace wuji_glove_driver::sdk {
namespace {

const char* last_error_for_test = nullptr;

extern "C" const char* test_last_error() {
  return last_error_for_test;
}

TEST(GloveSdkApi, ProductionTablePointsDirectlyAtSdkFunctions) {
  const auto api = GloveSdkApi::production();
  EXPECT_EQ(api.init, &wuji_init);
  EXPECT_EQ(api.shutdown, &wuji_shutdown);
  EXPECT_EQ(api.last_error, &wuji_last_error);
  EXPECT_EQ(api.connect_options_default, &wuji_connect_options_default);
  EXPECT_EQ(api.scan, &wuji_scan);
  EXPECT_EQ(api.discovered_free, &wuji_discovered_free);
  EXPECT_EQ(api.connect, &wuji_connect);
  EXPECT_EQ(api.disconnect, &wuji_dev_disconnect);
  EXPECT_EQ(api.release, &wuji_dev_release);
  EXPECT_EQ(api.device_serial_number, &wuji_dev_serial_number);
  EXPECT_EQ(api.device_model, &wuji_dev_model);
  EXPECT_EQ(api.device_is_connected, &wuji_dev_is_connected);
  EXPECT_EQ(api.get_hand_side, &wuji_glove_get_hand_side);
  EXPECT_EQ(api.sub_close, &wuji_sub_close);
  EXPECT_EQ(api.subscribe_joint_angles, &wuji_glove_subscribe_hand_joint_angles);
  EXPECT_EQ(api.subscribe_emf_poses, &wuji_glove_subscribe_emf_poses);
  EXPECT_EQ(api.subscribe_tip_poses, &wuji_glove_subscribe_tip_poses);
  EXPECT_EQ(api.subscribe_hand_skeleton, &wuji_glove_subscribe_hand_skeleton);
  EXPECT_EQ(api.subscribe_imu_palm, &wuji_glove_subscribe_imu_palm);
  EXPECT_EQ(api.subscribe_tactile, &wuji_glove_subscribe_tactile);
  EXPECT_EQ(api.subscribe_tactile_residual, &wuji_glove_subscribe_tactile_residual);
  EXPECT_EQ(api.subscribe_tactile_binary, &wuji_glove_subscribe_tactile_binary);
  EXPECT_EQ(api.subscribe_tactile_zones, &wuji_glove_subscribe_tactile_zones);
  EXPECT_EQ(api.subscribe_tactile_point_cloud, &wuji_glove_subscribe_tactile_point_cloud);
}

TEST(GloveSdkStatus, CapturesLastErrorBeforeTheSdkStateChanges) {
  auto api = GloveSdkApi::production();
  api.last_error = &test_last_error;

  last_error_for_test = "temporary failure";

  const Status status = sdk_status(api, "wuji_scan", static_cast<WujiStatus>(1));
  last_error_for_test = "different failure";

  EXPECT_FALSE(status);
  EXPECT_EQ(status.message, "wuji_scan failed (status 1): temporary failure");
}

}  // namespace
}  // namespace wuji_glove_driver::sdk
