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

#ifndef WUJI_GLOVE_DRIVER__SDK__GLOVE_SDK_API_HPP_
#define WUJI_GLOVE_DRIVER__SDK__GLOVE_SDK_API_HPP_

#include <wuji_sdk.h>

namespace wuji_glove_driver::sdk {

struct GloveSdkApi {
  decltype(&wuji_init) init;
  decltype(&wuji_shutdown) shutdown;
  decltype(&wuji_last_error) last_error;
  decltype(&wuji_connect_options_default) connect_options_default;
  decltype(&wuji_scan) scan;
  decltype(&wuji_discovered_free) discovered_free;
  decltype(&wuji_connect) connect;
  decltype(&wuji_dev_disconnect) disconnect;
  decltype(&wuji_dev_release) release;
  decltype(&wuji_dev_serial_number) device_serial_number;
  decltype(&wuji_dev_model) device_model;
  decltype(&wuji_dev_is_connected) device_is_connected;
  decltype(&wuji_glove_get_hand_side) get_hand_side;
  decltype(&wuji_sub_close) sub_close;
  decltype(&wuji_glove_subscribe_hand_joint_angles) subscribe_joint_angles;
  decltype(&wuji_glove_subscribe_emf_poses) subscribe_emf_poses;
  decltype(&wuji_glove_subscribe_tip_poses) subscribe_tip_poses;
  decltype(&wuji_glove_subscribe_hand_skeleton) subscribe_hand_skeleton;
  decltype(&wuji_glove_subscribe_imu_palm) subscribe_imu_palm;
  decltype(&wuji_glove_subscribe_tactile) subscribe_tactile;
  decltype(&wuji_glove_subscribe_tactile_residual) subscribe_tactile_residual;
  decltype(&wuji_glove_subscribe_tactile_binary) subscribe_tactile_binary;
  decltype(&wuji_glove_subscribe_tactile_zones) subscribe_tactile_zones;
  decltype(&wuji_glove_subscribe_tactile_point_cloud) subscribe_tactile_point_cloud;

  static GloveSdkApi production() noexcept;
};

}  // namespace wuji_glove_driver::sdk

#endif  // WUJI_GLOVE_DRIVER__SDK__GLOVE_SDK_API_HPP_
