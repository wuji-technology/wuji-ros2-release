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

#ifndef WUJI_GLOVE_DRIVER__DRIVER_CONFIG_HPP_
#define WUJI_GLOVE_DRIVER__DRIVER_CONFIG_HPP_

#include <string>

#include <rclcpp/node.hpp>

#include "wuji_glove_driver/sdk/sdk_status.hpp"

namespace wuji_glove_driver {

struct DriverConfig {
  std::string endpoint;
  std::string serial_number;
  std::string hand_side;
  double connect_timeout{10.0};
  bool publish_emf_poses{true};
  bool publish_tip_poses{true};
  bool publish_skeleton{true};
  bool publish_imu{true};
  bool publish_tactile{true};
  bool publish_tactile_residual{true};
  bool publish_tactile_binary{true};
  bool publish_tactile_zones{true};
  bool publish_tactile_points{true};
};

void declare_driver_parameters(rclcpp::Node& node);
sdk::Status read_driver_config(const rclcpp::Node& node, DriverConfig& output);

}  // namespace wuji_glove_driver

#endif  // WUJI_GLOVE_DRIVER__DRIVER_CONFIG_HPP_
