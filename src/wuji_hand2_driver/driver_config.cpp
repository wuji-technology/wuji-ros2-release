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

#include "wuji_hand2_driver/driver_config.hpp"

#include <cmath>

#include <utility>
namespace wuji_hand2_driver {

void declare_driver_parameters(rclcpp::Node& node) {
  node.declare_parameter<std::string>("endpoint", "");
  node.declare_parameter<std::string>("serial_number", "");
  node.declare_parameter<std::string>("hand_side", "");
  node.declare_parameter<double>("connect_timeout", 10.0);
  node.declare_parameter<bool>("auto_enable", false);
  node.declare_parameter<bool>("publish_joint_diagnostics", true);
  node.declare_parameter<bool>("publish_imu", true);
  node.declare_parameter<bool>("publish_fingertip_tactile", false);
}

sdk::Status read_driver_config(const rclcpp::Node& node, DriverConfig& output) {
  DriverConfig config;
  config.endpoint = node.get_parameter("endpoint").as_string();
  config.serial_number = node.get_parameter("serial_number").as_string();
  config.hand_side = node.get_parameter("hand_side").as_string();
  config.connect_timeout = node.get_parameter("connect_timeout").as_double();
  config.auto_enable = node.get_parameter("auto_enable").as_bool();
  config.publish_joint_diagnostics = node.get_parameter("publish_joint_diagnostics").as_bool();
  config.publish_imu = node.get_parameter("publish_imu").as_bool();
  config.publish_fingertip_tactile = node.get_parameter("publish_fingertip_tactile").as_bool();

  if (!config.hand_side.empty() && config.hand_side != "left" && config.hand_side != "right") {
    return sdk::Status::failure("hand_side must be left, right, or empty");
  }
  if (!std::isfinite(config.connect_timeout) || config.connect_timeout < 0.0) {
    return sdk::Status::failure("connect_timeout must be finite and nonnegative");
  }

  output = std::move(config);
  return sdk::Status::success();
}

}  // namespace wuji_hand2_driver
