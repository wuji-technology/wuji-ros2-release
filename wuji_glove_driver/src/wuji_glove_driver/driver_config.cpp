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

#include "wuji_glove_driver/driver_config.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace wuji_glove_driver {

void declare_driver_parameters(rclcpp::Node& node) {
  node.declare_parameter<std::string>("endpoint", "");
  node.declare_parameter<std::string>("serial_number", "");
  node.declare_parameter<std::string>("hand_side", "");
  node.declare_parameter<double>("connect_timeout", 10.0);
  node.declare_parameter<bool>("publish_emf_poses", true);
  node.declare_parameter<bool>("publish_tip_poses", true);
  node.declare_parameter<bool>("publish_skeleton", true);
  node.declare_parameter<bool>("publish_imu", true);
  node.declare_parameter<bool>("publish_tactile", true);
  node.declare_parameter<bool>("publish_tactile_residual", true);
  node.declare_parameter<bool>("publish_tactile_binary", true);
  node.declare_parameter<bool>("publish_tactile_zones", true);
  node.declare_parameter<bool>("publish_tactile_points", true);
}

sdk::Status read_driver_config(const rclcpp::Node& node, DriverConfig& output) {
  DriverConfig config;
  config.endpoint = node.get_parameter("endpoint").as_string();
  config.serial_number = node.get_parameter("serial_number").as_string();
  config.hand_side = node.get_parameter("hand_side").as_string();
  config.connect_timeout = node.get_parameter("connect_timeout").as_double();
  config.publish_emf_poses = node.get_parameter("publish_emf_poses").as_bool();
  config.publish_tip_poses = node.get_parameter("publish_tip_poses").as_bool();
  config.publish_skeleton = node.get_parameter("publish_skeleton").as_bool();
  config.publish_imu = node.get_parameter("publish_imu").as_bool();
  config.publish_tactile = node.get_parameter("publish_tactile").as_bool();
  config.publish_tactile_residual = node.get_parameter("publish_tactile_residual").as_bool();
  config.publish_tactile_binary = node.get_parameter("publish_tactile_binary").as_bool();
  config.publish_tactile_zones = node.get_parameter("publish_tactile_zones").as_bool();
  config.publish_tactile_points = node.get_parameter("publish_tactile_points").as_bool();

  if (!config.hand_side.empty() && config.hand_side != "left" && config.hand_side != "right") {
    return sdk::Status::failure("hand_side must be left, right, or empty");
  }
  constexpr double kMaxTimeoutSeconds =
      static_cast<double>(std::numeric_limits<std::uint32_t>::max()) / 1000.0;
  if (!std::isfinite(config.connect_timeout) || config.connect_timeout < 0.0 ||
      config.connect_timeout > kMaxTimeoutSeconds) {
    return sdk::Status::failure("connect_timeout is outside the SDK millisecond range");
  }

  output = std::move(config);
  return sdk::Status::success();
}

}  // namespace wuji_glove_driver
