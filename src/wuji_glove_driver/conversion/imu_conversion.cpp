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
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "wuji_glove_driver/conversion/imu_conversion.hpp"

#include <algorithm>
#include <iterator>
#include <utility>

#include "wuji_glove_driver/conversion/ros_header_conversion.hpp"

namespace wuji_glove_driver::conversion {

Status to_imu(const WujiImuData& input, sensor_msgs::msg::Imu& output) {
  sensor_msgs::msg::Imu converted;
  const Status header_status = to_ros_header(input.header, converted.header);
  if (!header_status) {
    return header_status;
  }
  converted.orientation.x = input.orientation.x;
  converted.orientation.y = input.orientation.y;
  converted.orientation.z = input.orientation.z;
  converted.orientation.w = input.orientation.w;
  std::copy(std::begin(input.orientation_covariance), std::end(input.orientation_covariance),
            converted.orientation_covariance.begin());
  converted.angular_velocity.x = input.angular_velocity.x;
  converted.angular_velocity.y = input.angular_velocity.y;
  converted.angular_velocity.z = input.angular_velocity.z;
  std::copy(std::begin(input.angular_velocity_covariance),
            std::end(input.angular_velocity_covariance),
            converted.angular_velocity_covariance.begin());
  converted.linear_acceleration.x = input.linear_acceleration.x;
  converted.linear_acceleration.y = input.linear_acceleration.y;
  converted.linear_acceleration.z = input.linear_acceleration.z;
  std::copy(std::begin(input.linear_acceleration_covariance),
            std::end(input.linear_acceleration_covariance),
            converted.linear_acceleration_covariance.begin());
  output = std::move(converted);
  return Status::success();
}

}  // namespace wuji_glove_driver::conversion
