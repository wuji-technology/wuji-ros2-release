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

#include "wuji_glove_driver/conversion/joint_angles_conversion.hpp"

#include <array>
#include <utility>

#include "wuji_glove_driver/conversion/ros_header_conversion.hpp"

namespace wuji_glove_driver::conversion {
namespace {

constexpr std::array<const char*, 5> kFingerNames{"thumb", "index", "middle", "ring", "pinky"};
constexpr std::array<std::size_t, 5> kAngleCounts{5U, 4U, 4U, 4U, 4U};

}  // namespace

Status to_hand_joint_angles(const WujiHandJointAngles& input,
                            wuji_msgs::msg::HandJointAngles& output) {
  if (input.fingers_len != kFingerNames.size()) {
    return Status::failure("joint angle frame must contain exactly five fingers");
  }
  if (input.fingers == nullptr) {
    return Status::failure("joint angle finger array is null");
  }

  wuji_msgs::msg::HandJointAngles converted;
  const Status header_status = to_ros_header(input.header, converted.header);
  if (!header_status) {
    return header_status;
  }

  for (std::size_t finger = 0; finger < kFingerNames.size(); ++finger) {
    auto& converted_finger = converted.fingers[finger];
    const WujiFingerJointAngles& input_finger = input.fingers[finger];
    converted_finger.name = kFingerNames[finger];
    converted_finger.angles.assign(input_finger.angles, input_finger.angles + kAngleCounts[finger]);
    converted_finger.confidence = input_finger.confidence;
  }
  output = std::move(converted);
  return Status::success();
}

}  // namespace wuji_glove_driver::conversion
