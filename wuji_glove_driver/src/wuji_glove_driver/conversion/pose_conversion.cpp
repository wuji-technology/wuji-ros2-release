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

#include "wuji_glove_driver/conversion/pose_conversion.hpp"

#include <algorithm>
#include <array>
#include <iterator>
#include <utility>

#include "wuji_glove_driver/conversion/ros_header_conversion.hpp"

namespace wuji_glove_driver::conversion {
namespace {

constexpr std::array<const char*, 5> kFingerNames{"thumb", "index", "middle", "ring", "pinky"};

void copy_pose(const WujiPose& input, geometry_msgs::msg::Pose& output) {
  output.position.x = input.position[0];
  output.position.y = input.position[1];
  output.position.z = input.position[2];
  output.orientation.x = input.orientation.x;
  output.orientation.y = input.orientation.y;
  output.orientation.z = input.orientation.z;
  output.orientation.w = input.orientation.w;
}

template <typename PoseArray>
Status to_finger_poses(const PoseArray& input, wuji_msgs::msg::TrackedPoseArray& output) {
  if (input.poses_len != kFingerNames.size()) {
    return Status::failure("pose frame must contain exactly five entries");
  }
  if (input.poses == nullptr) {
    return Status::failure("pose array is null");
  }

  wuji_msgs::msg::TrackedPoseArray converted;
  const Status header_status = to_ros_header(input.header, converted.header);
  if (!header_status) {
    return header_status;
  }

  converted.poses.reserve(kFingerNames.size());
  for (std::size_t index = 0; index < kFingerNames.size(); ++index) {
    wuji_msgs::msg::TrackedPose pose;
    pose.name = kFingerNames[index];
    copy_pose(input.poses[index].pose, pose.pose);
    pose.confidence = input.poses[index].confidence;
    converted.poses.push_back(std::move(pose));
  }
  output = std::move(converted);
  return Status::success();
}

}  // namespace

Status to_emf_poses(const WujiEmfPoseArray& input, wuji_msgs::msg::TrackedPoseArray& output) {
  return to_finger_poses(input, output);
}

Status to_tip_poses(const WujiFingertipPoses& input, wuji_msgs::msg::TrackedPoseArray& output) {
  return to_finger_poses(input, output);
}

Status to_hand_skeleton(const WujiHandSkeleton& input, wuji_msgs::msg::TrackedPoseArray& output) {
  if (input.joints_len != 0U && input.joints == nullptr) {
    return Status::failure("skeleton joint array is null");
  }

  wuji_msgs::msg::TrackedPoseArray converted;
  const Status header_status = to_ros_header(input.header, converted.header);
  if (!header_status) {
    return header_status;
  }

  converted.poses.reserve(input.joints_len);
  for (std::size_t index = 0; index < input.joints_len; ++index) {
    const WujiSkeletonJoint& joint = input.joints[index];
    const auto terminator = std::find(std::begin(joint.name), std::end(joint.name), '\0');
    if (terminator == std::end(joint.name)) {
      return Status::failure("SDK skeleton joint name is not NUL-terminated");
    }

    wuji_msgs::msg::TrackedPose pose;
    pose.name.assign(joint.name, terminator);
    copy_pose(joint.pose, pose.pose);
    pose.confidence = joint.confidence;
    converted.poses.push_back(std::move(pose));
  }
  output = std::move(converted);
  return Status::success();
}

}  // namespace wuji_glove_driver::conversion
