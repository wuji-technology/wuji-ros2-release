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

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "wuji_glove_driver/conversion/joint_angles_conversion.hpp"
#include "wuji_glove_driver/conversion/imu_conversion.hpp"
#include "wuji_glove_driver/conversion/point_cloud_conversion.hpp"
#include "wuji_glove_driver/conversion/pose_conversion.hpp"
#include "wuji_glove_driver/conversion/ros_header_conversion.hpp"
#include "wuji_glove_driver/conversion/tactile_conversion.hpp"

namespace wuji_glove_driver::conversion {
namespace {

constexpr std::array<const char*, 5> kFingerNames{"thumb", "index", "middle", "ring", "pinky"};
constexpr std::array<std::size_t, 5> kAngleCounts{5U, 4U, 4U, 4U, 4U};

void set_header(WujiFrameHeader& header, std::uint64_t timestamp_us, const char* frame_id) {
  header = {};
  header.timestamp_us = timestamp_us;
  std::strncpy(header.frame_id, frame_id, sizeof(header.frame_id) - 1);
}

wuji_msgs::msg::TrackedPoseArray pose_sentinel() {
  wuji_msgs::msg::TrackedPoseArray output;
  output.header.frame_id = "sentinel";
  output.poses.resize(1);
  output.poses[0].name = "unchanged";
  output.poses[0].confidence = -1.0F;
  return output;
}

wuji_msgs::msg::HandJointAngles angles_sentinel() {
  wuji_msgs::msg::HandJointAngles output;
  output.header.frame_id = "sentinel";
  output.fingers[0].name = "unchanged";
  output.fingers[0].angles = {-1.0};
  output.fingers[0].confidence = -1.0;
  return output;
}

void expect_pose(const WujiPose& input, const geometry_msgs::msg::Pose& output) {
  EXPECT_EQ(output.position.x, input.position[0]);
  EXPECT_EQ(output.position.y, input.position[1]);
  EXPECT_EQ(output.position.z, input.position[2]);
  EXPECT_EQ(output.orientation.x, input.orientation.x);
  EXPECT_EQ(output.orientation.y, input.orientation.y);
  EXPECT_EQ(output.orientation.z, input.orientation.z);
  EXPECT_EQ(output.orientation.w, input.orientation.w);
}

TEST(RosHeaderConversion, CopiesTimestampAndFrameIdAndKeepsZeroTimestamp) {
  WujiFrameHeader input{};
  set_header(input, 1234567U, "glove_frame");
  std_msgs::msg::Header output;

  ASSERT_TRUE(to_ros_header(input, output));
  EXPECT_EQ(output.stamp.sec, 1);
  EXPECT_EQ(output.stamp.nanosec, 234567000U);
  EXPECT_EQ(output.frame_id, "glove_frame");

  set_header(input, 0U, "zero_time");
  ASSERT_TRUE(to_ros_header(input, output));
  EXPECT_EQ(output.stamp.sec, 0);
  EXPECT_EQ(output.stamp.nanosec, 0U);
  EXPECT_EQ(output.frame_id, "zero_time");
}

TEST(RosHeaderConversion, RejectsUnrepresentableTimestampWithoutChangingOutput) {
  WujiFrameHeader input{};
  constexpr std::uint64_t kMicrosecondsPerSecond = 1000000U;
  set_header(input,
             (static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()) + 1U) *
                 kMicrosecondsPerSecond,
             "too_late");
  std_msgs::msg::Header output;
  output.stamp.sec = 42;
  output.stamp.nanosec = 9U;
  output.frame_id = "sentinel";
  const auto sentinel = output;

  EXPECT_FALSE(to_ros_header(input, output));
  EXPECT_EQ(output, sentinel);
}

TEST(PoseConversion, EmfCopiesFiveNamedEntriesAndRejectsStructuralErrorsAtomically) {
  std::array<WujiEmfPose, 5> poses{};
  for (std::size_t index = 0; index < poses.size(); ++index) {
    poses[index].pose.position[0] = static_cast<float>(index + 1U);
    poses[index].pose.position[1] = static_cast<float>(index + 11U);
    poses[index].pose.position[2] = static_cast<float>(index + 21U);
    poses[index].pose.orientation = {
        static_cast<float>(index + 31U), static_cast<float>(index + 41U),
        static_cast<float>(index + 51U), static_cast<float>(index + 61U)};
    poses[index].confidence = static_cast<float>(index) / 10.0F;
  }
  WujiEmfPoseArray input{};
  set_header(input.header, 2000001U, "emf_frame");
  input.poses = poses.data();
  input.poses_len = poses.size();
  wuji_msgs::msg::TrackedPoseArray output;

  ASSERT_TRUE(to_emf_poses(input, output));
  ASSERT_EQ(output.poses.size(), poses.size());
  EXPECT_EQ(output.header.stamp.sec, 2);
  EXPECT_EQ(output.header.stamp.nanosec, 1000U);
  EXPECT_EQ(output.header.frame_id, "emf_frame");
  for (std::size_t index = 0; index < poses.size(); ++index) {
    EXPECT_EQ(output.poses[index].name, kFingerNames[index]);
    expect_pose(poses[index].pose, output.poses[index].pose);
    EXPECT_EQ(output.poses[index].confidence, poses[index].confidence);
  }

  const auto sentinel = pose_sentinel();
  output = sentinel;
  input.poses_len = 4U;
  EXPECT_FALSE(to_emf_poses(input, output));
  EXPECT_EQ(output, sentinel);

  input.poses_len = poses.size();
  input.poses = nullptr;
  EXPECT_FALSE(to_emf_poses(input, output));
  EXPECT_EQ(output, sentinel);
}

TEST(PoseConversion, TipCopiesFiveNamedEntriesAndRejectsStructuralErrorsAtomically) {
  std::array<WujiFingertipPose, 5> poses{};
  for (std::size_t index = 0; index < poses.size(); ++index) {
    poses[index].pose.position[0] = static_cast<float>(index + 101U);
    poses[index].pose.position[1] = static_cast<float>(index + 111U);
    poses[index].pose.position[2] = static_cast<float>(index + 121U);
    poses[index].pose.orientation = {
        static_cast<float>(index + 131U), static_cast<float>(index + 141U),
        static_cast<float>(index + 151U), static_cast<float>(index + 161U)};
    poses[index].confidence = static_cast<float>(index + 1U) / 5.0F;
  }
  WujiFingertipPoses input{};
  set_header(input.header, 3000002U, "tip_frame");
  input.poses = poses.data();
  input.poses_len = poses.size();
  wuji_msgs::msg::TrackedPoseArray output;

  ASSERT_TRUE(to_tip_poses(input, output));
  ASSERT_EQ(output.poses.size(), poses.size());
  EXPECT_EQ(output.header.stamp.sec, 3);
  EXPECT_EQ(output.header.stamp.nanosec, 2000U);
  EXPECT_EQ(output.header.frame_id, "tip_frame");
  for (std::size_t index = 0; index < poses.size(); ++index) {
    EXPECT_EQ(output.poses[index].name, kFingerNames[index]);
    expect_pose(poses[index].pose, output.poses[index].pose);
    EXPECT_EQ(output.poses[index].confidence, poses[index].confidence);
  }

  const auto sentinel = pose_sentinel();
  output = sentinel;
  input.poses_len = 0U;
  input.poses = nullptr;
  EXPECT_FALSE(to_tip_poses(input, output));
  EXPECT_EQ(output, sentinel);

  input.poses_len = poses.size();
  EXPECT_FALSE(to_tip_poses(input, output));
  EXPECT_EQ(output, sentinel);
}

class SkeletonConversion : public ::testing::TestWithParam<std::size_t> {};

TEST_P(SkeletonConversion, CopiesAnyLengthAndSdkNames) {
  const std::size_t length = GetParam();
  std::vector<WujiSkeletonJoint> joints(length);
  for (std::size_t index = 0; index < joints.size(); ++index) {
    const std::string name = "sdk_joint_" + std::to_string(index);
    std::strncpy(joints[index].name, name.c_str(), sizeof(joints[index].name) - 1);
    joints[index].pose.position[0] = static_cast<float>(index + 1U);
    joints[index].pose.position[1] = static_cast<float>(index + 2U);
    joints[index].pose.position[2] = static_cast<float>(index + 3U);
    joints[index].pose.orientation = {
        static_cast<float>(index + 4U), static_cast<float>(index + 5U),
        static_cast<float>(index + 6U), static_cast<float>(index + 7U)};
    joints[index].confidence = static_cast<float>(index) / 100.0F;
  }
  WujiHandSkeleton input{};
  set_header(input.header, 4000003U, "skeleton_frame");
  input.joints = joints.empty() ? nullptr : joints.data();
  input.joints_len = joints.size();
  wuji_msgs::msg::TrackedPoseArray output;

  ASSERT_TRUE(to_hand_skeleton(input, output));
  ASSERT_EQ(output.poses.size(), joints.size());
  EXPECT_EQ(output.header.stamp.sec, 4);
  EXPECT_EQ(output.header.stamp.nanosec, 3000U);
  EXPECT_EQ(output.header.frame_id, "skeleton_frame");
  for (std::size_t index = 0; index < joints.size(); ++index) {
    EXPECT_EQ(output.poses[index].name, joints[index].name);
    expect_pose(joints[index].pose, output.poses[index].pose);
    EXPECT_EQ(output.poses[index].confidence, joints[index].confidence);
  }
}

INSTANTIATE_TEST_SUITE_P(Lengths, SkeletonConversion, ::testing::Values(0U, 1U, 21U, 37U));

TEST(PoseConversion, SkeletonRejectsNullNonemptyArrayWithoutChangingOutput) {
  WujiHandSkeleton input{};
  set_header(input.header, 1U, "skeleton_frame");
  input.joints = nullptr;
  input.joints_len = 1U;
  auto output = pose_sentinel();
  const auto sentinel = output;

  EXPECT_FALSE(to_hand_skeleton(input, output));
  EXPECT_EQ(output, sentinel);
}

TEST(JointAnglesConversion, CopiesDefinedFingerAnglesAndRejectsStructuralErrorsAtomically) {
  std::array<WujiFingerJointAngles, 5> fingers{};
  for (std::size_t finger = 0; finger < fingers.size(); ++finger) {
    for (std::size_t angle = 0; angle < 5U; ++angle) {
      fingers[finger].angles[angle] = static_cast<double>(finger * 10U + angle);
    }
    fingers[finger].confidence = static_cast<double>(finger) / 10.0;
  }
  WujiHandJointAngles input{};
  set_header(input.header, 5000004U, "angles_frame");
  input.fingers = fingers.data();
  input.fingers_len = fingers.size();
  wuji_msgs::msg::HandJointAngles output;

  ASSERT_TRUE(to_hand_joint_angles(input, output));
  EXPECT_EQ(output.header.stamp.sec, 5);
  EXPECT_EQ(output.header.stamp.nanosec, 4000U);
  EXPECT_EQ(output.header.frame_id, "angles_frame");
  for (std::size_t finger = 0; finger < fingers.size(); ++finger) {
    EXPECT_EQ(output.fingers[finger].name, kFingerNames[finger]);
    ASSERT_EQ(output.fingers[finger].angles.size(), kAngleCounts[finger]);
    for (std::size_t angle = 0; angle < kAngleCounts[finger]; ++angle) {
      EXPECT_EQ(output.fingers[finger].angles[angle], fingers[finger].angles[angle]);
    }
    EXPECT_EQ(output.fingers[finger].confidence, fingers[finger].confidence);
  }

  const auto sentinel = angles_sentinel();
  output = sentinel;
  input.fingers_len = 4U;
  EXPECT_FALSE(to_hand_joint_angles(input, output));
  EXPECT_EQ(output, sentinel);

  input.fingers_len = fingers.size();
  input.fingers = nullptr;
  EXPECT_FALSE(to_hand_joint_angles(input, output));
  EXPECT_EQ(output, sentinel);
}

TEST(ImuConversion, CopiesEveryChannelAndCovariance) {
  WujiImuData input{};
  set_header(input.header, 6000005U, "imu_frame");
  input.orientation = {1.0, 2.0, 3.0, 4.0};
  input.angular_velocity = {5.0, 6.0, 7.0};
  input.linear_acceleration = {8.0, 9.0, 10.0};
  for (std::size_t index = 0; index < 9U; ++index) {
    input.orientation_covariance[index] = index == 0U ? -1.0 : static_cast<double>(index);
    input.angular_velocity_covariance[index] = static_cast<double>(index + 10U);
    input.linear_acceleration_covariance[index] = static_cast<double>(index + 20U);
  }
  sensor_msgs::msg::Imu output;

  ASSERT_TRUE(to_imu(input, output));
  EXPECT_EQ(output.header.frame_id, "imu_frame");
  EXPECT_EQ(output.orientation.x, 1.0);
  EXPECT_EQ(output.orientation.y, 2.0);
  EXPECT_EQ(output.orientation.z, 3.0);
  EXPECT_EQ(output.orientation.w, 4.0);
  EXPECT_EQ(output.angular_velocity.x, 5.0);
  EXPECT_EQ(output.angular_velocity.y, 6.0);
  EXPECT_EQ(output.angular_velocity.z, 7.0);
  EXPECT_EQ(output.linear_acceleration.x, 8.0);
  EXPECT_EQ(output.linear_acceleration.y, 9.0);
  EXPECT_EQ(output.linear_acceleration.z, 10.0);
  for (std::size_t index = 0; index < 9U; ++index) {
    EXPECT_EQ(output.orientation_covariance[index], input.orientation_covariance[index]);
    EXPECT_EQ(output.angular_velocity_covariance[index], input.angular_velocity_covariance[index]);
    EXPECT_EQ(output.linear_acceleration_covariance[index],
              input.linear_acceleration_covariance[index]);
  }
}

template <typename TactileFrame>
void expect_tactile_grid(std::size_t length) {
  std::vector<float> values(length);
  for (std::size_t index = 0; index < values.size(); ++index) {
    values[index] = static_cast<float>(index) - 2.5F;
  }
  TactileFrame input{};
  set_header(input.header, 7000006U, "tactile_frame");
  input.data = values.empty() ? nullptr : values.data();
  input.data_len = values.size();
  wuji_msgs::msg::TactileGrid output;

  ASSERT_TRUE(to_tactile_grid(input, output));
  EXPECT_EQ(output.header.frame_id, "tactile_frame");
  EXPECT_EQ(output.rows, 24U);
  EXPECT_EQ(output.columns, 31U);
  EXPECT_EQ(output.data, values);
}

TEST(TactileConversion, CopiesExpectedGridLengthForAllThreeKinds) {
  expect_tactile_grid<WujiTactileFrame>(744U);
  expect_tactile_grid<WujiTactileResidual>(744U);
  expect_tactile_grid<WujiTactileBinary>(744U);
}

template <typename TactileFrame>
void expect_tactile_grid_length_rejected(std::size_t length) {
  std::vector<float> values(length);
  TactileFrame input{};
  set_header(input.header, 7000006U, "tactile_frame");
  input.data = values.empty() ? nullptr : values.data();
  input.data_len = values.size();
  wuji_msgs::msg::TactileGrid output;
  output.header.frame_id = "sentinel";
  output.rows = 9U;
  output.columns = 8U;
  output.data = {-1.0F};
  const auto sentinel = output;

  EXPECT_FALSE(to_tactile_grid(input, output));
  EXPECT_EQ(output, sentinel);
}

TEST(TactileConversion, RejectsUnexpectedGridLengthsForAllThreeKinds) {
  for (const std::size_t length : {0U, 3U, 800U}) {
    expect_tactile_grid_length_rejected<WujiTactileFrame>(length);
    expect_tactile_grid_length_rejected<WujiTactileResidual>(length);
    expect_tactile_grid_length_rejected<WujiTactileBinary>(length);
  }
}

TEST(TactileConversion, RejectsNullNonemptyGridWithoutChangingOutput) {
  WujiTactileFrame input{};
  set_header(input.header, 1U, "tactile_frame");
  input.data_len = 744U;
  wuji_msgs::msg::TactileGrid output;
  output.header.frame_id = "sentinel";
  output.rows = 9U;
  output.data = {-1.0F};
  const auto sentinel = output;

  EXPECT_FALSE(to_tactile_grid(input, output));
  EXPECT_EQ(output, sentinel);
}

TEST(TactileConversion, CopiesSixNamedZonesWithIndependentLengths) {
  std::vector<float> palm{1.0F};
  std::vector<float> thumb{2.0F, 3.0F};
  std::vector<float> index{4.0F, 5.0F, 6.0F};
  std::vector<float> middle{7.0F, 8.0F, 9.0F, 10.0F};
  std::vector<float> ring{11.0F, 12.0F, 13.0F, 14.0F, 15.0F};
  std::vector<float> pinky;
  WujiTactileZones input{};
  set_header(input.header, 8000007U, "zones_frame");
  input.palm = palm.data();
  input.palm_len = palm.size();
  input.thumb = thumb.data();
  input.thumb_len = thumb.size();
  input.index = index.data();
  input.index_len = index.size();
  input.middle = middle.data();
  input.middle_len = middle.size();
  input.ring = ring.data();
  input.ring_len = ring.size();
  input.pinky = nullptr;
  input.pinky_len = 0U;
  wuji_msgs::msg::TactileZoneArray output;

  ASSERT_TRUE(to_tactile_zones(input, output));
  ASSERT_EQ(output.zones.size(), 6U);
  const std::array<const char*, 6> names{"palm", "thumb", "index", "middle", "ring", "pinky"};
  const std::array<std::vector<float>, 6> expected{palm, thumb, index, middle, ring, pinky};
  for (std::size_t zone = 0; zone < names.size(); ++zone) {
    EXPECT_EQ(output.zones[zone].name, names[zone]);
    EXPECT_EQ(output.zones[zone].values, expected[zone]);
  }
}

TEST(TactileConversion, RejectsNullNonemptyZoneWithoutChangingOutput) {
  WujiTactileZones input{};
  set_header(input.header, 1U, "zones_frame");
  input.middle_len = 1U;
  wuji_msgs::msg::TactileZoneArray output;
  output.header.frame_id = "sentinel";
  output.zones.resize(1U);
  output.zones[0].name = "unchanged";
  const auto sentinel = output;

  EXPECT_FALSE(to_tactile_zones(input, output));
  EXPECT_EQ(output, sentinel);
}

TEST(PointCloudConversion, CopiesSelfDescribingLayoutsAndMapsEveryDatatype) {
  std::array<WujiPointField, 8> fields{};
  const std::array<const char*, 8> names{"u8", "i8", "u16", "i16", "u32", "i32", "f32", "f64"};
  const std::array<std::uint8_t, 8> expected_types{
      sensor_msgs::msg::PointField::UINT8,   sensor_msgs::msg::PointField::INT8,
      sensor_msgs::msg::PointField::UINT16,  sensor_msgs::msg::PointField::INT16,
      sensor_msgs::msg::PointField::UINT32,  sensor_msgs::msg::PointField::INT32,
      sensor_msgs::msg::PointField::FLOAT32, sensor_msgs::msg::PointField::FLOAT64};
  for (std::size_t index = 0; index < fields.size(); ++index) {
    std::strncpy(fields[index].name, names[index], sizeof(fields[index].name) - 1U);
    fields[index].offset = static_cast<std::uint32_t>(index * 2U);
    fields[index].type_ = static_cast<std::uint8_t>(index + 1U);
  }
  std::vector<std::uint8_t> bytes(64U);
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    bytes[index] = static_cast<std::uint8_t>(index);
  }
  WujiPointCloud input{};
  set_header(input.header, 9000008U, "header_frame");
  std::strncpy(input.frame_id, "cloud_frame", sizeof(input.frame_id) - 1U);
  input.point_stride = 16U;
  input.fields = fields.data();
  input.fields_len = fields.size();
  input.data = bytes.data();
  input.data_len = bytes.size();
  sensor_msgs::msg::PointCloud2 output;

  ASSERT_TRUE(to_point_cloud2(input, output));
  EXPECT_EQ(output.header.frame_id, "cloud_frame");
  EXPECT_EQ(output.height, 1U);
  EXPECT_EQ(output.width, 4U);
  EXPECT_FALSE(output.is_bigendian);
  EXPECT_EQ(output.point_step, 16U);
  EXPECT_EQ(output.row_step, 64U);
  EXPECT_FALSE(output.is_dense);
  EXPECT_EQ(output.data, bytes);
  ASSERT_EQ(output.fields.size(), fields.size());
  for (std::size_t index = 0; index < fields.size(); ++index) {
    EXPECT_EQ(output.fields[index].name, names[index]);
    EXPECT_EQ(output.fields[index].offset, fields[index].offset);
    EXPECT_EQ(output.fields[index].datatype, expected_types[index]);
    EXPECT_EQ(output.fields[index].count, 1U);
  }

  std::array<WujiPointField, 1> alternate_field{};
  std::strncpy(alternate_field[0].name, "pressure", sizeof(alternate_field[0].name) - 1U);
  alternate_field[0].offset = 3U;
  alternate_field[0].type_ = 7U;
  std::vector<std::uint8_t> alternate_bytes(15U, 42U);
  input.point_stride = 5U;
  input.fields = alternate_field.data();
  input.fields_len = alternate_field.size();
  input.data = alternate_bytes.data();
  input.data_len = alternate_bytes.size();
  ASSERT_TRUE(to_point_cloud2(input, output));
  EXPECT_EQ(output.width, 3U);
  EXPECT_EQ(output.point_step, 5U);
  EXPECT_EQ(output.fields.size(), 1U);
  EXPECT_EQ(output.data, alternate_bytes);
}

TEST(PointCloudConversion, RejectsMalformedMetadataWithoutChangingOutput) {
  std::array<WujiPointField, 1> fields{};
  std::strncpy(fields[0].name, "x", sizeof(fields[0].name) - 1U);
  fields[0].type_ = 7U;
  std::array<std::uint8_t, 4> bytes{};
  WujiPointCloud input{};
  set_header(input.header, 1U, "header_frame");
  std::strncpy(input.frame_id, "cloud_frame", sizeof(input.frame_id) - 1U);
  input.point_stride = 4U;
  input.fields = fields.data();
  input.fields_len = fields.size();
  input.data = bytes.data();
  input.data_len = bytes.size();
  sensor_msgs::msg::PointCloud2 output;
  output.header.frame_id = "sentinel";
  output.width = 99U;
  const auto sentinel = output;

  fields[0].type_ = 9U;
  EXPECT_FALSE(to_point_cloud2(input, output));
  EXPECT_EQ(output, sentinel);
  fields[0].type_ = 7U;

  input.point_stride = 0U;
  EXPECT_FALSE(to_point_cloud2(input, output));
  input.point_stride = 3U;
  EXPECT_FALSE(to_point_cloud2(input, output));
  input.point_stride = 4U;

  input.fields = nullptr;
  EXPECT_FALSE(to_point_cloud2(input, output));
  input.fields = fields.data();
  input.data = nullptr;
  EXPECT_FALSE(to_point_cloud2(input, output));
  input.data = bytes.data();

  input.data_len = static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()) + 1U;
  input.point_stride = 1U;
  EXPECT_FALSE(to_point_cloud2(input, output));
  EXPECT_EQ(output, sentinel);
}

}  // namespace
}  // namespace wuji_glove_driver::conversion
