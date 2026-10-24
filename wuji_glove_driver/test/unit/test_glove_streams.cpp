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
#include <wuji_sdk.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <wuji_msgs/msg/hand_joint_angles.hpp>
#include <wuji_msgs/msg/tactile_grid.hpp>
#include <wuji_msgs/msg/tactile_zone_array.hpp>
#include <wuji_msgs/msg/tracked_pose_array.hpp>

#include "wuji_driver_common/stream/stream_controller.hpp"
#include "wuji_driver_common/stream/stream_types.hpp"
#include "wuji_glove_driver/sdk/glove_sdk_api.hpp"
#include "wuji_glove_driver/stream/emf_poses_stream.hpp"
#include "wuji_glove_driver/stream/joint_angles_stream.hpp"
#include "wuji_glove_driver/stream/imu_stream.hpp"
#include "wuji_glove_driver/stream/skeleton_stream.hpp"
#include "wuji_glove_driver/stream/tactile_binary_stream.hpp"
#include "wuji_glove_driver/stream/tactile_point_cloud_stream.hpp"
#include "wuji_glove_driver/stream/tactile_residual_stream.hpp"
#include "wuji_glove_driver/stream/tactile_stream.hpp"
#include "wuji_glove_driver/stream/tactile_zones_stream.hpp"
#include "wuji_glove_driver/stream/tip_poses_stream.hpp"

namespace wuji_glove_driver::stream {
namespace {

using namespace std::chrono_literals;
using wuji_driver_common::stream::StreamController;
using wuji_driver_common::stream::StreamKey;
using wuji_driver_common::stream::StreamPolicy;
using wuji_driver_common::stream::StreamState;
using wuji_driver_common::stream::TerminalKind;

WujiDevice* fake_device() {
  return reinterpret_cast<WujiDevice*>(std::uintptr_t{1});
}

void set_header(WujiFrameHeader& header, std::uint64_t timestamp_us, const char* frame_id) {
  header = {};
  header.timestamp_us = timestamp_us;
  std::strncpy(header.frame_id, frame_id, sizeof(header.frame_id) - 1U);
}

class StreamSdkHarness {
 public:
  StreamSdkHarness() {
    EXPECT_EQ(current_, nullptr);
    current_ = this;
    api_.sub_close = &sub_close;
    api_.subscribe_joint_angles = &subscribe_joint_angles;
    api_.subscribe_emf_poses = &subscribe_emf_poses;
    api_.subscribe_tip_poses = &subscribe_tip_poses;
    api_.subscribe_hand_skeleton = &subscribe_hand_skeleton;
    api_.subscribe_imu_palm = &subscribe_imu_palm;
    api_.subscribe_tactile = &subscribe_tactile;
    api_.subscribe_tactile_residual = &subscribe_tactile_residual;
    api_.subscribe_tactile_binary = &subscribe_tactile_binary;
    api_.subscribe_tactile_zones = &subscribe_tactile_zones;
    api_.subscribe_tactile_point_cloud = &subscribe_tactile_point_cloud;
  }
  ~StreamSdkHarness() {
    current_ = nullptr;
  }
  sdk::GloveSdkApi& api() noexcept {
    return api_;
  }
  std::size_t subscribe_count(std::size_t index) const noexcept {
    return subscribe_counts_[index];
  }
  std::size_t close_count(std::size_t index) const noexcept {
    return close_counts_[index];
  }
  std::size_t total_close_count() const noexcept {
    std::size_t result = 0U;
    for (const auto count : close_counts_) {
      result += count;
    }
    return result;
  }
  void emit_joint_angles(WujiFrameKind kind, const WujiHandJointAngles* frame) {
    joint_angles_callback_(kind, frame, joint_angles_user_data_);
  }
  void emit_emf_poses(WujiFrameKind kind, const WujiEmfPoseArray* frame) {
    emf_poses_callback_(kind, frame, emf_poses_user_data_);
  }
  void emit_tip_poses(WujiFrameKind kind, const WujiFingertipPoses* frame) {
    tip_poses_callback_(kind, frame, tip_poses_user_data_);
  }
  void emit_hand_skeleton(WujiFrameKind kind, const WujiHandSkeleton* frame) {
    hand_skeleton_callback_(kind, frame, hand_skeleton_user_data_);
  }
  void emit_imu(WujiFrameKind kind, const WujiImuData* frame) {
    imu_callback_(kind, frame, imu_user_data_);
  }
  void emit_tactile(WujiFrameKind kind, const WujiTactileFrame* frame) {
    tactile_callback_(kind, frame, tactile_user_data_);
  }
  void emit_tactile_residual(WujiFrameKind kind, const WujiTactileResidual* frame) {
    tactile_residual_callback_(kind, frame, tactile_residual_user_data_);
  }
  void emit_tactile_binary(WujiFrameKind kind, const WujiTactileBinary* frame) {
    tactile_binary_callback_(kind, frame, tactile_binary_user_data_);
  }
  void emit_tactile_zones(WujiFrameKind kind, const WujiTactileZones* frame) {
    tactile_zones_callback_(kind, frame, tactile_zones_user_data_);
  }
  void emit_tactile_point_cloud(WujiFrameKind kind, const WujiPointCloud* frame) {
    tactile_point_cloud_callback_(kind, frame, tactile_point_cloud_user_data_);
  }

 private:
  static StreamSdkHarness& current() {
    return *current_;
  }
  static WujiSub* subscription(std::size_t index) {
    return reinterpret_cast<WujiSub*>(static_cast<std::uintptr_t>(index + 1U));
  }
  static void sub_close(WujiSub* handle) {
    const auto token = reinterpret_cast<std::uintptr_t>(handle);
    if (token >= 1U && token <= 10U) {
      ++current().close_counts_[token - 1U];
    }
  }
  static WujiStatus subscribe_joint_angles(WujiDevice*, WujiHandJointAnglesSubCallback callback,
                                           void* data, WujiSub** output) {
    auto& self = current();
    ++self.subscribe_counts_[0];
    self.joint_angles_callback_ = callback;
    self.joint_angles_user_data_ = data;
    *output = subscription(0U);
    return WUJI_STATUS_OK;
  }
  static WujiStatus subscribe_emf_poses(WujiDevice*, WujiEmfPoseArraySubCallback callback,
                                        void* data, WujiSub** output) {
    auto& self = current();
    ++self.subscribe_counts_[1];
    self.emf_poses_callback_ = callback;
    self.emf_poses_user_data_ = data;
    *output = subscription(1U);
    return WUJI_STATUS_OK;
  }
  static WujiStatus subscribe_tip_poses(WujiDevice*, WujiFingertipPosesSubCallback callback,
                                        void* data, WujiSub** output) {
    auto& self = current();
    ++self.subscribe_counts_[2];
    self.tip_poses_callback_ = callback;
    self.tip_poses_user_data_ = data;
    *output = subscription(2U);
    return WUJI_STATUS_OK;
  }
  static WujiStatus subscribe_hand_skeleton(WujiDevice*, WujiHandSkeletonSubCallback callback,
                                            void* data, WujiSub** output) {
    auto& self = current();
    ++self.subscribe_counts_[3];
    self.hand_skeleton_callback_ = callback;
    self.hand_skeleton_user_data_ = data;
    *output = subscription(3U);
    return WUJI_STATUS_OK;
  }

  static WujiStatus subscribe_imu_palm(WujiDevice*, WujiImuDataSubCallback callback, void* data,
                                       WujiSub** output) {
    auto& self = current();
    ++self.subscribe_counts_[4U];
    self.imu_callback_ = callback;
    self.imu_user_data_ = data;
    *output = subscription(4U);
    return WUJI_STATUS_OK;
  }

#define WUJI_TEST_TACTILE_SUBSCRIBE(name, index, callback_type, callback_member, data_member) \
  static WujiStatus subscribe_##name(WujiDevice*, callback_type callback, void* data,         \
                                     WujiSub** output) {                                      \
    auto& self = current();                                                                   \
    ++self.subscribe_counts_[index];                                                          \
    self.callback_member = callback;                                                          \
    self.data_member = data;                                                                  \
    *output = subscription(index);                                                            \
    return WUJI_STATUS_OK;                                                                    \
  }
  WUJI_TEST_TACTILE_SUBSCRIBE(tactile, 5U, WujiTactileFrameSubCallback, tactile_callback_,
                              tactile_user_data_)
  WUJI_TEST_TACTILE_SUBSCRIBE(tactile_residual, 6U, WujiTactileResidualSubCallback,
                              tactile_residual_callback_, tactile_residual_user_data_)
  WUJI_TEST_TACTILE_SUBSCRIBE(tactile_binary, 7U, WujiTactileBinarySubCallback,
                              tactile_binary_callback_, tactile_binary_user_data_)
  WUJI_TEST_TACTILE_SUBSCRIBE(tactile_zones, 8U, WujiTactileZonesSubCallback,
                              tactile_zones_callback_, tactile_zones_user_data_)
  WUJI_TEST_TACTILE_SUBSCRIBE(tactile_point_cloud, 9U, WujiPointCloudSubCallback,
                              tactile_point_cloud_callback_, tactile_point_cloud_user_data_)
#undef WUJI_TEST_TACTILE_SUBSCRIBE

  static StreamSdkHarness* current_;
  sdk::GloveSdkApi api_{};
  std::array<std::size_t, 10> subscribe_counts_{};
  std::array<std::size_t, 10> close_counts_{};
  WujiHandJointAnglesSubCallback joint_angles_callback_{nullptr};
  WujiEmfPoseArraySubCallback emf_poses_callback_{nullptr};
  WujiFingertipPosesSubCallback tip_poses_callback_{nullptr};
  WujiHandSkeletonSubCallback hand_skeleton_callback_{nullptr};
  void* joint_angles_user_data_{nullptr};
  void* emf_poses_user_data_{nullptr};
  void* tip_poses_user_data_{nullptr};
  void* hand_skeleton_user_data_{nullptr};
  WujiImuDataSubCallback imu_callback_{nullptr};
  void* imu_user_data_{nullptr};
  WujiTactileFrameSubCallback tactile_callback_{nullptr};
  WujiTactileResidualSubCallback tactile_residual_callback_{nullptr};
  WujiTactileBinarySubCallback tactile_binary_callback_{nullptr};
  WujiTactileZonesSubCallback tactile_zones_callback_{nullptr};
  WujiPointCloudSubCallback tactile_point_cloud_callback_{nullptr};
  void* tactile_user_data_{nullptr};
  void* tactile_residual_user_data_{nullptr};
  void* tactile_binary_user_data_{nullptr};
  void* tactile_zones_user_data_{nullptr};
  void* tactile_point_cloud_user_data_{nullptr};
};

StreamSdkHarness* StreamSdkHarness::current_{nullptr};

template <typename StreamT>
struct StopDeleter {
  void operator()(StreamT* stream) const noexcept {
    if (stream != nullptr) {
      stream->stop();
      delete stream;
    }
  }
};

template <typename StreamT>
using StreamPtr = std::unique_ptr<StreamT, StopDeleter<StreamT>>;

struct StreamRow {
  StreamKey key;
  const char* topic;
  StreamPolicy policy;
  const char* sdk_counter;
  const char* ros_type;
};

constexpr std::array<StreamRow, 10> kRows{{
    {StreamKey{1}, "joint_angles", StreamPolicy::Required, "subscribe_joint_angles",
     "wuji_msgs/msg/HandJointAngles"},
    {StreamKey{2}, "emf_poses", StreamPolicy::Optional, "subscribe_emf_poses",
     "wuji_msgs/msg/TrackedPoseArray"},
    {StreamKey{3}, "tip_poses", StreamPolicy::Optional, "subscribe_tip_poses",
     "wuji_msgs/msg/TrackedPoseArray"},
    {StreamKey{4}, "hand_skeleton", StreamPolicy::Optional, "subscribe_hand_skeleton",
     "wuji_msgs/msg/TrackedPoseArray"},
    {StreamKey{5}, "imu/palm", StreamPolicy::Optional, "subscribe_imu_palm", "sensor_msgs/msg/Imu"},
    {StreamKey{6}, "tactile", StreamPolicy::Optional, "subscribe_tactile",
     "wuji_msgs/msg/TactileGrid"},
    {StreamKey{7}, "tactile/residual", StreamPolicy::Optional, "subscribe_tactile_residual",
     "wuji_msgs/msg/TactileGrid"},
    {StreamKey{8}, "tactile/binary", StreamPolicy::Optional, "subscribe_tactile_binary",
     "wuji_msgs/msg/TactileGrid"},
    {StreamKey{9}, "tactile/zones", StreamPolicy::Optional, "subscribe_tactile_zones",
     "wuji_msgs/msg/TactileZoneArray"},
    {StreamKey{10}, "tactile/points", StreamPolicy::Optional, "subscribe_tactile_point_cloud",
     "sensor_msgs/msg/PointCloud2"},
}};

class GloveStreamsTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }
  void SetUp() override {
    namespace_ = "/glove_stream_test_" +
                 std::to_string(next_namespace_.fetch_add(1U, std::memory_order_relaxed));
    source_ = std::make_shared<rclcpp::Node>("source", namespace_);
    observer_ = std::make_shared<rclcpp::Node>("observer", namespace_);
    executor_.add_node(source_);
    executor_.add_node(observer_);
  }
  void TearDown() override {
    executor_.remove_node(observer_);
    executor_.remove_node(source_);
  }
  StreamPtr<JointAnglesStream> make_joint_angles() {
    return StreamPtr<JointAnglesStream>(
        new JointAnglesStream(kRows[0].key, fake_device(), sdk_.api(), *source_));
  }
  StreamPtr<EmfPosesStream> make_emf_poses() {
    return StreamPtr<EmfPosesStream>(
        new EmfPosesStream(kRows[1].key, fake_device(), sdk_.api(), *source_));
  }
  StreamPtr<TipPosesStream> make_tip_poses() {
    return StreamPtr<TipPosesStream>(
        new TipPosesStream(kRows[2].key, fake_device(), sdk_.api(), *source_));
  }
  StreamPtr<SkeletonStream> make_skeleton() {
    return StreamPtr<SkeletonStream>(
        new SkeletonStream(kRows[3].key, fake_device(), sdk_.api(), *source_));
  }
  StreamPtr<ImuStream> make_imu() {
    return StreamPtr<ImuStream>(new ImuStream(kRows[4].key, fake_device(), sdk_.api(), *source_));
  }
  StreamPtr<TactileStream> make_tactile() {
    return StreamPtr<TactileStream>(
        new TactileStream(kRows[5].key, fake_device(), sdk_.api(), *source_));
  }
  StreamPtr<TactileResidualStream> make_tactile_residual() {
    return StreamPtr<TactileResidualStream>(
        new TactileResidualStream(kRows[6].key, fake_device(), sdk_.api(), *source_));
  }
  StreamPtr<TactileBinaryStream> make_tactile_binary() {
    return StreamPtr<TactileBinaryStream>(
        new TactileBinaryStream(kRows[7].key, fake_device(), sdk_.api(), *source_));
  }
  StreamPtr<TactileZonesStream> make_tactile_zones() {
    return StreamPtr<TactileZonesStream>(
        new TactileZonesStream(kRows[8].key, fake_device(), sdk_.api(), *source_));
  }
  StreamPtr<TactilePointCloudStream> make_tactile_point_cloud() {
    return StreamPtr<TactilePointCloudStream>(
        new TactilePointCloudStream(kRows[9].key, fake_device(), sdk_.api(), *source_));
  }
  void capture_messages() {
    angles_sub_ = observer_->create_subscription<wuji_msgs::msg::HandJointAngles>(
        topic("joint_angles"), rclcpp::SensorDataQoS().keep_last(5),
        [this](wuji_msgs::msg::HandJointAngles::ConstSharedPtr message) {
          angle_messages_.push_back(*message);
        });
    for (std::size_t index = 0; index < pose_subs_.size(); ++index) {
      pose_subs_[index] = observer_->create_subscription<wuji_msgs::msg::TrackedPoseArray>(
          topic(kRows[index + 1U].topic), rclcpp::SensorDataQoS().keep_last(5),
          [this, index](wuji_msgs::msg::TrackedPoseArray::ConstSharedPtr message) {
            pose_messages_[index].push_back(*message);
          });
    }
  }
  void capture_task7_messages() {
    imu_sub_ = observer_->create_subscription<sensor_msgs::msg::Imu>(
        topic(kRows[4].topic), rclcpp::SensorDataQoS().keep_last(5),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          imu_messages_.push_back(*message);
        });
    for (std::size_t index = 0; index < grid_subs_.size(); ++index) {
      grid_subs_[index] = observer_->create_subscription<wuji_msgs::msg::TactileGrid>(
          topic(kRows[index + 5U].topic), rclcpp::SensorDataQoS().keep_last(5),
          [this, index](wuji_msgs::msg::TactileGrid::ConstSharedPtr message) {
            grid_messages_[index].push_back(*message);
          });
    }
    zones_sub_ = observer_->create_subscription<wuji_msgs::msg::TactileZoneArray>(
        topic(kRows[8].topic), rclcpp::SensorDataQoS().keep_last(5),
        [this](wuji_msgs::msg::TactileZoneArray::ConstSharedPtr message) {
          zone_messages_.push_back(*message);
        });
    point_cloud_sub_ = observer_->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic(kRows[9].topic), rclcpp::SensorDataQoS().keep_last(5),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          point_cloud_messages_.push_back(*message);
        });
  }
  bool spin_until(const std::function<bool()>& predicate, std::chrono::milliseconds timeout = 2s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
    return predicate();
  }
  void expect_sensor_publisher(const StreamRow& row) {
    const auto infos = source_->get_publishers_info_by_topic(topic(row.topic));
    ASSERT_EQ(infos.size(), 1U) << row.topic;
    EXPECT_EQ(infos.front().topic_type(), row.ros_type);
    const auto& qos = infos.front().qos_profile();
    EXPECT_EQ(qos.history(), rclcpp::HistoryPolicy::KeepLast);
    EXPECT_EQ(qos.depth(), 5U);
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
    EXPECT_EQ(source_->count_publishers(std::string{"/"} + row.topic), 0U);
  }
  std::string topic(const char* name) const {
    return namespace_ + "/" + name;
  }

  static std::atomic<std::uint64_t> next_namespace_;
  StreamSdkHarness sdk_;
  std::string namespace_;
  rclcpp::Node::SharedPtr source_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<wuji_msgs::msg::HandJointAngles>::SharedPtr angles_sub_;
  std::array<rclcpp::Subscription<wuji_msgs::msg::TrackedPoseArray>::SharedPtr, 3> pose_subs_;
  std::vector<wuji_msgs::msg::HandJointAngles> angle_messages_;
  std::array<std::vector<wuji_msgs::msg::TrackedPoseArray>, 3> pose_messages_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::array<rclcpp::Subscription<wuji_msgs::msg::TactileGrid>::SharedPtr, 3> grid_subs_;
  rclcpp::Subscription<wuji_msgs::msg::TactileZoneArray>::SharedPtr zones_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  std::vector<sensor_msgs::msg::Imu> imu_messages_;
  std::array<std::vector<wuji_msgs::msg::TactileGrid>, 3> grid_messages_;
  std::vector<wuji_msgs::msg::TactileZoneArray> zone_messages_;
  std::vector<sensor_msgs::msg::PointCloud2> point_cloud_messages_;
};

std::atomic<std::uint64_t> GloveStreamsTest::next_namespace_{1U};

TEST_F(GloveStreamsTest, FourRowWiringPublishesRepresentativeFramesAndStopsExactlyOnce) {
  capture_messages();
  auto angles_stream = make_joint_angles();
  auto emf_stream = make_emf_poses();
  auto tip_stream = make_tip_poses();
  auto skeleton_stream = make_skeleton();
  std::array<StreamController*, 4> streams{angles_stream.get(), emf_stream.get(), tip_stream.get(),
                                           skeleton_stream.get()};
  std::set<std::uint16_t> keys;
  for (std::size_t index = 0; index < streams.size(); ++index) {
    ASSERT_TRUE(streams[index]->start()) << kRows[index].topic;
    const auto snapshot = streams[index]->snapshot();
    EXPECT_EQ(snapshot.key, kRows[index].key);
    EXPECT_EQ(snapshot.policy, kRows[index].policy);
    EXPECT_TRUE(keys.insert(snapshot.key.value).second);
    EXPECT_EQ(sdk_.subscribe_count(index), 1U) << kRows[index].sdk_counter;
    expect_sensor_publisher(kRows[index]);
  }
  ASSERT_TRUE(spin_until([this]() {
    return angles_sub_->get_publisher_count() == 1U && pose_subs_[0]->get_publisher_count() == 1U &&
           pose_subs_[1]->get_publisher_count() == 1U && pose_subs_[2]->get_publisher_count() == 1U;
  }));

  std::array<WujiFingerJointAngles, 5> fingers{};
  fingers[0].angles[0] = 1.25;
  WujiHandJointAngles angles{};
  set_header(angles.header, 1000001U, "angles_frame");
  angles.fingers = fingers.data();
  angles.fingers_len = fingers.size();
  std::array<WujiEmfPose, 5> emf_entries{};
  WujiEmfPoseArray emf{};
  set_header(emf.header, 2000002U, "emf_frame");
  emf.poses = emf_entries.data();
  emf.poses_len = emf_entries.size();
  std::array<WujiFingertipPose, 5> tip_entries{};
  WujiFingertipPoses tip{};
  set_header(tip.header, 3000003U, "tip_frame");
  tip.poses = tip_entries.data();
  tip.poses_len = tip_entries.size();
  WujiSkeletonJoint joint{};
  std::strncpy(joint.name, "sdk_wrist", sizeof(joint.name) - 1U);
  WujiHandSkeleton skeleton{};
  set_header(skeleton.header, 4000004U, "skeleton_frame");
  skeleton.joints = &joint;
  skeleton.joints_len = 1U;

  sdk_.emit_joint_angles(WUJI_FRAME_KIND_OK, &angles);
  sdk_.emit_emf_poses(WUJI_FRAME_KIND_OK, &emf);
  sdk_.emit_tip_poses(WUJI_FRAME_KIND_OK, &tip);
  sdk_.emit_hand_skeleton(WUJI_FRAME_KIND_OK, &skeleton);
  ASSERT_TRUE(spin_until([this]() {
    return angle_messages_.size() == 1U && pose_messages_[0].size() == 1U &&
           pose_messages_[1].size() == 1U && pose_messages_[2].size() == 1U;
  }));
  EXPECT_EQ(angle_messages_.front().header.frame_id, "angles_frame");
  EXPECT_DOUBLE_EQ(angle_messages_.front().fingers[0].angles[0], 1.25);
  EXPECT_EQ(pose_messages_[0].front().header.frame_id, "emf_frame");
  EXPECT_EQ(pose_messages_[1].front().header.frame_id, "tip_frame");
  ASSERT_EQ(pose_messages_[2].front().poses.size(), 1U);
  EXPECT_EQ(pose_messages_[2].front().poses[0].name, "sdk_wrist");

  for (std::size_t index = streams.size(); index > 0U; --index) {
    streams[index - 1U]->stop();
    streams[index - 1U]->stop();
    EXPECT_EQ(sdk_.close_count(index - 1U), 1U);
  }
  EXPECT_EQ(sdk_.total_close_count(), 4U);
}

TEST_F(GloveStreamsTest, SixOptionalRowsPublishRepresentativeFramesAndStopExactlyOnce) {
  capture_task7_messages();
  auto imu_stream = make_imu();
  auto tactile_stream = make_tactile();
  auto residual_stream = make_tactile_residual();
  auto binary_stream = make_tactile_binary();
  auto zones_stream = make_tactile_zones();
  auto point_cloud_stream = make_tactile_point_cloud();
  std::array<StreamController*, 6> streams{imu_stream.get(),      tactile_stream.get(),
                                           residual_stream.get(), binary_stream.get(),
                                           zones_stream.get(),    point_cloud_stream.get()};
  std::set<std::uint16_t> keys{1U, 2U, 3U, 4U};
  for (std::size_t index = 0; index < streams.size(); ++index) {
    const std::size_t row = index + 4U;
    ASSERT_TRUE(streams[index]->start()) << kRows[row].topic;
    const auto snapshot = streams[index]->snapshot();
    EXPECT_EQ(snapshot.key, kRows[row].key);
    EXPECT_EQ(snapshot.policy, StreamPolicy::Optional);
    EXPECT_TRUE(keys.insert(snapshot.key.value).second);
    EXPECT_EQ(sdk_.subscribe_count(row), 1U) << kRows[row].sdk_counter;
    expect_sensor_publisher(kRows[row]);
  }
  ASSERT_TRUE(spin_until([this]() {
    if (imu_sub_->get_publisher_count() != 1U) {
      return false;
    }
    for (const auto& subscription : grid_subs_) {
      if (subscription->get_publisher_count() != 1U) {
        return false;
      }
    }
    return zones_sub_->get_publisher_count() == 1U && point_cloud_sub_->get_publisher_count() == 1U;
  }));

  WujiImuData imu{};
  set_header(imu.header, 5000000U, kRows[4].topic);
  imu.angular_velocity.x = 1.0;
  sdk_.emit_imu(WUJI_FRAME_KIND_OK, &imu);
  std::array<float, 744> tactile_data{};
  tactile_data[0] = 1.0F;
  tactile_data[1] = 2.0F;
  tactile_data[2] = 3.0F;
  WujiTactileFrame tactile{};
  set_header(tactile.header, 6000001U, "tactile_frame");
  tactile.data = tactile_data.data();
  tactile.data_len = tactile_data.size();
  WujiTactileResidual residual{};
  set_header(residual.header, 6000002U, "residual_frame");
  residual.data = tactile_data.data();
  residual.data_len = tactile_data.size();
  WujiTactileBinary binary{};
  set_header(binary.header, 6000003U, "binary_frame");
  binary.data = tactile_data.data();
  binary.data_len = tactile_data.size();
  float palm_value = 4.0F;
  WujiTactileZones zones{};
  set_header(zones.header, 6000004U, "zones_frame");
  zones.palm = &palm_value;
  zones.palm_len = 1U;
  std::array<std::uint8_t, 6> point_data{1U, 2U, 3U, 4U, 5U, 6U};
  WujiPointCloud point_cloud{};
  set_header(point_cloud.header, 6000005U, "header_frame");
  std::strncpy(point_cloud.frame_id, "points_frame", sizeof(point_cloud.frame_id) - 1U);
  point_cloud.point_stride = 3U;
  point_cloud.data = point_data.data();
  point_cloud.data_len = point_data.size();
  sdk_.emit_tactile(WUJI_FRAME_KIND_OK, &tactile);
  sdk_.emit_tactile_residual(WUJI_FRAME_KIND_OK, &residual);
  sdk_.emit_tactile_binary(WUJI_FRAME_KIND_OK, &binary);
  sdk_.emit_tactile_zones(WUJI_FRAME_KIND_OK, &zones);
  sdk_.emit_tactile_point_cloud(WUJI_FRAME_KIND_OK, &point_cloud);
  ASSERT_TRUE(spin_until([this]() {
    if (imu_messages_.size() != 1U) {
      return false;
    }
    for (const auto& messages : grid_messages_) {
      if (messages.size() != 1U) {
        return false;
      }
    }
    return zone_messages_.size() == 1U && point_cloud_messages_.size() == 1U;
  }));
  EXPECT_DOUBLE_EQ(imu_messages_.front().angular_velocity.x, 1.0);
  EXPECT_EQ(grid_messages_[0].front().header.frame_id, "tactile_frame");
  EXPECT_EQ(grid_messages_[1].front().header.frame_id, "residual_frame");
  EXPECT_EQ(grid_messages_[2].front().header.frame_id, "binary_frame");
  ASSERT_EQ(zone_messages_.front().zones.size(), 6U);
  ASSERT_EQ(zone_messages_.front().zones[0].values.size(), 1U);
  EXPECT_FLOAT_EQ(zone_messages_.front().zones[0].values[0], palm_value);
  EXPECT_EQ(point_cloud_messages_.front().header.frame_id, "points_frame");
  EXPECT_EQ(point_cloud_messages_.front().width, 2U);

  for (std::size_t index = streams.size(); index > 0U; --index) {
    streams[index - 1U]->stop();
    streams[index - 1U]->stop();
    EXPECT_EQ(sdk_.close_count(index + 3U), 1U);
  }
  EXPECT_EQ(sdk_.total_close_count(), 6U);
}

TEST_F(GloveStreamsTest, DropsMalformedTactileFramesWithoutFault) {
  capture_task7_messages();
  auto tactile_stream = make_tactile();
  auto residual_stream = make_tactile_residual();
  auto binary_stream = make_tactile_binary();
  auto zones_stream = make_tactile_zones();
  auto point_cloud_stream = make_tactile_point_cloud();
  ASSERT_TRUE(tactile_stream->start());
  ASSERT_TRUE(residual_stream->start());
  ASSERT_TRUE(binary_stream->start());
  ASSERT_TRUE(zones_stream->start());
  ASSERT_TRUE(point_cloud_stream->start());
  ASSERT_TRUE(spin_until([this]() {
    for (const auto& subscription : grid_subs_) {
      if (subscription->get_publisher_count() != 1U) {
        return false;
      }
    }
    return zones_sub_->get_publisher_count() == 1U && point_cloud_sub_->get_publisher_count() == 1U;
  }));

  std::array<float, 744> valid_grid_values{};
  valid_grid_values[0] = 1.0F;
  WujiTactileFrame valid_tactile{};
  valid_tactile.data = valid_grid_values.data();
  valid_tactile.data_len = valid_grid_values.size();
  WujiTactileResidual valid_residual{};
  valid_residual.data = valid_grid_values.data();
  valid_residual.data_len = valid_grid_values.size();
  WujiTactileBinary valid_binary{};
  valid_binary.data = valid_grid_values.data();
  valid_binary.data_len = valid_grid_values.size();
  WujiTactileZones valid_zones{};
  std::uint8_t valid_point = 1U;
  WujiPointCloud valid_point_cloud{};
  std::strncpy(valid_point_cloud.frame_id, "baseline", sizeof(valid_point_cloud.frame_id) - 1U);
  valid_point_cloud.point_stride = 1U;
  valid_point_cloud.data = &valid_point;
  valid_point_cloud.data_len = 1U;
  sdk_.emit_tactile(WUJI_FRAME_KIND_OK, &valid_tactile);
  sdk_.emit_tactile_residual(WUJI_FRAME_KIND_OK, &valid_residual);
  sdk_.emit_tactile_binary(WUJI_FRAME_KIND_OK, &valid_binary);
  sdk_.emit_tactile_zones(WUJI_FRAME_KIND_OK, &valid_zones);
  sdk_.emit_tactile_point_cloud(WUJI_FRAME_KIND_OK, &valid_point_cloud);
  ASSERT_TRUE(spin_until([this]() {
    for (const auto& messages : grid_messages_) {
      if (messages.size() != 1U) {
        return false;
      }
    }
    return zone_messages_.size() == 1U && point_cloud_messages_.size() == 1U;
  }));
  for (auto& messages : grid_messages_) {
    messages.clear();
  }
  zone_messages_.clear();
  point_cloud_messages_.clear();

  WujiTactileFrame tactile{};
  tactile.data = valid_grid_values.data();
  tactile.data_len = 1U;
  WujiTactileResidual residual{};
  residual.data = valid_grid_values.data();
  residual.data_len = 1U;
  WujiTactileBinary binary{};
  binary.data = valid_grid_values.data();
  binary.data_len = 1U;
  WujiTactileZones zones{};
  zones.thumb_len = 1U;
  WujiPointCloud point_cloud{};
  point_cloud.point_stride = 3U;
  point_cloud.data_len = 1U;
  sdk_.emit_tactile(WUJI_FRAME_KIND_OK, &tactile);
  sdk_.emit_tactile_residual(WUJI_FRAME_KIND_OK, &residual);
  sdk_.emit_tactile_binary(WUJI_FRAME_KIND_OK, &binary);
  sdk_.emit_tactile_zones(WUJI_FRAME_KIND_OK, &zones);
  sdk_.emit_tactile_point_cloud(WUJI_FRAME_KIND_OK, &point_cloud);
  spin_until([]() { return false; }, 100ms);

  for (const auto& messages : grid_messages_) {
    EXPECT_TRUE(messages.empty());
  }
  EXPECT_TRUE(zone_messages_.empty());
  EXPECT_TRUE(point_cloud_messages_.empty());
  std::array<StreamController*, 5> streams{tactile_stream.get(), residual_stream.get(),
                                           binary_stream.get(), zones_stream.get(),
                                           point_cloud_stream.get()};
  for (const auto* stream : streams) {
    EXPECT_EQ(stream->snapshot().fault.kind, TerminalKind::None);
  }
}

TEST_F(GloveStreamsTest, DropsMalformedPoseJointAndSkeletonFramesWithoutFault) {
  capture_messages();
  auto angles_stream = make_joint_angles();
  auto emf_stream = make_emf_poses();
  auto skeleton_stream = make_skeleton();
  ASSERT_TRUE(angles_stream->start());
  ASSERT_TRUE(emf_stream->start());
  ASSERT_TRUE(skeleton_stream->start());
  ASSERT_TRUE(spin_until([this]() {
    return angles_sub_->get_publisher_count() == 1U && pose_subs_[0]->get_publisher_count() == 1U &&
           pose_subs_[2]->get_publisher_count() == 1U;
  }));

  std::array<WujiFingerJointAngles, 5> valid_fingers{};
  WujiHandJointAngles valid_angles{};
  valid_angles.fingers = valid_fingers.data();
  valid_angles.fingers_len = valid_fingers.size();
  std::array<WujiEmfPose, 5> valid_poses{};
  WujiEmfPoseArray valid_emf{};
  valid_emf.poses = valid_poses.data();
  valid_emf.poses_len = valid_poses.size();
  WujiSkeletonJoint valid_joint{};
  std::strncpy(valid_joint.name, "wrist", sizeof(valid_joint.name) - 1U);
  WujiHandSkeleton valid_skeleton{};
  valid_skeleton.joints = &valid_joint;
  valid_skeleton.joints_len = 1U;
  sdk_.emit_joint_angles(WUJI_FRAME_KIND_OK, &valid_angles);
  sdk_.emit_emf_poses(WUJI_FRAME_KIND_OK, &valid_emf);
  sdk_.emit_hand_skeleton(WUJI_FRAME_KIND_OK, &valid_skeleton);
  ASSERT_TRUE(spin_until([this]() {
    return angle_messages_.size() == 1U && pose_messages_[0].size() == 1U &&
           pose_messages_[2].size() == 1U;
  }));
  angle_messages_.clear();
  pose_messages_[0].clear();
  pose_messages_[2].clear();

  std::array<WujiFingerJointAngles, 4> fingers{};
  WujiHandJointAngles angles{};
  angles.fingers = fingers.data();
  angles.fingers_len = fingers.size();
  std::array<WujiEmfPose, 4> poses{};
  WujiEmfPoseArray emf{};
  emf.poses = poses.data();
  emf.poses_len = poses.size();
  WujiHandSkeleton skeleton{};
  skeleton.joints_len = 1U;
  sdk_.emit_joint_angles(WUJI_FRAME_KIND_OK, &angles);
  sdk_.emit_emf_poses(WUJI_FRAME_KIND_OK, &emf);
  sdk_.emit_hand_skeleton(WUJI_FRAME_KIND_OK, &skeleton);
  spin_until([]() { return false; }, 100ms);

  EXPECT_TRUE(angle_messages_.empty());
  EXPECT_TRUE(pose_messages_[0].empty());
  EXPECT_TRUE(pose_messages_[2].empty());
  EXPECT_EQ(angles_stream->snapshot().fault.kind, TerminalKind::None);
  EXPECT_EQ(emf_stream->snapshot().fault.kind, TerminalKind::None);
  EXPECT_EQ(skeleton_stream->snapshot().fault.kind, TerminalKind::None);
}

TEST_F(GloveStreamsTest, IgnoresLagAndLatchesTerminalWithoutClosingInCallback) {
  auto angles_stream = make_joint_angles();
  auto tip_stream = make_tip_poses();
  ASSERT_TRUE(angles_stream->start());
  ASSERT_TRUE(tip_stream->start());
  sdk_.emit_joint_angles(WUJI_FRAME_KIND_LAG, nullptr);
  EXPECT_EQ(angles_stream->snapshot().fault.kind, TerminalKind::None);
  sdk_.emit_joint_angles(WUJI_FRAME_KIND_END, nullptr);
  sdk_.emit_joint_angles(WUJI_FRAME_KIND_ERROR, nullptr);
  sdk_.emit_tip_poses(WUJI_FRAME_KIND_ERROR, nullptr);

  EXPECT_EQ(angles_stream->snapshot().fault.kind, TerminalKind::End);
  EXPECT_EQ(angles_stream->snapshot().state, StreamState::Terminal);
  EXPECT_EQ(tip_stream->snapshot().fault.kind, TerminalKind::Error);
  EXPECT_EQ(tip_stream->snapshot().state, StreamState::Terminal);
  EXPECT_EQ(sdk_.total_close_count(), 0U);
  EXPECT_EQ(source_->count_publishers(topic("joint_angles")), 1U);
  EXPECT_EQ(source_->count_publishers(topic("tip_poses")), 1U);
}

}  // namespace
}  // namespace wuji_glove_driver::stream
