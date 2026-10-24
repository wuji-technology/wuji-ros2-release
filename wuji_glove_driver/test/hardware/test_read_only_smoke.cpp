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

#include <wuji_sdk.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <wuji_msgs/msg/hand_joint_angles.hpp>
#include <wuji_msgs/msg/tactile_grid.hpp>
#include <wuji_msgs/msg/tactile_zone_array.hpp>
#include <wuji_msgs/msg/tracked_pose_array.hpp>
#include "hardware_test_config.hpp"
#include "hardware_test_validation.hpp"
#include "read_only_stream_probe.hpp"

#include "wuji_glove_driver/conversion/imu_conversion.hpp"
#include "wuji_glove_driver/conversion/joint_angles_conversion.hpp"
#include "wuji_glove_driver/conversion/point_cloud_conversion.hpp"
#include "wuji_glove_driver/conversion/pose_conversion.hpp"
#include "wuji_glove_driver/conversion/tactile_conversion.hpp"
#include "wuji_glove_driver/sdk/glove_sdk_api.hpp"
#include "wuji_glove_driver/sdk/glove_session.hpp"

namespace wuji_glove_driver::hardware_test {
namespace {

std::atomic_bool interrupted{false};

extern "C" void handle_signal(int) {
  interrupted.store(true);
}

class SignalGuard {
 public:
  SignalGuard()
      : old_int_(std::signal(SIGINT, handle_signal)),
        old_term_(std::signal(SIGTERM, handle_signal)) {}
  ~SignalGuard() noexcept {
    (void)std::signal(SIGINT, old_int_);
    (void)std::signal(SIGTERM, old_term_);
  }

 private:
  using Handler = void (*)(int);
  Handler old_int_;
  Handler old_term_;
};

struct JointConverter {
  auto operator()(const WujiHandJointAngles& input, wuji_msgs::msg::HandJointAngles& output) const {
    return conversion::to_hand_joint_angles(input, output);
  }
};
struct EmfConverter {
  auto operator()(const WujiEmfPoseArray& input, wuji_msgs::msg::TrackedPoseArray& output) const {
    return conversion::to_emf_poses(input, output);
  }
};
struct TipConverter {
  auto operator()(const WujiFingertipPoses& input, wuji_msgs::msg::TrackedPoseArray& output) const {
    return conversion::to_tip_poses(input, output);
  }
};
struct SkeletonConverter {
  auto operator()(const WujiHandSkeleton& input, wuji_msgs::msg::TrackedPoseArray& output) const {
    return conversion::to_hand_skeleton(input, output);
  }
};
struct ImuConverter {
  auto operator()(const WujiImuData& input, sensor_msgs::msg::Imu& output) const {
    return conversion::to_imu(input, output);
  }
};
struct TactileConverter {
  auto operator()(const WujiTactileFrame& input, wuji_msgs::msg::TactileGrid& output) const {
    return conversion::to_tactile_grid(input, output);
  }
};
struct ResidualConverter {
  auto operator()(const WujiTactileResidual& input, wuji_msgs::msg::TactileGrid& output) const {
    return conversion::to_tactile_grid(input, output);
  }
};
struct BinaryConverter {
  auto operator()(const WujiTactileBinary& input, wuji_msgs::msg::TactileGrid& output) const {
    return conversion::to_tactile_grid(input, output);
  }
};
struct ZonesConverter {
  auto operator()(const WujiTactileZones& input, wuji_msgs::msg::TactileZoneArray& output) const {
    return conversion::to_tactile_zones(input, output);
  }
};
struct PointsConverter {
  auto operator()(const WujiPointCloud& input, sensor_msgs::msg::PointCloud2& output) const {
    return conversion::to_point_cloud2(input, output);
  }
};

// One machine-readable line per check: RESULT<TAB>PASS|FAIL|SKIP<TAB>check<TAB>message.
void report(const char* status, const std::string& check, std::string message) {
  std::replace_if(
      message.begin(), message.end(),
      [](const char character) { return character == '\t' || character == '\n'; }, ' ');
  std::cout << "RESULT\t" << status << '\t' << check << '\t' << message << std::endl;
}

constexpr const char* kStreamNames[] = {
    "joint_angles", "emf_poses",        "tip_poses",      "hand_skeleton", "imu/palm",
    "tactile",      "tactile/residual", "tactile/binary", "tactile/zones", "tactile/points"};

int run_read_only_smoke(const HardwareTestConfig& config) {
  interrupted.store(false);
  SignalGuard signal_guard;
  auto api = sdk::GloveSdkApi::production();
  sdk::GloveSessionHooks hooks;
  hooks.cancelled = []() { return interrupted.load(); };
  sdk::GloveSession session(api, std::move(hooks));
  sdk::GloveConnectConfig connect_config;
  connect_config.endpoint = config.driver.endpoint;
  connect_config.serial_number = config.driver.serial_number;
  connect_config.hand_side = config.driver.hand_side;
  connect_config.connect_timeout = config.driver.connect_timeout;
  const auto connect_status = session.connect(connect_config);
  if (!connect_status) {
    std::cerr << "FAIL: " << connect_status.message << '\n';
    report("FAIL", "connect", connect_status.message);
    return 1;
  }
  report("PASS", "connect", "serial=" + session.serial_number());
  std::cout << "CONNECTED: serial=" << session.serial_number() << " model=" << session.model()
            << " hand_side=" << session.hand_side() << '\n';

  using JointProbe =
      StreamProbe<WujiHandJointAngles, wuji_msgs::msg::HandJointAngles, JointConverter>;
  using PoseProbe = StreamProbe<WujiEmfPoseArray, wuji_msgs::msg::TrackedPoseArray, EmfConverter>;
  using TipProbe = StreamProbe<WujiFingertipPoses, wuji_msgs::msg::TrackedPoseArray, TipConverter>;
  using SkeletonProbe =
      StreamProbe<WujiHandSkeleton, wuji_msgs::msg::TrackedPoseArray, SkeletonConverter>;
  using ImuProbe = StreamProbe<WujiImuData, sensor_msgs::msg::Imu, ImuConverter>;
  using TactileProbe = StreamProbe<WujiTactileFrame, wuji_msgs::msg::TactileGrid, TactileConverter>;
  using ResidualProbe =
      StreamProbe<WujiTactileResidual, wuji_msgs::msg::TactileGrid, ResidualConverter>;
  using BinaryProbe = StreamProbe<WujiTactileBinary, wuji_msgs::msg::TactileGrid, BinaryConverter>;
  using ZonesProbe =
      StreamProbe<WujiTactileZones, wuji_msgs::msg::TactileZoneArray, ZonesConverter>;
  using PointsProbe = StreamProbe<WujiPointCloud, sensor_msgs::msg::PointCloud2, PointsConverter>;

  JointProbe joint{"joint_angles", interrupted};
  PoseProbe emf{"emf_poses", interrupted};
  TipProbe tip{"tip_poses", interrupted};
  SkeletonProbe skeleton{"hand_skeleton", interrupted};
  ImuProbe palm{"imu/palm", interrupted};
  TactileProbe tactile{"tactile", interrupted};
  ResidualProbe residual{"tactile/residual", interrupted};
  BinaryProbe binary{"tactile/binary", interrupted};
  ZonesProbe zones{"tactile/zones", interrupted};
  PointsProbe points{"tactile/points", interrupted};

  SubscriptionSet subscriptions(api);
  std::string error;
  std::vector<std::pair<const char*, JointProbe*>> joints;
  std::vector<std::pair<const char*, PoseProbe*>> emfs;
  std::vector<std::pair<const char*, TipProbe*>> tips;
  std::vector<std::pair<const char*, SkeletonProbe*>> skeletons;
  std::vector<std::pair<const char*, ImuProbe*>> imus;
  std::vector<std::pair<const char*, TactileProbe*>> tactiles;
  std::vector<std::pair<const char*, ResidualProbe*>> residuals;
  std::vector<std::pair<const char*, BinaryProbe*>> binaries;
  std::vector<std::pair<const char*, ZonesProbe*>> zone_streams;
  std::vector<std::pair<const char*, PointsProbe*>> point_streams;
  WujiDevice* const device = session.device();

  // A failed stream stops only itself, so one run reports every stream.
  std::vector<std::string> failed;
  const auto fail = [&](const char* name, const std::string& message) {
    std::cerr << "FAIL: " << message << '\n';
    report("FAIL", name, message);
    failed.emplace_back(name);
  };
  const auto has_failed = [&](const char* name) {
    return std::find(failed.begin(), failed.end(), name) != failed.end();
  };

#define START(name, required, enabled, member, probe, active)                               \
  if (!start_stream(name, required, enabled, device, api, subscriptions, probe, api.member, \
                    active, error)) {                                                       \
    fail(name, error);                                                                      \
  }
  START("joint_angles", true, true, subscribe_joint_angles, joint, joints)
  START("emf_poses", false, config.driver.publish_emf_poses, subscribe_emf_poses, emf, emfs)
  START("tip_poses", false, config.driver.publish_tip_poses, subscribe_tip_poses, tip, tips)
  START("hand_skeleton", false, config.driver.publish_skeleton, subscribe_hand_skeleton, skeleton,
        skeletons)
  START("imu/palm", false, config.driver.publish_imu, subscribe_imu_palm, palm, imus)
  START("tactile", false, config.driver.publish_tactile, subscribe_tactile, tactile, tactiles)
  START("tactile/residual", false, config.driver.publish_tactile_residual,
        subscribe_tactile_residual, residual, residuals)
  START("tactile/binary", false, config.driver.publish_tactile_binary, subscribe_tactile_binary,
        binary, binaries)
  START("tactile/zones", false, config.driver.publish_tactile_zones, subscribe_tactile_zones, zones,
        zone_streams)
  START("tactile/points", false, config.driver.publish_tactile_points,
        subscribe_tactile_point_cloud, points, point_streams)
#undef START

  const auto stream_deadline = std::chrono::steady_clock::now() +
                               std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                   std::chrono::duration<double>{config.stream_timeout_seconds});
  std::vector<std::string> started;
  const auto for_each_stream = [&](const auto& step) {
    step(joints);
    step(emfs);
    step(tips);
    step(skeletons);
    step(imus);
    step(tactiles);
    step(residuals);
    step(binaries);
    step(zone_streams);
    step(point_streams);
  };
  for_each_stream([&](const auto& group) {
    for (const auto& entry : group) {
      started.emplace_back(entry.first);
      if (!entry.second->wait_until(stream_deadline, error)) {
        fail(entry.first, error);
        continue;
      }
      std::cout << "PASS: " << entry.first << " received a convertible OK frame\n";
    }
  });
  for_each_stream([&](const auto& group) {
    for (const auto& entry : group) {
      if (!has_failed(entry.first) && !entry.second->snapshot_success(error)) {
        fail(entry.first, error);
      }
    }
  });
  subscriptions.close_all();
  for_each_stream([&](const auto& group) {
    for (const auto& entry : group) {
      if (!has_failed(entry.first) && !entry.second->check_after_close(error)) {
        fail(entry.first, error);
      }
    }
  });
  for (const char* name : kStreamNames) {
    if (has_failed(name)) {
      continue;
    }
    if (std::find(started.begin(), started.end(), name) != started.end()) {
      report("PASS", name, "received a convertible OK frame");
    } else {
      report("SKIP", name, "disabled by publish flag or unavailable");
    }
  }
  if (!failed.empty()) {
    return 1;
  }
  std::cout << "PASS: read-only Glove hardware smoke completed\n";
  return 0;
}

}  // namespace
}  // namespace wuji_glove_driver::hardware_test

int main() {
  return wuji_glove_driver::hardware_test::run_hardware_exception_boundary(std::cerr, []() {
    const auto result = wuji_glove_driver::hardware_test::load_hardware_test_config();
    if (!result.config) {
      std::cerr << result.message << '\n';
      return result.exit_code;
    }
    return wuji_glove_driver::hardware_test::run_read_only_smoke(*result.config);
  });
}
