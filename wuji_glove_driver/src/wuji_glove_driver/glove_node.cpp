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

#include "wuji_glove_driver/glove_node.hpp"

#include <wuji_sdk.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/context.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>
#include <wuji_driver_common/stream/stream_registry.hpp>
#include <wuji_driver_common/stream/stream_types.hpp>

#include "wuji_glove_driver/driver_config.hpp"
#include "wuji_glove_driver/lifecycle/exit_state.hpp"
#include "wuji_glove_driver/sdk/glove_session.hpp"
#include "wuji_glove_driver/stream/emf_poses_stream.hpp"
#include "wuji_glove_driver/stream/imu_stream.hpp"
#include "wuji_glove_driver/stream/joint_angles_stream.hpp"
#include "wuji_glove_driver/stream/skeleton_stream.hpp"
#include "wuji_glove_driver/stream/tactile_binary_stream.hpp"
#include "wuji_glove_driver/stream/tactile_point_cloud_stream.hpp"
#include "wuji_glove_driver/stream/tactile_residual_stream.hpp"
#include "wuji_glove_driver/stream/tactile_stream.hpp"
#include "wuji_glove_driver/stream/tactile_zones_stream.hpp"
#include "wuji_glove_driver/stream/tip_poses_stream.hpp"

namespace wuji_glove_driver {
namespace {

using namespace std::chrono_literals;
using wuji_driver_common::stream::StreamKey;
using wuji_driver_common::stream::StreamPolicy;
using wuji_driver_common::stream::StreamSnapshot;
using wuji_driver_common::stream::StreamState;
using wuji_driver_common::stream::TerminalKind;

constexpr auto kFaultBridgePeriod = 50ms;
constexpr StreamKey kJointAnglesKey{1U};
constexpr StreamKey kEmfPosesKey{2U};
constexpr StreamKey kTipPosesKey{3U};
constexpr StreamKey kHandSkeletonKey{4U};
constexpr StreamKey kImuPalmKey{5U};
constexpr StreamKey kTactileKey{6U};
constexpr StreamKey kTactileResidualKey{7U};
constexpr StreamKey kTactileBinaryKey{8U};
constexpr StreamKey kTactileZonesKey{9U};
constexpr StreamKey kTactilePointCloudKey{10U};

rclcpp::NodeOptions without_parameter_endpoints(rclcpp::NodeOptions options) {
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  return options;
}

std::string exception_message(const char* boundary) {
  try {
    throw;
  } catch (const std::exception& error) {
    return std::string{boundary} + ": " + error.what();
  } catch (...) {
    return std::string{boundary} + ": unknown exception";
  }
}

const char* stream_label(StreamKey key) noexcept {
  switch (key.value) {
    case 1U:
      return "joint_angles";
    case 2U:
      return "emf_poses";
    case 3U:
      return "tip_poses";
    case 4U:
      return "hand_skeleton";
    case 5U:
      return "imu/palm";
    case 6U:
      return "tactile";
    case 7U:
      return "tactile/residual";
    case 8U:
      return "tactile/binary";
    case 9U:
      return "tactile/zones";
    case 10U:
      return "tactile/points";
    default:
      return "unknown";
  }
}

std::string stream_fault_message(const StreamSnapshot& snapshot) {
  std::string message = stream_label(snapshot.key);
  message += snapshot.fault.kind == TerminalKind::End ? " END" : " ERROR";
  if (!snapshot.fault.reason.empty()) {
    message += ": " + snapshot.fault.reason;
  }
  return message;
}

GloveNodeHooks complete_hooks(GloveNode& node, GloveNodeHooks hooks) {
  const auto context = node.get_node_base_interface()->get_context();
  if (!hooks.now) {
    hooks.now = []() { return GloveNodeHooks::Clock::now(); };
  }
  if (!hooks.sleep_for) {
    hooks.sleep_for = [](GloveNodeHooks::Clock::duration duration) {
      std::this_thread::sleep_for(duration);
    };
  }
  if (!hooks.cancelled) {
    hooks.cancelled = [context]() { return !context->is_valid(); };
  }
  if (!hooks.request_shutdown) {
    hooks.request_shutdown = [context]() { rclcpp::shutdown(context); };
  }
  return hooks;
}

}  // namespace

class GloveNode::Impl {
 public:
  Impl(GloveNode& node, sdk::GloveSdkApi sdk_api, GloveNodeHooks hooks)
      : node_(node),
        sdk_(std::move(sdk_api)),
        hooks_(complete_hooks(node, std::move(hooks))),
        session_(sdk_, sdk::GloveSessionHooks{hooks_.now, hooks_.sleep_for, hooks_.cancelled}) {}

  sdk::Status start() {
    {
      const std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_ == State::Running) {
        return sdk::Status::success();
      }
      if (state_ != State::Constructed) {
        return sdk::Status::failure("GloveNode cannot restart after stopping");
      }
      state_ = State::Starting;
    }

    try {
      sdk::Status status = read_driver_config(node_, config_);
      if (!status) {
        return fail_start(std::move(status));
      }
      status = session_.connect(sdk::GloveConnectConfig{
          config_.endpoint, config_.serial_number, config_.hand_side, config_.connect_timeout});
      if (!status) {
        return fail_start(std::move(status));
      }
      status = register_streams();
      if (!status) {
        return fail_start(std::move(status));
      }
      status = registry_->start_required();
      if (!status) {
        return fail_start(std::move(status));
      }
      status = registry_->start_optional();
      // A shutdown request invalidates the context, so the optional streams that were still
      // starting fail to create their publishers. Report the stop, not those failures.
      if (hooks_.cancelled()) {
        return fail_start(sdk::Status::failure("shutdown requested during stream startup"));
      }
      if (!status) {
        RCLCPP_ERROR(node_.get_logger(), "optional stream startup failed: %s",
                     status.message.c_str());
      }
      for (const auto& snapshot : registry_->snapshots()) {
        if (snapshot.policy == StreamPolicy::Optional &&
            snapshot.fault.kind != TerminalKind::None) {
          const auto message = stream_fault_message(snapshot);
          RCLCPP_ERROR(node_.get_logger(), "optional stream startup failed: %s", message.c_str());
        }
      }
      fault_timer_ = node_.create_wall_timer(kFaultBridgePeriod, [this]() { on_fault_timer(); });
    } catch (...) {
      return fail_start(sdk::Status::failure(exception_message("GloveNode start")));
    }

    {
      const std::lock_guard<std::mutex> lock(state_mutex_);
      state_ = State::Running;
    }
    RCLCPP_INFO(node_.get_logger(), "GloveNode started");
    return sdk::Status::success();
  }

  void stop() noexcept {
    {
      const std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_ == State::Stopping || state_ == State::Stopped) {
        return;
      }
      state_ = State::Stopping;
    }
    stopping_.store(true, std::memory_order_release);
    try {
      fault_timer_.reset();
      if (registry_) {
        registry_->stop_all();
      }
      session_.disconnect();
      registry_.reset();
    } catch (...) {
      RCLCPP_ERROR(node_.get_logger(), "GloveNode stop contained an exception");
      try {
        session_.disconnect();
      } catch (...) {
      }
    }
    const std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = State::Stopped;
  }

  bool exit_requested() const noexcept {
    return exit_state_.requested();
  }

  int exit_code() const noexcept {
    return exit_state_.code();
  }

  std::string exit_reason() const {
    return exit_state_.reason();
  }

 private:
  enum class State { Constructed, Starting, Running, Stopping, Stopped };

  sdk::Status fail_start(sdk::Status status) {
    // Once Ctrl+C or SIGTERM arrived, the failure is a consequence of the stop the user asked for.
    if (hooks_.cancelled()) {
      exit_state_.request(EXIT_SUCCESS, "startup cancelled by shutdown request: " + status.message);
      RCLCPP_INFO(node_.get_logger(), "GloveNode startup cancelled by shutdown request: %s",
                  status.message.c_str());
    } else {
      exit_state_.request(EXIT_FAILURE, status.message);
      RCLCPP_ERROR(node_.get_logger(), "GloveNode startup failed: %s", status.message.c_str());
    }
    stop();
    return status;
  }

  sdk::Status register_streams() {
    registry_ = std::make_unique<wuji_driver_common::stream::StreamRegistry>();
    sdk::Status status = registry_->add(std::make_unique<stream::JointAnglesStream>(
        kJointAnglesKey, session_.device(), sdk_, node_));
    if (!status) {
      return status;
    }
    if (config_.publish_emf_poses) {
      status = registry_->add(
          std::make_unique<stream::EmfPosesStream>(kEmfPosesKey, session_.device(), sdk_, node_));
      if (!status) {
        return status;
      }
    }
    if (config_.publish_tip_poses) {
      status = registry_->add(
          std::make_unique<stream::TipPosesStream>(kTipPosesKey, session_.device(), sdk_, node_));
      if (!status) {
        return status;
      }
    }
    if (config_.publish_skeleton) {
      status = registry_->add(std::make_unique<stream::SkeletonStream>(
          kHandSkeletonKey, session_.device(), sdk_, node_));
      if (!status) {
        return status;
      }
    }
    if (config_.publish_imu) {
      status = registry_->add(
          std::make_unique<stream::ImuStream>(kImuPalmKey, session_.device(), sdk_, node_));
      if (!status) {
        return status;
      }
    }
    if (config_.publish_tactile) {
      status = registry_->add(
          std::make_unique<stream::TactileStream>(kTactileKey, session_.device(), sdk_, node_));
      if (!status) {
        return status;
      }
    }
    if (config_.publish_tactile_residual) {
      status = registry_->add(std::make_unique<stream::TactileResidualStream>(
          kTactileResidualKey, session_.device(), sdk_, node_));
      if (!status) {
        return status;
      }
    }
    if (config_.publish_tactile_binary) {
      status = registry_->add(std::make_unique<stream::TactileBinaryStream>(
          kTactileBinaryKey, session_.device(), sdk_, node_));
      if (!status) {
        return status;
      }
    }
    if (config_.publish_tactile_zones) {
      status = registry_->add(std::make_unique<stream::TactileZonesStream>(
          kTactileZonesKey, session_.device(), sdk_, node_));
      if (!status) {
        return status;
      }
    }
    if (config_.publish_tactile_points) {
      status = registry_->add(std::make_unique<stream::TactilePointCloudStream>(
          kTactilePointCloudKey, session_.device(), sdk_, node_));
      if (!status) {
        return status;
      }
    }
    return sdk::Status::success();
  }

  void request_runtime_failure(std::string reason) noexcept {
    try {
      if (!exit_state_.request(EXIT_FAILURE, std::move(reason))) {
        return;
      }
      try {
        hooks_.request_shutdown();
      } catch (...) {
        RCLCPP_ERROR(node_.get_logger(), "shutdown request failed");
      }
    } catch (...) {
      RCLCPP_ERROR(node_.get_logger(), "runtime failure handling contained an exception");
    }
  }

  void on_fault_timer() noexcept {
    try {
      if (stopping_.load(std::memory_order_acquire) || exit_state_.requested()) {
        return;
      }
      for (const auto& snapshot : registry_->snapshots()) {
        if (snapshot.fault.kind == TerminalKind::None) {
          continue;
        }
        if (snapshot.policy == StreamPolicy::Required) {
          auto message = stream_fault_message(snapshot);
          RCLCPP_ERROR(node_.get_logger(), "required stream fault: %s", message.c_str());
          request_runtime_failure(std::move(message));
          return;
        }
        if (snapshot.state == StreamState::Stopping || snapshot.state == StreamState::Stopped) {
          continue;
        }
        const auto message = stream_fault_message(snapshot);
        RCLCPP_WARN(node_.get_logger(), "optional stream fault: %s", message.c_str());
        registry_->stop(snapshot.key);
      }
    } catch (...) {
      auto message = exception_message("stream fault bridge");
      RCLCPP_ERROR(node_.get_logger(), "%s", message.c_str());
      request_runtime_failure(std::move(message));
    }
  }

  GloveNode& node_;
  sdk::GloveSdkApi sdk_;
  GloveNodeHooks hooks_;
  sdk::GloveSession session_;
  DriverConfig config_;
  lifecycle::ExitState exit_state_;
  std::unique_ptr<wuji_driver_common::stream::StreamRegistry> registry_;
  rclcpp::TimerBase::SharedPtr fault_timer_;
  mutable std::mutex state_mutex_;
  State state_{State::Constructed};
  std::atomic<bool> stopping_{false};
};

GloveNode::GloveNode(const rclcpp::NodeOptions& options)
    : GloveNode(sdk::GloveSdkApi::production(), GloveNodeHooks{}, options) {}

GloveNode::GloveNode(sdk::GloveSdkApi sdk_api, GloveNodeHooks hooks,
                     const rclcpp::NodeOptions& options)
    : rclcpp::Node("wuji_glove_driver", without_parameter_endpoints(options)) {
  declare_driver_parameters(*this);
  impl_ = std::make_unique<Impl>(*this, std::move(sdk_api), std::move(hooks));
}

GloveNode::~GloveNode() noexcept {
  stop();
}

sdk::Status GloveNode::start() {
  return impl_->start();
}

void GloveNode::stop() noexcept {
  if (impl_) {
    impl_->stop();
  }
}

bool GloveNode::exit_requested() const noexcept {
  return impl_->exit_requested();
}

int GloveNode::exit_code() const noexcept {
  return impl_->exit_code();
}

std::string GloveNode::exit_reason() const {
  return impl_->exit_reason();
}

}  // namespace wuji_glove_driver
