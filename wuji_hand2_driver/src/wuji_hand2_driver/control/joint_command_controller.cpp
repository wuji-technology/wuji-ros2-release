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

#include "wuji_hand2_driver/control/joint_command_controller.hpp"

#include <array>
#include <exception>
#include <string>
#include <utility>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>

#include "wuji_hand2_driver/conversion/joint_command_conversion.hpp"

namespace wuji_hand2_driver::control {

class JointCommandController::CallbackState {
 public:
  explicit CallbackState(const OperationContext& context) : context_(context) {}

  void receive(const sensor_msgs::msg::JointState& message) {
    if (!context_.gate.accepting()) {
      report_failure("control operations are stopping");
      return;
    }

    std::array<WujiJointCommand, conversion::JointMapping::kJointCount> commands{};
    sdk::Status status = conversion::to_joint_commands(message, context_.mapping, commands);
    if (!status) {
      report_failure(status.message);
      return;
    }

    status = context_.session.send(commands);
    if (!status) {
      report_failure(status.message);
    }
  }

 private:
  void report_failure(const std::string& message) {
    RCLCPP_ERROR_THROTTLE(context_.logger, *context_.node.get_clock(), 1000,
                          "joint command rejected: %s", message.c_str());
  }

  const OperationContext context_;
};

JointCommandController::JointCommandController(const OperationContext& context)
    : context_(context), callback_state_(std::make_shared<CallbackState>(context)) {}

JointCommandController::~JointCommandController() noexcept {
  stop();
}

sdk::Status JointCommandController::start(const rclcpp::CallbackGroup::SharedPtr& command_group) {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (subscription_ != nullptr) {
    return sdk::Status::success();
  }
  if (command_group == nullptr) {
    return sdk::Status::failure("joint command callback group is null");
  }
  if (!context_.gate.accepting()) {
    return sdk::Status::failure("control operations are stopping");
  }

  try {
    rclcpp::SubscriptionOptions options;
    options.callback_group = command_group;
    const auto state = callback_state_;
    subscription_ = context_.node.create_subscription<sensor_msgs::msg::JointState>(
        "joint_commands", rclcpp::SensorDataQoS().keep_last(1),
        [state](sensor_msgs::msg::JointState::ConstSharedPtr message) { state->receive(*message); },
        options);
    if (subscription_ == nullptr) {
      return sdk::Status::failure("joint command subscription creation returned null");
    }
    if (!context_.gate.accepting()) {
      subscription_.reset();
      return sdk::Status::failure("control operations are stopping");
    }
    return sdk::Status::success();
  } catch (const std::exception& error) {
    subscription_.reset();
    return sdk::Status::failure(error.what());
  } catch (...) {
    subscription_.reset();
    return sdk::Status::failure("joint command subscription creation failed");
  }
}

void JointCommandController::stop() noexcept {
  context_.gate.stop();
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  subscription_.reset();
}

}  // namespace wuji_hand2_driver::control
