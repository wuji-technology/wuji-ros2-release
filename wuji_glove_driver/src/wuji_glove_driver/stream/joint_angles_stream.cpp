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

#include "wuji_glove_driver/stream/joint_angles_stream.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include "wuji_glove_driver/conversion/joint_angles_conversion.hpp"
#include "wuji_glove_driver/sdk/sdk_status.hpp"

namespace wuji_glove_driver::stream {

using Status = wuji_driver_common::stream::Status;
using StreamPolicy = wuji_driver_common::stream::StreamPolicy;

JointAnglesStream::JointAnglesStream(wuji_driver_common::stream::StreamKey key, WujiDevice* device,
                                     sdk::GloveSdkApi& sdk, rclcpp::Node& node) noexcept
    : StreamController(key, device, StreamPolicy::Required), sdk_(sdk), node_(node) {}

Status JointAnglesStream::prepare_impl() {
  publisher_ = node_.create_publisher<wuji_msgs::msg::HandJointAngles>(
      "joint_angles", rclcpp::SensorDataQoS().keep_last(5).best_effort().durability_volatile());
  return Status::success();
}

Status JointAnglesStream::subscribe_impl(WujiSub** output) {
  const WujiStatus status =
      sdk_.subscribe_joint_angles(device(), &JointAnglesStream::sdk_callback, this, output);
  return sdk::sdk_status(sdk_, "wuji_glove_subscribe_hand_joint_angles", status);
}

void JointAnglesStream::unsubscribe_impl(WujiSub* subscription) noexcept {
  sdk_.sub_close(subscription);
}

void JointAnglesStream::cleanup_impl() noexcept {
  publisher_.reset();
}

void JointAnglesStream::sdk_callback(WujiFrameKind kind, const WujiHandJointAngles* frame,
                                     void* user_data) noexcept {
  if (user_data == nullptr) {
    return;
  }
  auto* const self = static_cast<JointAnglesStream*>(user_data);
  self->dispatch(kind, frame,
                 [self](const WujiHandJointAngles& received) { self->publish_frame(received); });
}

void JointAnglesStream::publish_frame(const WujiHandJointAngles& frame) {
  wuji_msgs::msg::HandJointAngles message;
  const Status status = conversion::to_hand_joint_angles(frame, message);
  if (!status) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "Dropping malformed joint_angles frame: %s", status.message.c_str());
    return;
  }
  publisher_->publish(message);
}

}  // namespace wuji_glove_driver::stream
