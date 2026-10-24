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

#include "wuji_glove_driver/stream/skeleton_stream.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include "wuji_glove_driver/conversion/pose_conversion.hpp"
#include "wuji_glove_driver/sdk/sdk_status.hpp"

namespace wuji_glove_driver::stream {

using Status = wuji_driver_common::stream::Status;
using StreamPolicy = wuji_driver_common::stream::StreamPolicy;

SkeletonStream::SkeletonStream(wuji_driver_common::stream::StreamKey key, WujiDevice* device,
                               sdk::GloveSdkApi& sdk, rclcpp::Node& node) noexcept
    : StreamController(key, device, StreamPolicy::Optional), sdk_(sdk), node_(node) {}

Status SkeletonStream::prepare_impl() {
  publisher_ = node_.create_publisher<wuji_msgs::msg::TrackedPoseArray>(
      "hand_skeleton", rclcpp::SensorDataQoS().keep_last(5).best_effort().durability_volatile());
  return Status::success();
}

Status SkeletonStream::subscribe_impl(WujiSub** output) {
  const WujiStatus status =
      sdk_.subscribe_hand_skeleton(device(), &SkeletonStream::sdk_callback, this, output);
  return sdk::sdk_status(sdk_, "wuji_glove_subscribe_hand_skeleton", status);
}

void SkeletonStream::unsubscribe_impl(WujiSub* subscription) noexcept {
  sdk_.sub_close(subscription);
}

void SkeletonStream::cleanup_impl() noexcept {
  publisher_.reset();
}

void SkeletonStream::sdk_callback(WujiFrameKind kind, const WujiHandSkeleton* frame,
                                  void* user_data) noexcept {
  if (user_data == nullptr) {
    return;
  }
  auto* const self = static_cast<SkeletonStream*>(user_data);
  self->dispatch(kind, frame,
                 [self](const WujiHandSkeleton& received) { self->publish_frame(received); });
}

void SkeletonStream::publish_frame(const WujiHandSkeleton& frame) {
  wuji_msgs::msg::TrackedPoseArray message;
  const Status status = conversion::to_hand_skeleton(frame, message);
  if (!status) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "Dropping malformed hand_skeleton frame: %s", status.message.c_str());
    return;
  }
  publisher_->publish(message);
}

}  // namespace wuji_glove_driver::stream
