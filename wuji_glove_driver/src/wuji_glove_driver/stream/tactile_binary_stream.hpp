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

#ifndef WUJI_GLOVE_DRIVER__STREAM__TACTILE_BINARY_STREAM_HPP_
#define WUJI_GLOVE_DRIVER__STREAM__TACTILE_BINARY_STREAM_HPP_

#include <wuji_sdk.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <wuji_msgs/msg/tactile_grid.hpp>
#include "wuji_driver_common/stream/status.hpp"
#include "wuji_driver_common/stream/stream_controller.hpp"
#include "wuji_driver_common/stream/stream_types.hpp"
#include "wuji_glove_driver/sdk/glove_sdk_api.hpp"

namespace wuji_glove_driver::stream {
class TactileBinaryStream final : public wuji_driver_common::stream::StreamController {
 public:
  TactileBinaryStream(wuji_driver_common::stream::StreamKey key, WujiDevice* device,
                      sdk::GloveSdkApi& sdk, rclcpp::Node& node) noexcept;

 protected:
  wuji_driver_common::stream::Status prepare_impl() override;
  wuji_driver_common::stream::Status subscribe_impl(WujiSub** output) override;
  void unsubscribe_impl(WujiSub* subscription) noexcept override;
  void cleanup_impl() noexcept override;

 private:
  static void sdk_callback(WujiFrameKind kind, const WujiTactileBinary* frame,
                           void* user_data) noexcept;
  void publish_frame(const WujiTactileBinary& frame);
  sdk::GloveSdkApi& sdk_;
  rclcpp::Node& node_;
  rclcpp::Publisher<wuji_msgs::msg::TactileGrid>::SharedPtr publisher_;
};
}  // namespace wuji_glove_driver::stream
#endif  // WUJI_GLOVE_DRIVER__STREAM__TACTILE_BINARY_STREAM_HPP_
