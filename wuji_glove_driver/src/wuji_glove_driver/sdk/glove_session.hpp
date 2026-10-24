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

#ifndef WUJI_GLOVE_DRIVER__SDK__GLOVE_SESSION_HPP_
#define WUJI_GLOVE_DRIVER__SDK__GLOVE_SESSION_HPP_

#include <wuji_sdk.h>

#include <chrono>
#include <functional>
#include <optional>
#include <string>

#include "wuji_glove_driver/sdk/glove_sdk_api.hpp"
#include "wuji_glove_driver/sdk/sdk_status.hpp"

namespace wuji_glove_driver::sdk {

struct GloveConnectConfig {
  std::string endpoint;
  std::string serial_number;
  std::string hand_side;
  double connect_timeout{10.0};
};

struct GloveSessionHooks {
  std::function<std::chrono::steady_clock::time_point()> now;
  std::function<void(std::chrono::steady_clock::duration)> sleep_for;
  std::function<bool()> cancelled;
};

class GloveSession {
 public:
  explicit GloveSession(GloveSdkApi& sdk, GloveSessionHooks hooks = {});
  ~GloveSession() noexcept;

  GloveSession(const GloveSession&) = delete;
  GloveSession& operator=(const GloveSession&) = delete;

  Status connect(const GloveConnectConfig& config);
  WujiDevice* device() const noexcept;
  std::string serial_number() const;
  std::string model() const;
  std::string hand_side() const;
  void disconnect() noexcept;

 private:
  using TimePoint = std::chrono::steady_clock::time_point;

  Status acquire_runtime();
  void release_runtime() noexcept;
  Status check_wait_state(const std::optional<TimePoint>& deadline) const;
  Status read_device_string(decltype(&wuji_dev_serial_number) getter, const char* operation,
                            std::string& output, const std::optional<TimePoint>& deadline);
  Status read_hand_side(std::string& output, const std::optional<TimePoint>& deadline);

  GloveSdkApi& sdk_;
  GloveSessionHooks hooks_;
  WujiDevice* device_{nullptr};
  std::string serial_number_;
  std::string model_;
  std::string hand_side_;
  bool runtime_acquired_{false};
};

}  // namespace wuji_glove_driver::sdk

#endif  // WUJI_GLOVE_DRIVER__SDK__GLOVE_SESSION_HPP_
