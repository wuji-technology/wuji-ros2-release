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

#ifndef FAKE_SDK__FAKE_GLOVE_SDK_HPP_
#define FAKE_SDK__FAKE_GLOVE_SDK_HPP_

#include <wuji_sdk.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "wuji_glove_driver/sdk/glove_sdk_api.hpp"

namespace wuji_glove_driver::test {

class FakeGloveSdk {
 public:
  enum class TypedStream : std::uint8_t {
    JointAngles,
    EmfPoses,
    TipPoses,
    HandSkeleton,
    ImuPalm,
    Tactile,
    TactileResidual,
    TactileBinary,
    TactileZones,
    TactilePointCloud,
  };

  struct Discovery {
    std::string serial_number;
    std::string model{"WujiGlove"};
    WujiDeviceType device_id{WUJI_DEVICE_TYPE_WUJI_GLOVE};
    WujiTransportType transport{WUJI_TRANSPORT_TYPE_UDP};
    std::string address;
  };

  struct ConnectCall {
    std::uint8_t target_kind;
    std::string target_value;
    WujiConnectOptions options;
  };

  FakeGloveSdk();
  ~FakeGloveSdk();

  FakeGloveSdk(const FakeGloveSdk&) = delete;
  FakeGloveSdk& operator=(const FakeGloveSdk&) = delete;

  sdk::GloveSdkApi& api() noexcept;
  const std::vector<std::string>& calls() const noexcept;
  std::size_t count(std::string_view operation) const;
  const std::vector<ConnectCall>& connect_calls() const noexcept;
  const std::vector<TypedStream>& subscribed_streams() const noexcept;
  const std::vector<TypedStream>& closed_streams() const noexcept;
  static const char* stream_name(TypedStream stream) noexcept;

  void set_scan_results_sequence(std::vector<std::vector<Discovery>> results);
  void set_scan_failure(WujiStatus status, std::string error);
  void set_init_failure(WujiStatus status, std::string error);
  void set_connect_failure(WujiStatus status, std::string error, bool return_device = false);
  void set_disconnect_failure(WujiStatus status, std::string error);
  void set_serial_number(std::string value);
  void set_serial_number_failure(WujiStatus status, std::string error);
  void set_model(std::string value);
  void set_model_failure(WujiStatus status, std::string error);
  void set_hand_side(std::string value);
  void set_hand_side_failure(WujiStatus status, std::string error);
  void set_connected(bool value) noexcept;
  void set_identity_from_connect_target(bool enabled) noexcept;
  void set_call_hook(std::function<void(std::string_view)> hook);
  void set_subscribe_failure(TypedStream stream, WujiStatus status, std::string error,
                             bool return_subscription = false);
  void emit(TypedStream stream, WujiFrameKind kind);

 private:
  struct Failure {
    WujiStatus status{WUJI_STATUS_OK};
    std::string error;
    bool return_handle{false};
  };

  static FakeGloveSdk& current();
  static void record(std::string operation);
  static WujiStatus finish(const Failure& failure);
  static std::int32_t copy_device_string(std::string_view operation, const std::string& value,
                                         const Failure& failure, char* buffer,
                                         std::size_t buffer_length);

  static WujiStatus init(const WujiInitOptions* options);
  static void shutdown();
  static const char* last_error();
  static WujiConnectOptions connect_options_default();
  static WujiStatus scan(WujiDiscovered** output, std::size_t* output_size);
  static void discovered_free(WujiDiscovered* discovered, std::size_t size);
  static WujiStatus connect(const WujiConnectTarget* target, const char* alias,
                            const WujiConnectOptions* options, WujiDevice** output);
  static WujiStatus disconnect(WujiDevice* device);
  static void release(WujiDevice* device);
  static std::int32_t device_serial_number(WujiDevice* device, char* buffer,
                                           std::size_t buffer_length);
  static std::int32_t device_model(WujiDevice* device, char* buffer, std::size_t buffer_length);
  static std::uint8_t device_is_connected(WujiDevice* device);
  static WujiStatus get_hand_side(WujiDevice* device, char* buffer, std::size_t buffer_length,
                                  std::size_t* needed);
  static void sub_close(WujiSub* subscription);

  template <typename Callback>
  static WujiStatus subscribe_typed(TypedStream stream, std::string operation, Callback callback,
                                    void* user_data, WujiSub** output);
  static WujiStatus subscribe_joint_angles(WujiDevice*, WujiHandJointAnglesSubCallback callback,
                                           void* user_data, WujiSub** output);
  static WujiStatus subscribe_emf_poses(WujiDevice*, WujiEmfPoseArraySubCallback callback,
                                        void* user_data, WujiSub** output);
  static WujiStatus subscribe_tip_poses(WujiDevice*, WujiFingertipPosesSubCallback callback,
                                        void* user_data, WujiSub** output);
  static WujiStatus subscribe_hand_skeleton(WujiDevice*, WujiHandSkeletonSubCallback callback,
                                            void* user_data, WujiSub** output);
  static WujiStatus subscribe_imu_palm(WujiDevice*, WujiImuDataSubCallback callback,
                                       void* user_data, WujiSub** output);
  static WujiStatus subscribe_tactile(WujiDevice*, WujiTactileFrameSubCallback callback,
                                      void* user_data, WujiSub** output);
  static WujiStatus subscribe_tactile_residual(WujiDevice*, WujiTactileResidualSubCallback callback,
                                               void* user_data, WujiSub** output);
  static WujiStatus subscribe_tactile_binary(WujiDevice*, WujiTactileBinarySubCallback callback,
                                             void* user_data, WujiSub** output);
  static WujiStatus subscribe_tactile_zones(WujiDevice*, WujiTactileZonesSubCallback callback,
                                            void* user_data, WujiSub** output);
  static WujiStatus subscribe_tactile_point_cloud(WujiDevice*, WujiPointCloudSubCallback callback,
                                                  void* user_data, WujiSub** output);

  static FakeGloveSdk* current_;
  static thread_local std::string last_error_;

  sdk::GloveSdkApi api_{};
  std::vector<std::string> calls_;
  std::vector<ConnectCall> connect_calls_;
  std::vector<TypedStream> subscribed_streams_;
  std::vector<TypedStream> closed_streams_;
  std::vector<std::vector<Discovery>> scan_results_sequence_;
  std::size_t scan_index_{0};
  Failure init_failure_;
  Failure scan_failure_;
  Failure connect_failure_;
  Failure disconnect_failure_;
  Failure serial_failure_;
  Failure model_failure_;
  Failure hand_side_failure_;
  std::string serial_number_{"WGLJ000000000001"};
  std::string model_{"0x0001"};
  std::string hand_side_{"left"};
  bool connected_{true};
  bool identity_from_connect_target_{true};
  std::uint64_t next_token_{1};
  std::function<void(std::string_view)> call_hook_;
  std::array<Failure, 10> subscribe_failures_{};
  std::array<std::function<void(WujiFrameKind)>, 10> emitters_{};
};

}  // namespace wuji_glove_driver::test

#endif  // FAKE_SDK__FAKE_GLOVE_SDK_HPP_
