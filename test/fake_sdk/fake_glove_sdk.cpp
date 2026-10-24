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

#include "fake_sdk/fake_glove_sdk.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <utility>

struct WujiDevice {
  std::uint64_t token;
  std::uint8_t target_kind;
  std::string target_value;
};

struct WujiSub {
  std::uint8_t stream;
};

namespace wuji_glove_driver::test {

FakeGloveSdk* FakeGloveSdk::current_{nullptr};
thread_local std::string FakeGloveSdk::last_error_;

FakeGloveSdk::FakeGloveSdk() {
  if (current_ != nullptr) {
    throw std::logic_error("only one FakeGloveSdk may exist at a time");
  }
  current_ = this;
  api_.init = &init;
  api_.shutdown = &shutdown;
  api_.last_error = &last_error;
  api_.connect_options_default = &connect_options_default;
  api_.scan = &scan;
  api_.discovered_free = &discovered_free;
  api_.connect = &connect;
  api_.disconnect = &disconnect;
  api_.release = &release;
  api_.device_serial_number = &device_serial_number;
  api_.device_model = &device_model;
  api_.device_is_connected = &device_is_connected;
  api_.get_hand_side = &get_hand_side;
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

FakeGloveSdk::~FakeGloveSdk() {
  current_ = nullptr;
}

sdk::GloveSdkApi& FakeGloveSdk::api() noexcept {
  return api_;
}

const std::vector<std::string>& FakeGloveSdk::calls() const noexcept {
  return calls_;
}

std::size_t FakeGloveSdk::count(std::string_view operation) const {
  return static_cast<std::size_t>(std::count(calls_.begin(), calls_.end(), operation));
}

const std::vector<FakeGloveSdk::ConnectCall>& FakeGloveSdk::connect_calls() const noexcept {
  return connect_calls_;
}

const std::vector<FakeGloveSdk::TypedStream>& FakeGloveSdk::subscribed_streams() const noexcept {
  return subscribed_streams_;
}

const std::vector<FakeGloveSdk::TypedStream>& FakeGloveSdk::closed_streams() const noexcept {
  return closed_streams_;
}

const char* FakeGloveSdk::stream_name(TypedStream stream) noexcept {
  switch (stream) {
    case TypedStream::JointAngles:
      return "joint_angles";
    case TypedStream::EmfPoses:
      return "emf_poses";
    case TypedStream::TipPoses:
      return "tip_poses";
    case TypedStream::HandSkeleton:
      return "hand_skeleton";
    case TypedStream::ImuPalm:
      return "imu_palm";
    case TypedStream::Tactile:
      return "tactile";
    case TypedStream::TactileResidual:
      return "tactile_residual";
    case TypedStream::TactileBinary:
      return "tactile_binary";
    case TypedStream::TactileZones:
      return "tactile_zones";
    case TypedStream::TactilePointCloud:
      return "tactile_point_cloud";
  }
  return "unknown";
}

void FakeGloveSdk::set_scan_results_sequence(std::vector<std::vector<Discovery>> results) {
  scan_results_sequence_ = std::move(results);
  scan_index_ = 0;
}

void FakeGloveSdk::set_scan_failure(WujiStatus status, std::string error) {
  scan_failure_ = Failure{status, std::move(error), false};
}

void FakeGloveSdk::set_init_failure(WujiStatus status, std::string error) {
  init_failure_ = Failure{status, std::move(error), false};
}

void FakeGloveSdk::set_connect_failure(WujiStatus status, std::string error, bool return_device) {
  connect_failure_ = Failure{status, std::move(error), return_device};
}

void FakeGloveSdk::set_disconnect_failure(WujiStatus status, std::string error) {
  disconnect_failure_ = Failure{status, std::move(error), false};
}

void FakeGloveSdk::set_serial_number(std::string value) {
  serial_number_ = std::move(value);
}

void FakeGloveSdk::set_serial_number_failure(WujiStatus status, std::string error) {
  serial_failure_ = Failure{status, std::move(error), false};
}

void FakeGloveSdk::set_model(std::string value) {
  model_ = std::move(value);
}

void FakeGloveSdk::set_model_failure(WujiStatus status, std::string error) {
  model_failure_ = Failure{status, std::move(error), false};
}

void FakeGloveSdk::set_hand_side(std::string value) {
  hand_side_ = std::move(value);
}

void FakeGloveSdk::set_hand_side_failure(WujiStatus status, std::string error) {
  hand_side_failure_ = Failure{status, std::move(error), false};
}

void FakeGloveSdk::set_connected(bool value) noexcept {
  connected_ = value;
}

void FakeGloveSdk::set_identity_from_connect_target(bool enabled) noexcept {
  identity_from_connect_target_ = enabled;
}

void FakeGloveSdk::set_call_hook(std::function<void(std::string_view)> hook) {
  call_hook_ = std::move(hook);
}

void FakeGloveSdk::set_subscribe_failure(TypedStream stream, WujiStatus status, std::string error,
                                         bool return_subscription) {
  subscribe_failures_[static_cast<std::size_t>(stream)] =
      Failure{status, std::move(error), return_subscription};
}

void FakeGloveSdk::emit(TypedStream stream, WujiFrameKind kind) {
  const auto& emitter = emitters_[static_cast<std::size_t>(stream)];
  if (emitter) {
    emitter(kind);
  }
}

FakeGloveSdk& FakeGloveSdk::current() {
  return *current_;
}

void FakeGloveSdk::record(std::string operation) {
  auto& fake = current();
  fake.calls_.push_back(operation);
  last_error_.clear();
  if (fake.call_hook_) {
    fake.call_hook_(operation);
  }
}

WujiStatus FakeGloveSdk::finish(const Failure& failure) {
  if (failure.status != WUJI_STATUS_OK) {
    last_error_ = failure.error;
  }
  return failure.status;
}

std::int32_t FakeGloveSdk::copy_device_string(std::string_view operation, const std::string& value,
                                              const Failure& failure, char* buffer,
                                              std::size_t buffer_length) {
  record(std::string{operation});
  if (failure.status != WUJI_STATUS_OK) {
    return static_cast<std::int32_t>(finish(failure));
  }
  if (buffer_length == 0) {
    return static_cast<std::int32_t>(value.size());
  }
  const auto copied = std::min(value.size(), buffer_length - 1);
  std::memcpy(buffer, value.data(), copied);
  buffer[copied] = '\0';
  return static_cast<std::int32_t>(copied);
}

WujiStatus FakeGloveSdk::init(const WujiInitOptions*) {
  record("init");
  return finish(current().init_failure_);
}

void FakeGloveSdk::shutdown() {
  record("shutdown");
}

const char* FakeGloveSdk::last_error() {
  current().calls_.push_back("last_error");
  return last_error_.empty() ? nullptr : last_error_.c_str();
}

WujiConnectOptions FakeGloveSdk::connect_options_default() {
  record("connect_options_default");
  WujiConnectOptions options{};
  options.timeout_ms = 1000;
  options.retry_count = 3;
  options.enable_bridge = true;
  return options;
}

WujiStatus FakeGloveSdk::scan(WujiDiscovered** output, std::size_t* output_size) {
  record("scan");
  if (output == nullptr || output_size == nullptr) {
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  *output = nullptr;
  *output_size = 0;
  auto& fake = current();
  if (fake.scan_failure_.status != WUJI_STATUS_OK) {
    return finish(fake.scan_failure_);
  }
  if (fake.scan_results_sequence_.empty()) {
    return WUJI_STATUS_OK;
  }
  const auto index = std::min(fake.scan_index_, fake.scan_results_sequence_.size() - 1);
  const auto& discoveries = fake.scan_results_sequence_[index];
  ++fake.scan_index_;
  if (discoveries.empty()) {
    return WUJI_STATUS_OK;
  }
  auto* result = new WujiDiscovered[discoveries.size()]{};
  for (std::size_t item = 0; item < discoveries.size(); ++item) {
    const auto& source = discoveries[item];
    std::snprintf(result[item].serial_number, sizeof(result[item].serial_number), "%s",
                  source.serial_number.c_str());
    std::snprintf(result[item].model, sizeof(result[item].model), "%s", source.model.c_str());
    result[item].device_id = source.device_id;
    result[item].transport = source.transport;
    std::snprintf(result[item].address, sizeof(result[item].address), "%s", source.address.c_str());
  }
  *output = result;
  *output_size = discoveries.size();
  return WUJI_STATUS_OK;
}

void FakeGloveSdk::discovered_free(WujiDiscovered* discovered, std::size_t) {
  record("discovered_free");
  delete[] discovered;
}

WujiStatus FakeGloveSdk::connect(const WujiConnectTarget* target, const char*,
                                 const WujiConnectOptions* options, WujiDevice** output) {
  record("connect");
  auto& fake = current();
  fake.connect_calls_.push_back(
      ConnectCall{target == nullptr ? std::uint8_t{255} : target->kind,
                  target == nullptr || target->value == nullptr ? std::string{} : target->value,
                  options == nullptr ? WujiConnectOptions{} : *options});
  if (output == nullptr) {
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  *output = nullptr;
  if (fake.connect_failure_.status == WUJI_STATUS_OK || fake.connect_failure_.return_handle) {
    *output = new WujiDevice{fake.next_token_++, fake.connect_calls_.back().target_kind,
                             fake.connect_calls_.back().target_value};
  }
  return finish(fake.connect_failure_);
}

WujiStatus FakeGloveSdk::disconnect(WujiDevice*) {
  record("disconnect");
  return finish(current().disconnect_failure_);
}

void FakeGloveSdk::release(WujiDevice* device) {
  record("release");
  delete device;
}

std::int32_t FakeGloveSdk::device_serial_number(WujiDevice* device, char* buffer,
                                                std::size_t buffer_length) {
  auto& fake = current();
  const bool use_target = fake.identity_from_connect_target_ && device != nullptr &&
                          device->target_kind == WUJI_CONNECT_TARGET_KIND_SN;
  const std::string& value = use_target ? device->target_value : fake.serial_number_;
  return copy_device_string("device_serial_number", value, fake.serial_failure_, buffer,
                            buffer_length);
}

std::int32_t FakeGloveSdk::device_model(WujiDevice*, char* buffer, std::size_t buffer_length) {
  auto& fake = current();
  return copy_device_string("device_model", fake.model_, fake.model_failure_, buffer,
                            buffer_length);
}

std::uint8_t FakeGloveSdk::device_is_connected(WujiDevice*) {
  record("device_is_connected");
  return current().connected_ ? std::uint8_t{1} : std::uint8_t{0};
}

WujiStatus FakeGloveSdk::get_hand_side(WujiDevice*, char* buffer, std::size_t buffer_length,
                                       std::size_t* needed) {
  record("get_hand_side");
  auto& fake = current();
  if (fake.hand_side_failure_.status != WUJI_STATUS_OK) {
    return finish(fake.hand_side_failure_);
  }
  const std::size_t required = fake.hand_side_.size() + 1;
  if (needed != nullptr) {
    *needed = required;
  }
  if (buffer == nullptr || buffer_length < required) {
    return WUJI_STATUS_ERR_BUFFER_TOO_SMALL;
  }
  std::memcpy(buffer, fake.hand_side_.c_str(), required);
  return WUJI_STATUS_OK;
}

void FakeGloveSdk::sub_close(WujiSub* subscription) {
  record("sub_close");
  if (subscription == nullptr) {
    return;
  }
  auto& fake = current();
  const auto stream = static_cast<TypedStream>(subscription->stream);
  fake.closed_streams_.push_back(stream);
  fake.emitters_[static_cast<std::size_t>(stream)] = {};
  record(std::string{"close:"} + stream_name(stream));
  delete subscription;
}

template <typename Callback>
WujiStatus FakeGloveSdk::subscribe_typed(TypedStream stream, std::string operation,
                                         Callback callback, void* user_data, WujiSub** output) {
  record(std::move(operation));
  if (output == nullptr) {
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  *output = nullptr;
  auto& fake = current();
  const auto index = static_cast<std::size_t>(stream);
  const Failure& failure = fake.subscribe_failures_[index];
  if (failure.status == WUJI_STATUS_OK || failure.return_handle) {
    *output = new WujiSub{static_cast<std::uint8_t>(stream)};
    fake.subscribed_streams_.push_back(stream);
    fake.emitters_[index] = [callback, user_data](WujiFrameKind kind) {
      callback(kind, nullptr, user_data);
    };
  }
  return finish(failure);
}

#define WUJI_FAKE_SUBSCRIBE(NAME, STREAM, CALLBACK)                                  \
  WujiStatus FakeGloveSdk::NAME(WujiDevice*, CALLBACK callback, void* user_data,     \
                                WujiSub** output) {                                  \
    return subscribe_typed(TypedStream::STREAM, #NAME, callback, user_data, output); \
  }

WUJI_FAKE_SUBSCRIBE(subscribe_joint_angles, JointAngles, WujiHandJointAnglesSubCallback)
WUJI_FAKE_SUBSCRIBE(subscribe_emf_poses, EmfPoses, WujiEmfPoseArraySubCallback)
WUJI_FAKE_SUBSCRIBE(subscribe_tip_poses, TipPoses, WujiFingertipPosesSubCallback)
WUJI_FAKE_SUBSCRIBE(subscribe_hand_skeleton, HandSkeleton, WujiHandSkeletonSubCallback)
WUJI_FAKE_SUBSCRIBE(subscribe_imu_palm, ImuPalm, WujiImuDataSubCallback)
WUJI_FAKE_SUBSCRIBE(subscribe_tactile, Tactile, WujiTactileFrameSubCallback)
WUJI_FAKE_SUBSCRIBE(subscribe_tactile_residual, TactileResidual, WujiTactileResidualSubCallback)
WUJI_FAKE_SUBSCRIBE(subscribe_tactile_binary, TactileBinary, WujiTactileBinarySubCallback)
WUJI_FAKE_SUBSCRIBE(subscribe_tactile_zones, TactileZones, WujiTactileZonesSubCallback)
WUJI_FAKE_SUBSCRIBE(subscribe_tactile_point_cloud, TactilePointCloud, WujiPointCloudSubCallback)

#undef WUJI_FAKE_SUBSCRIBE

}  // namespace wuji_glove_driver::test
