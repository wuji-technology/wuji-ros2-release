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

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <utility>
#include <vector>

#include "read_only_stream_probe.hpp"

#include "wuji_driver_common/stream/status.hpp"

namespace wuji_glove_driver::hardware_test {
namespace {

struct TestPayload {
  bool convertible{true};
};

struct TestMessage {};

struct TestConverter {
  auto operator()(const TestPayload& input, TestMessage&) const {
    if (input.convertible) {
      return wuji_driver_common::stream::Status::success();
    }
    return wuji_driver_common::stream::Status::failure("injected malformed payload");
  }
};

using TestProbe = StreamProbe<TestPayload, TestMessage, TestConverter>;

struct FailedSubscriptionState {
  using Callback = void (*)(WujiFrameKind, const TestPayload*, void*);

  WujiStatus subscribe_status{WUJI_STATUS_ERR_INTERNAL};
  std::string last_error{"subscribe detail"};
  Callback callback{nullptr};
  void* user_data{nullptr};
  int close_calls{0};
  bool probe_alive{true};
  bool callback_ran_while_probe_alive{false};
  std::vector<std::string> events;
};

FailedSubscriptionState* failed_subscription_state = nullptr;

class TrackedTestProbe : public TestProbe {
 public:
  using TestProbe::TestProbe;
  ~TrackedTestProbe() {
    failed_subscription_state->probe_alive = false;
    failed_subscription_state->events.emplace_back("probe destroyed");
  }
};

class DisconnectRecorder {
 public:
  explicit DisconnectRecorder(FailedSubscriptionState& state) : state_(state) {}
  ~DisconnectRecorder() {
    state_.events.emplace_back("device disconnected");
  }

 private:
  FailedSubscriptionState& state_;
};

const char* failed_last_error() {
  return failed_subscription_state->last_error.c_str();
}

WujiStatus subscribe_with_failed_handle(WujiDevice*,
                                        void (*callback)(WujiFrameKind, const TestPayload*, void*),
                                        void* user_data, WujiSub** output) {
  failed_subscription_state->events.emplace_back("subscribe failed");
  failed_subscription_state->callback = callback;
  failed_subscription_state->user_data = user_data;
  *output = reinterpret_cast<WujiSub*>(1);
  return failed_subscription_state->subscribe_status;
}

void close_failed_handle(WujiSub*) {
  ++failed_subscription_state->close_calls;
  failed_subscription_state->events.emplace_back("subscription closed");
  failed_subscription_state->callback_ran_while_probe_alive =
      failed_subscription_state->probe_alive;
  failed_subscription_state->callback(WUJI_FRAME_KIND_END, nullptr,
                                      failed_subscription_state->user_data);
  failed_subscription_state->callback = nullptr;
  failed_subscription_state->user_data = nullptr;
  failed_subscription_state->last_error = "close changed last error";
}

TestProbe* close_probe = nullptr;
WujiFrameKind close_kind = WUJI_FRAME_KIND_END;

void close_with_terminal(WujiSub*) {
  TestProbe::callback(close_kind, nullptr, close_probe);
}

TEST(ReadOnlyStreamProbe, IgnoresOnlyFinalEndDeliveredSynchronouslyByClose) {
  std::atomic_bool cancelled{false};
  TestProbe probe{"test", cancelled};
  TestPayload payload;
  TestProbe::callback(WUJI_FRAME_KIND_OK, &payload, &probe);

  std::string error;
  ASSERT_TRUE(probe.snapshot_success(error)) << error;
  sdk::GloveSdkApi api{};
  api.sub_close = &close_with_terminal;
  SubscriptionSet subscriptions{api};
  subscriptions.add(reinterpret_cast<WujiSub*>(1), probe);
  close_probe = &probe;
  close_kind = WUJI_FRAME_KIND_END;
  subscriptions.close_all();

  EXPECT_TRUE(probe.check_after_close(error)) << error;
}

TEST(ReadOnlyStreamProbe, RejectsEndBeforeIntentionalClose) {
  std::atomic_bool cancelled{false};
  TestProbe probe{"test", cancelled};
  TestPayload payload;
  TestProbe::callback(WUJI_FRAME_KIND_OK, &payload, &probe);
  TestProbe::callback(WUJI_FRAME_KIND_END, nullptr, &probe);

  std::string error;
  EXPECT_FALSE(probe.snapshot_success(error));
  EXPECT_NE(error.find("terminal frame kind"), std::string::npos);
}

TEST(ReadOnlyStreamProbe, RejectsErrorDeliveredDuringIntentionalClose) {
  std::atomic_bool cancelled{false};
  TestProbe probe{"test", cancelled};
  TestPayload payload;
  TestProbe::callback(WUJI_FRAME_KIND_OK, &payload, &probe);

  std::string error;
  ASSERT_TRUE(probe.snapshot_success(error)) << error;
  sdk::GloveSdkApi api{};
  api.sub_close = &close_with_terminal;
  SubscriptionSet subscriptions{api};
  subscriptions.add(reinterpret_cast<WujiSub*>(1), probe);
  close_probe = &probe;
  close_kind = WUJI_FRAME_KIND_ERROR;
  subscriptions.close_all();

  EXPECT_FALSE(probe.check_after_close(error));
  EXPECT_NE(error.find("terminal frame kind"), std::string::npos);
}

TEST(ReadOnlyStreamProbe, RejectsUnexpectedNullOkDeliveredDuringIntentionalClose) {
  std::atomic_bool cancelled{false};
  TestProbe probe{"test", cancelled};
  TestPayload payload;
  TestProbe::callback(WUJI_FRAME_KIND_OK, &payload, &probe);

  std::string error;
  ASSERT_TRUE(probe.snapshot_success(error)) << error;
  sdk::GloveSdkApi api{};
  api.sub_close = &close_with_terminal;
  SubscriptionSet subscriptions{api};
  subscriptions.add(reinterpret_cast<WujiSub*>(1), probe);
  close_probe = &probe;
  close_kind = WUJI_FRAME_KIND_OK;
  subscriptions.close_all();

  EXPECT_FALSE(probe.check_after_close(error));
  EXPECT_NE(error.find("null payload"), std::string::npos);
}

TEST(ReadOnlyStreamStart, ClosesFailedOptionalHandleBeforeProbeAndDisconnect) {
  FailedSubscriptionState state;
  state.subscribe_status = WUJI_STATUS_ERR_UNSUPPORTED;
  failed_subscription_state = &state;
  {
    DisconnectRecorder device{state};
    std::atomic_bool cancelled{false};
    TrackedTestProbe probe{"optional", cancelled};
    sdk::GloveSdkApi api{};
    api.last_error = &failed_last_error;
    api.sub_close = &close_failed_handle;
    SubscriptionSet subscriptions{api};
    std::vector<std::pair<const char*, TrackedTestProbe*>> active;
    std::string error;

    testing::internal::CaptureStdout();
    EXPECT_TRUE(start_stream("optional", false, true, nullptr, api, subscriptions, probe,
                             &subscribe_with_failed_handle, active, error));
    const std::string output = testing::internal::GetCapturedStdout();

    EXPECT_TRUE(error.empty());
    EXPECT_NE(output.find("subscribe detail"), std::string::npos);
    EXPECT_TRUE(active.empty());
    EXPECT_EQ(state.close_calls, 1);
    EXPECT_TRUE(state.callback_ran_while_probe_alive);
    EXPECT_EQ(state.callback, nullptr);
    EXPECT_EQ(state.user_data, nullptr);
    std::string probe_error;
    EXPECT_FALSE(probe.snapshot_success(probe_error));
    EXPECT_EQ(probe_error.find("terminal frame kind"), std::string::npos);
  }
  failed_subscription_state = nullptr;

  EXPECT_EQ(state.events, (std::vector<std::string>{"subscribe failed", "subscription closed",
                                                    "probe destroyed", "device disconnected"}));
}

TEST(ReadOnlyStreamStart, ClosesFailedRequiredHandleWithoutMaskingSubscribeError) {
  FailedSubscriptionState state;
  state.subscribe_status = WUJI_STATUS_ERR_PROTOCOL;
  failed_subscription_state = &state;
  {
    DisconnectRecorder device{state};
    std::atomic_bool cancelled{false};
    TrackedTestProbe probe{"required", cancelled};
    sdk::GloveSdkApi api{};
    api.last_error = &failed_last_error;
    api.sub_close = &close_failed_handle;
    SubscriptionSet subscriptions{api};
    std::vector<std::pair<const char*, TrackedTestProbe*>> active;
    std::string error;

    EXPECT_FALSE(start_stream("required", true, true, nullptr, api, subscriptions, probe,
                              &subscribe_with_failed_handle, active, error));

    EXPECT_NE(error.find("subscribe required failed"), std::string::npos);
    EXPECT_NE(error.find("subscribe detail"), std::string::npos);
    EXPECT_EQ(error.find("close changed last error"), std::string::npos);
    EXPECT_TRUE(active.empty());
    EXPECT_EQ(state.close_calls, 1);
    EXPECT_TRUE(state.callback_ran_while_probe_alive);
    EXPECT_EQ(state.callback, nullptr);
    EXPECT_EQ(state.user_data, nullptr);
    std::string probe_error;
    EXPECT_FALSE(probe.snapshot_success(probe_error));
    EXPECT_EQ(probe_error.find("terminal frame kind"), std::string::npos);
  }
  failed_subscription_state = nullptr;

  EXPECT_EQ(state.events, (std::vector<std::string>{"subscribe failed", "subscription closed",
                                                    "probe destroyed", "device disconnected"}));
}

}  // namespace
}  // namespace wuji_glove_driver::hardware_test
