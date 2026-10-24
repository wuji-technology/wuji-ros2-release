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
#include <rcl/timer.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fake_sdk/fake_glove_sdk.hpp"
#include "wuji_glove_driver/glove_node.hpp"
#include "wuji_glove_driver/lifecycle/exit_state.hpp"

namespace wuji_glove_driver {
namespace {

using namespace std::chrono_literals;
using test::FakeGloveSdk;
using TypedStream = FakeGloveSdk::TypedStream;

constexpr std::array<TypedStream, 10> kAllStreams{
    TypedStream::JointAngles,       TypedStream::EmfPoses,      TypedStream::TipPoses,
    TypedStream::HandSkeleton,      TypedStream::ImuPalm,       TypedStream::Tactile,
    TypedStream::TactileResidual,   TypedStream::TactileBinary, TypedStream::TactileZones,
    TypedStream::TactilePointCloud,
};

class GloveNodeTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  std::shared_ptr<GloveNode> make_node(std::vector<rclcpp::Parameter> overrides = {}) {
    const auto has_target =
        std::find_if(overrides.cbegin(), overrides.cend(), [](const rclcpp::Parameter& parameter) {
          return parameter.get_name() == "serial_number" || parameter.get_name() == "endpoint";
        });
    if (has_target == overrides.cend()) {
      overrides.emplace_back("serial_number", "WGLJ000000000001");
    }
    rclcpp::NodeOptions options;
    options.start_parameter_services(true);
    options.start_parameter_event_publisher(true);
    options.parameter_overrides(std::move(overrides));
    GloveNodeHooks hooks;
    hooks.cancelled = [this]() {
      return cancelled_.load(std::memory_order_acquire) ||
             (cancel_after_first_subscribe_ && !fake_.subscribed_streams().empty());
    };
    hooks.request_shutdown = [this]() {
      shutdown_requested_.store(true, std::memory_order_release);
    };
    return std::make_shared<GloveNode>(fake_.api(), std::move(hooks), options);
  }

  bool spin_until(const std::shared_ptr<GloveNode>& node, const std::function<bool()>& predicate,
                  std::chrono::milliseconds timeout = 500ms) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    executor.remove_node(node);
    return predicate();
  }

  static std::vector<rclcpp::Parameter> all_optional_disabled() {
    return {
        {"publish_emf_poses", false},      {"publish_tip_poses", false},
        {"publish_skeleton", false},       {"publish_imu", false},
        {"publish_tactile", false},        {"publish_tactile_residual", false},
        {"publish_tactile_binary", false}, {"publish_tactile_zones", false},
        {"publish_tactile_points", false},
    };
  }

  FakeGloveSdk fake_;
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> cancelled_{false};
  bool cancel_after_first_subscribe_{false};
};

TEST(LifecycleExitStateTest, LatchesOnlyTheFirstRequest) {
  lifecycle::ExitState state;
  EXPECT_FALSE(state.requested());
  EXPECT_EQ(state.code(), EXIT_SUCCESS);
  EXPECT_TRUE(state.request(EXIT_FAILURE, "first"));
  EXPECT_FALSE(state.request(7, "second"));
  EXPECT_TRUE(state.requested());
  EXPECT_EQ(state.code(), EXIT_FAILURE);
  EXPECT_EQ(state.reason(), "first");
}

TEST_F(GloveNodeTest, DefaultRegistersOneRequiredAndNineOptionalStreams) {
  auto node = make_node();
  ASSERT_TRUE(node->start());
  EXPECT_EQ(fake_.subscribed_streams(),
            (std::vector<TypedStream>{kAllStreams.begin(), kAllStreams.end()}));
  node->stop();
}

TEST_F(GloveNodeTest, EachPublishFlagControlsItsExactOptionalStreams) {
  struct FlagCase {
    const char* parameter;
    std::vector<TypedStream> streams;
  };
  const std::vector<FlagCase> cases{
      {"publish_emf_poses", {TypedStream::EmfPoses}},
      {"publish_tip_poses", {TypedStream::TipPoses}},
      {"publish_skeleton", {TypedStream::HandSkeleton}},
      {"publish_imu", {TypedStream::ImuPalm}},
      {"publish_tactile", {TypedStream::Tactile}},
      {"publish_tactile_residual", {TypedStream::TactileResidual}},
      {"publish_tactile_binary", {TypedStream::TactileBinary}},
      {"publish_tactile_zones", {TypedStream::TactileZones}},
      {"publish_tactile_points", {TypedStream::TactilePointCloud}},
  };

  for (const auto& item : cases) {
    SCOPED_TRACE(item.parameter);
    auto overrides = all_optional_disabled();
    const auto selected = std::find_if(overrides.begin(), overrides.end(), [&item](const auto& p) {
      return p.get_name() == item.parameter;
    });
    ASSERT_NE(selected, overrides.end());
    *selected = rclcpp::Parameter(item.parameter, true);
    const auto subscribed_before = fake_.subscribed_streams().size();
    auto node = make_node(std::move(overrides));
    ASSERT_TRUE(node->start());
    std::vector<TypedStream> actual(fake_.subscribed_streams().begin() + subscribed_before,
                                    fake_.subscribed_streams().end());
    std::vector<TypedStream> expected{TypedStream::JointAngles};
    expected.insert(expected.end(), item.streams.begin(), item.streams.end());
    EXPECT_EQ(actual, expected);
    node->stop();
  }
}

TEST_F(GloveNodeTest, RequiredSubscribeFailureRollsBackBeforeDisconnect) {
  fake_.set_subscribe_failure(TypedStream::JointAngles, WUJI_STATUS_ERR_INTERNAL, "required failed",
                              true);
  auto node = make_node();
  EXPECT_FALSE(node->start());
  EXPECT_EQ(fake_.count("last_error"), 1U);
  EXPECT_NE(node->exit_reason().find("wuji_glove_subscribe_hand_joint_angles"), std::string::npos);
  EXPECT_NE(node->exit_reason().find("required failed"), std::string::npos);
  EXPECT_TRUE(node->exit_requested());
  EXPECT_EQ(node->exit_code(), EXIT_FAILURE);
  EXPECT_EQ(fake_.closed_streams(), (std::vector<TypedStream>{TypedStream::JointAngles}));
  const auto& calls = fake_.calls();
  const auto close = std::find(calls.begin(), calls.end(), "sub_close");
  const auto disconnect = std::find(calls.begin(), calls.end(), "disconnect");
  ASSERT_NE(close, calls.end());
  ASSERT_NE(disconnect, calls.end());
  EXPECT_LT(close, disconnect);
}

TEST_F(GloveNodeTest, StartCancelledByShutdownRequestExitsWithSuccess) {
  cancelled_.store(true, std::memory_order_release);
  auto node = make_node();

  EXPECT_FALSE(node->start());

  EXPECT_TRUE(node->exit_requested());
  EXPECT_EQ(node->exit_code(), EXIT_SUCCESS);
  EXPECT_NE(node->exit_reason().find("startup cancelled by shutdown request"), std::string::npos);
  EXPECT_EQ(fake_.count("connect"), 0U);
}

TEST_F(GloveNodeTest, ShutdownRequestDuringStreamStartupExitsWithSuccess) {
  cancel_after_first_subscribe_ = true;
  auto node = make_node();

  EXPECT_FALSE(node->start());

  EXPECT_EQ(node->exit_code(), EXIT_SUCCESS);
  EXPECT_NE(node->exit_reason().find("shutdown requested during stream startup"),
            std::string::npos);
  EXPECT_EQ(fake_.count("disconnect"), 1U);
}

TEST_F(GloveNodeTest, OptionalSubscribeFailureDoesNotStopOtherStreamsOrExit) {
  fake_.set_subscribe_failure(TypedStream::EmfPoses, WUJI_STATUS_ERR_NOT_FOUND,
                              "optional unsupported");
  auto node = make_node();
  ASSERT_TRUE(node->start());
  EXPECT_EQ(fake_.count("last_error"), 1U);
  EXPECT_FALSE(node->exit_requested());
  EXPECT_EQ(fake_.subscribed_streams().size(), 9U);
  EXPECT_EQ(std::count(fake_.subscribed_streams().begin(), fake_.subscribed_streams().end(),
                       TypedStream::EmfPoses),
            0);
  EXPECT_EQ(std::count(fake_.subscribed_streams().begin(), fake_.subscribed_streams().end(),
                       TypedStream::TipPoses),
            1);
  node->stop();
}

TEST_F(GloveNodeTest, RequiredEndAndErrorRequestNonzeroExit) {
  for (const auto kind : {WUJI_FRAME_KIND_END, WUJI_FRAME_KIND_ERROR}) {
    shutdown_requested_.store(false, std::memory_order_release);
    auto node = make_node(all_optional_disabled());
    ASSERT_TRUE(node->start());
    fake_.emit(TypedStream::JointAngles, kind);
    ASSERT_TRUE(
        spin_until(node, [this]() { return shutdown_requested_.load(std::memory_order_acquire); }));
    EXPECT_TRUE(node->exit_requested());
    EXPECT_NE(node->exit_code(), EXIT_SUCCESS);
    node->stop();
  }
}

TEST_F(GloveNodeTest, OptionalEndAndErrorStopOnlyTheirStreams) {
  auto node = make_node();
  ASSERT_TRUE(node->start());
  fake_.emit(TypedStream::EmfPoses, WUJI_FRAME_KIND_END);
  ASSERT_TRUE(spin_until(node, [this]() { return fake_.closed_streams().size() == 1U; }));
  EXPECT_EQ(fake_.closed_streams(), (std::vector<TypedStream>{TypedStream::EmfPoses}));
  EXPECT_FALSE(node->exit_requested());
  EXPECT_FALSE(shutdown_requested_.load(std::memory_order_acquire));

  fake_.emit(TypedStream::TipPoses, WUJI_FRAME_KIND_ERROR);
  ASSERT_TRUE(spin_until(node, [this]() { return fake_.closed_streams().size() == 2U; }));
  EXPECT_EQ(fake_.closed_streams().back(), TypedStream::TipPoses);
  EXPECT_FALSE(node->exit_requested());
  node->stop();
}

TEST_F(GloveNodeTest, StopClosesStreamsInReverseBeforeDisconnectAndIsIdempotent) {
  auto node = make_node();
  ASSERT_TRUE(node->start());
  node->stop();
  node->stop();

  EXPECT_EQ(fake_.closed_streams(),
            (std::vector<TypedStream>{kAllStreams.rbegin(), kAllStreams.rend()}));
  EXPECT_EQ(fake_.count("sub_close"), 10U);
  EXPECT_EQ(fake_.count("disconnect"), 1U);
  const auto& calls = fake_.calls();
  const auto last_close = std::find(calls.rbegin(), calls.rend(), "sub_close").base();
  const auto disconnect = std::find(calls.begin(), calls.end(), "disconnect");
  ASSERT_NE(disconnect, calls.end());
  EXPECT_LE(last_close, disconnect);
}

TEST_F(GloveNodeTest, GraphContainsOnlyDataPublishersAndFaultTimerIsFiftyMilliseconds) {
  auto node = make_node();
  ASSERT_TRUE(node->start());

  const auto graph = node->get_node_graph_interface();
  EXPECT_TRUE(graph->get_subscriber_names_and_types_by_node(node->get_name(), node->get_namespace())
                  .empty());
  const auto services =
      graph->get_service_names_and_types_by_node(node->get_name(), node->get_namespace());
  ASSERT_EQ(services.size(), 1U);
  EXPECT_EQ(services.begin()->first, "/wuji_glove_driver/get_type_description");
  const auto publishers =
      graph->get_publisher_names_and_types_by_node(node->get_name(), node->get_namespace());
  EXPECT_EQ(publishers.count("/parameter_events"), 0U);
  std::set<std::string> data_topics;
  for (const auto& [topic, types] : publishers) {
    static_cast<void>(types);
    if (topic != "/rosout") {
      data_topics.insert(topic);
    }
  }
  EXPECT_EQ(data_topics,
            (std::set<std::string>{"/joint_angles", "/emf_poses", "/tip_poses", "/hand_skeleton",
                                   "/imu/palm", "/tactile", "/tactile/residual", "/tactile/binary",
                                   "/tactile/zones", "/tactile/points"}));

  std::vector<std::int64_t> timer_periods;
  node->get_node_base_interface()->get_default_callback_group()->collect_all_ptrs(
      [](const rclcpp::SubscriptionBase::SharedPtr&) {},
      [](const rclcpp::ServiceBase::SharedPtr&) {}, [](const rclcpp::ClientBase::SharedPtr&) {},
      [&timer_periods](const rclcpp::TimerBase::SharedPtr& timer) {
        std::int64_t period = 0;
        EXPECT_EQ(rcl_timer_get_period(timer->get_timer_handle().get(), &period), RCL_RET_OK);
        timer_periods.push_back(period);
      },
      [](const rclcpp::Waitable::SharedPtr&) {});
  EXPECT_EQ(timer_periods, (std::vector<std::int64_t>{50'000'000}));
  node->stop();
}

}  // namespace
}  // namespace wuji_glove_driver
