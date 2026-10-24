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
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fake_sdk/fake_glove_sdk.hpp"
#include "wuji_glove_driver/driver_config.hpp"
#include "wuji_glove_driver/sdk/glove_session.hpp"

namespace wuji_glove_driver::test {
namespace {

using namespace std::chrono_literals;

class DriverConfigTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  static std::shared_ptr<rclcpp::Node> make_node(
      const std::vector<rclcpp::Parameter>& overrides = {}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    auto node = std::make_shared<rclcpp::Node>("glove_driver_config_test", options);
    declare_driver_parameters(*node);
    return node;
  }
};

TEST_F(DriverConfigTest, DeclaresExactlyThirteenParametersWithRequiredTypesAndDefaults) {
  const auto node = make_node();
  DriverConfig config;

  ASSERT_TRUE(read_driver_config(*node, config));
  EXPECT_EQ(config.endpoint, "");
  EXPECT_EQ(config.serial_number, "");
  EXPECT_EQ(config.hand_side, "");
  EXPECT_DOUBLE_EQ(config.connect_timeout, 10.0);
  EXPECT_TRUE(config.publish_emf_poses);
  EXPECT_TRUE(config.publish_tip_poses);
  EXPECT_TRUE(config.publish_skeleton);
  EXPECT_TRUE(config.publish_imu);
  EXPECT_TRUE(config.publish_tactile);
  EXPECT_TRUE(config.publish_tactile_residual);
  EXPECT_TRUE(config.publish_tactile_binary);
  EXPECT_TRUE(config.publish_tactile_zones);
  EXPECT_TRUE(config.publish_tactile_points);

  auto names = node->list_parameters({}, 1).names;
  const std::vector<std::string> framework_parameters{"start_type_description_service",
                                                      "use_sim_time"};
  names.erase(std::remove_if(names.begin(), names.end(),
                             [&framework_parameters](const auto& name) {
                               return std::find(framework_parameters.begin(),
                                                framework_parameters.end(),
                                                name) != framework_parameters.end();
                             }),
              names.end());
  std::sort(names.begin(), names.end());
  const std::vector<std::string> expected{"connect_timeout",
                                          "endpoint",
                                          "hand_side",
                                          "publish_emf_poses",
                                          "publish_imu",
                                          "publish_skeleton",
                                          "publish_tactile",
                                          "publish_tactile_binary",
                                          "publish_tactile_points",
                                          "publish_tactile_residual",
                                          "publish_tactile_zones",
                                          "publish_tip_poses",
                                          "serial_number"};
  EXPECT_EQ(names, expected);
  EXPECT_EQ(node->get_parameter("endpoint").get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(node->get_parameter("connect_timeout").get_type(),
            rclcpp::ParameterType::PARAMETER_DOUBLE);
  EXPECT_EQ(node->get_parameter("publish_imu").get_type(), rclcpp::ParameterType::PARAMETER_BOOL);
}

TEST_F(DriverConfigTest, ReadsAllThirteenDeclaredOverrides) {
  const auto node = make_node(
      {rclcpp::Parameter("endpoint", "udp://glove"),
       rclcpp::Parameter("serial_number", "WGLK000000000002"),
       rclcpp::Parameter("hand_side", "right"), rclcpp::Parameter("connect_timeout", 2.5),
       rclcpp::Parameter("publish_emf_poses", false), rclcpp::Parameter("publish_tip_poses", false),
       rclcpp::Parameter("publish_skeleton", false), rclcpp::Parameter("publish_imu", false),
       rclcpp::Parameter("publish_tactile", false),
       rclcpp::Parameter("publish_tactile_residual", false),
       rclcpp::Parameter("publish_tactile_binary", false),
       rclcpp::Parameter("publish_tactile_zones", false),
       rclcpp::Parameter("publish_tactile_points", false)});
  DriverConfig config;

  ASSERT_TRUE(read_driver_config(*node, config));
  EXPECT_EQ(config.endpoint, "udp://glove");
  EXPECT_EQ(config.serial_number, "WGLK000000000002");
  EXPECT_EQ(config.hand_side, "right");
  EXPECT_DOUBLE_EQ(config.connect_timeout, 2.5);
  EXPECT_FALSE(config.publish_emf_poses);
  EXPECT_FALSE(config.publish_tip_poses);
  EXPECT_FALSE(config.publish_skeleton);
  EXPECT_FALSE(config.publish_imu);
  EXPECT_FALSE(config.publish_tactile);
  EXPECT_FALSE(config.publish_tactile_residual);
  EXPECT_FALSE(config.publish_tactile_binary);
  EXPECT_FALSE(config.publish_tactile_zones);
  EXPECT_FALSE(config.publish_tactile_points);
}

TEST_F(DriverConfigTest, RejectsInvalidHandSide) {
  const auto node = make_node({rclcpp::Parameter("hand_side", "up")});
  DriverConfig output;
  EXPECT_FALSE(read_driver_config(*node, output));
}

class InvalidTimeoutTest : public DriverConfigTest, public ::testing::WithParamInterface<double> {};

TEST_P(InvalidTimeoutTest, RejectsValueOutsideSdkMillisecondRange) {
  const auto node = make_node({rclcpp::Parameter("connect_timeout", GetParam())});
  DriverConfig output;
  const auto status = read_driver_config(*node, output);
  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("connect_timeout"), std::string::npos);
}

INSTANTIATE_TEST_SUITE_P(
    InvalidValues, InvalidTimeoutTest,
    ::testing::Values(-0.1, std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity(),
                      static_cast<double>(std::numeric_limits<std::uint32_t>::max()) / 1000.0 +
                          0.001));

struct ManualClock {
  std::chrono::steady_clock::time_point value{};
  std::size_t sleeps{0};
  std::chrono::steady_clock::duration sleep_step{50ms};

  sdk::GloveSessionHooks hooks() {
    sdk::GloveSessionHooks result;
    result.now = [this]() { return value; };
    result.sleep_for = [this](std::chrono::steady_clock::duration) {
      ++sleeps;
      value += sleep_step;
    };
    result.cancelled = []() { return false; };
    return result;
  }
};

sdk::GloveConnectConfig endpoint_config() {
  sdk::GloveConnectConfig config;
  config.endpoint = "udp://glove";
  config.connect_timeout = 1.0;
  return config;
}

sdk::GloveConnectConfig serial_config(std::string serial = "WGLJ000000000001") {
  sdk::GloveConnectConfig config;
  config.serial_number = std::move(serial);
  config.connect_timeout = 1.0;
  return config;
}

FakeGloveSdk::Discovery glove(std::string serial, std::string address) {
  FakeGloveSdk::Discovery result;
  result.serial_number = std::move(serial);
  result.address = std::move(address);
  return result;
}

void expect_target(const FakeGloveSdk& fake, std::uint8_t kind, std::string_view value) {
  ASSERT_EQ(fake.connect_calls().size(), 1U);
  EXPECT_EQ(fake.connect_calls().front().target_kind, kind);
  EXPECT_EQ(fake.connect_calls().front().target_value, value);
}

TEST(GloveSession, EndpointBranchConnectsDirectlyByAddress) {
  FakeGloveSdk fake;
  fake.set_scan_results_sequence({{glove("WGLK000000000002", "udp://other")}});
  sdk::GloveSession session(fake.api());

  ASSERT_TRUE(session.connect(endpoint_config()));

  expect_target(fake, WUJI_CONNECT_TARGET_KIND_ADDR, "udp://glove");
  EXPECT_EQ(fake.count("scan"), 0U);
}

TEST(GloveSession, SerialBranchConnectsDirectlyBySerialNumber) {
  FakeGloveSdk fake;
  sdk::GloveSession session(fake.api());

  ASSERT_TRUE(session.connect(serial_config()));

  expect_target(fake, WUJI_CONNECT_TARGET_KIND_SN, "WGLJ000000000001");
  EXPECT_EQ(fake.count("scan"), 0U);
}

TEST(GloveSession, SerialSelectionAlsoValidatesExplicitHandSide) {
  FakeGloveSdk fake;
  fake.set_hand_side("left");
  sdk::GloveSession session(fake.api());
  auto config = serial_config();
  config.hand_side = "left";

  ASSERT_TRUE(session.connect(config));

  expect_target(fake, WUJI_CONNECT_TARGET_KIND_SN, config.serial_number);
  EXPECT_EQ(session.hand_side(), "left");
}

TEST(GloveSession, HandSideBranchFiltersSerialAndVerifiesConnectedSide) {
  FakeGloveSdk fake;
  fake.set_scan_results_sequence(
      {{glove("WGLJ000000000001", "udp://left"), glove("WGLK000000000002", "udp://right")}});
  fake.set_hand_side("right");
  sdk::GloveSession session(fake.api());
  sdk::GloveConnectConfig config;
  config.hand_side = "right";
  config.connect_timeout = 1.0;

  ASSERT_TRUE(session.connect(config));

  expect_target(fake, WUJI_CONNECT_TARGET_KIND_SN, "WGLK000000000002");
  EXPECT_EQ(session.serial_number(), "WGLK000000000002");
}

TEST(GloveSession, NoSelectorConnectsTheOnlyGloveAndIgnoresOtherDeviceTypes) {
  FakeGloveSdk fake;
  auto hand = glove("WH2J000000000001", "udp://hand");
  hand.device_id = WUJI_DEVICE_TYPE_WUJI_HAND_2;
  fake.set_scan_results_sequence({{hand, glove("WGLJ000000000001", "udp://glove")}});
  sdk::GloveSession session(fake.api());
  sdk::GloveConnectConfig config;
  config.connect_timeout = 1.0;

  ASSERT_TRUE(session.connect(config));

  expect_target(fake, WUJI_CONNECT_TARGET_KIND_SN, "WGLJ000000000001");
}

TEST(GloveSession, NoSelectorRejectsMultipleGlovesWithSerialNumberGuidance) {
  FakeGloveSdk fake;
  fake.set_scan_results_sequence(
      {{glove("WGLJ000000000001", "udp://left"), glove("WGLK000000000002", "udp://right")}});
  sdk::GloveSession session(fake.api());
  sdk::GloveConnectConfig config;

  const auto status = session.connect(config);

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("serial_number"), std::string::npos);
  EXPECT_EQ(fake.count("connect"), 0U);
}

TEST(GloveSession, DeduplicatesSameNonemptySerialAcrossTransports) {
  FakeGloveSdk fake;
  auto udp = glove("WGLJ000000000001", "udp://glove");
  auto usb = glove("WGLJ000000000001", "usb://glove");
  usb.transport = WUJI_TRANSPORT_TYPE_USB;
  fake.set_scan_results_sequence({{udp, usb}});
  sdk::GloveSession session(fake.api());
  sdk::GloveConnectConfig config;

  ASSERT_TRUE(session.connect(config));

  expect_target(fake, WUJI_CONNECT_TARGET_KIND_SN, "WGLJ000000000001");
}

TEST(GloveSession, DeduplicatesEmptySerialByFallbackIdentityAndUsesAddress) {
  FakeGloveSdk fake;
  auto first = glove("", "udp://glove");
  auto duplicate = first;
  fake.set_scan_results_sequence({{first, duplicate}});
  fake.set_identity_from_connect_target(false);
  sdk::GloveSession session(fake.api());
  sdk::GloveConnectConfig config;

  ASSERT_TRUE(session.connect(config));

  expect_target(fake, WUJI_CONNECT_TARGET_KIND_ADDR, "udp://glove");
}

TEST(GloveSession, UnknownSerialHandCodeDoesNotMatchExplicitSide) {
  FakeGloveSdk fake;
  fake.set_scan_results_sequence({{glove("WGLX000000000001", "udp://unknown")}});
  ManualClock clock;
  sdk::GloveSession session(fake.api(), clock.hooks());
  sdk::GloveConnectConfig config;
  config.hand_side = "left";
  config.connect_timeout = 0.1;

  const auto status = session.connect(config);

  EXPECT_FALSE(status);
  EXPECT_EQ(fake.count("connect"), 0U);
}

TEST(GloveSession, ExplicitSideRejectsMultipleMatchingGloves) {
  FakeGloveSdk fake;
  fake.set_scan_results_sequence(
      {{glove("WGLJ000000000001", "udp://one"), glove("WGLJ000000000002", "udp://two")}});
  sdk::GloveSession session(fake.api());
  sdk::GloveConnectConfig config;
  config.hand_side = "left";

  const auto status = session.connect(config);

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("serial_number"), std::string::npos);
}

TEST(GloveSession, RetriesZeroDeviceScansUntilOneAppears) {
  FakeGloveSdk fake;
  fake.set_scan_results_sequence({{}, {}, {glove("WGLJ000000000001", "udp://glove")}});
  ManualClock clock;
  sdk::GloveSession session(fake.api(), clock.hooks());
  sdk::GloveConnectConfig config;
  config.connect_timeout = 1.0;

  ASSERT_TRUE(session.connect(config));

  EXPECT_EQ(fake.count("scan"), 3U);
  EXPECT_EQ(clock.sleeps, 2U);
}

TEST(GloveSession, ZeroTimeoutWaitsUntilCancellation) {
  FakeGloveSdk fake;
  fake.set_scan_results_sequence({{}});
  ManualClock clock;
  auto hooks = clock.hooks();
  hooks.cancelled = [&clock]() { return clock.sleeps >= 3; };
  sdk::GloveSession session(fake.api(), std::move(hooks));
  sdk::GloveConnectConfig config;
  config.connect_timeout = 0.0;

  const auto status = session.connect(config);

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("cancel"), std::string::npos);
  EXPECT_EQ(fake.count("scan"), 3U);
  EXPECT_EQ(fake.count("shutdown"), 1U);
}

class InfiniteDirectConnectTest : public ::testing::TestWithParam<bool> {};

TEST_P(InfiniteDirectConnectTest, RetriesTransientFailuresUntilConnectionSucceeds) {
  FakeGloveSdk fake;
  fake.set_connect_failure(WUJI_STATUS_ERR_TIMEOUT, "connect failed");
  fake.set_call_hook([&fake](std::string_view operation) {
    if (operation == "connect" && fake.count("connect") == 3U) {
      fake.set_connect_failure(WUJI_STATUS_OK, "");
    }
  });
  ManualClock clock;
  sdk::GloveSession session(fake.api(), clock.hooks());
  auto config = GetParam() ? endpoint_config() : serial_config();
  config.connect_timeout = 0.0;

  ASSERT_TRUE(session.connect(config));

  ASSERT_EQ(fake.connect_calls().size(), 3U);
  EXPECT_EQ(clock.sleeps, 2U);
  for (const auto& call : fake.connect_calls()) {
    EXPECT_EQ(call.target_kind,
              GetParam() ? WUJI_CONNECT_TARGET_KIND_ADDR : WUJI_CONNECT_TARGET_KIND_SN);
    EXPECT_EQ(call.target_value, GetParam() ? "udp://glove" : "WGLJ000000000001");
  }
}

INSTANTIATE_TEST_SUITE_P(EndpointAndSerial, InfiniteDirectConnectTest,
                         ::testing::Values(false, true));

TEST(GloveSession, ScanWaitAndConnectShareOneDeadline) {
  FakeGloveSdk fake;
  fake.set_scan_results_sequence({{}, {glove("WGLJ000000000001", "udp://glove")}});
  ManualClock clock;
  clock.sleep_step = 100ms;
  sdk::GloveSession session(fake.api(), clock.hooks());
  sdk::GloveConnectConfig config;
  config.connect_timeout = 0.25;

  ASSERT_TRUE(session.connect(config));

  ASSERT_EQ(fake.connect_calls().size(), 1U);
  EXPECT_GT(fake.connect_calls().front().options.timeout_ms, 0U);
  EXPECT_LE(fake.connect_calls().front().options.timeout_ms, 150U);
  EXPECT_FALSE(fake.connect_calls().front().options.enable_bridge);
}

TEST(GloveSession, DeadlineExpiredWhilePreparingConnectSkipsSdkConnect) {
  FakeGloveSdk fake;
  ManualClock clock;
  fake.set_call_hook([&clock](std::string_view operation) {
    if (operation == "connect_options_default") {
      clock.value += 200ms;
    }
  });
  sdk::GloveSession session(fake.api(), clock.hooks());
  auto config = endpoint_config();
  config.connect_timeout = 0.1;

  const auto status = session.connect(config);

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("connect_timeout"), std::string::npos);
  EXPECT_EQ(fake.count("connect"), 0U);
  EXPECT_EQ(fake.count("device_serial_number"), 0U);
  EXPECT_EQ(fake.count("disconnect"), 0U);
  EXPECT_EQ(fake.count("release"), 0U);
  EXPECT_EQ(fake.count("shutdown"), 1U);
}

TEST(GloveSession, DeadlineExpiredByConnectRollsBackBeforeIdentityReads) {
  FakeGloveSdk fake;
  ManualClock clock;
  fake.set_call_hook([&clock](std::string_view operation) {
    if (operation == "connect") {
      clock.value += 200ms;
    }
  });
  sdk::GloveSession session(fake.api(), clock.hooks());
  auto config = endpoint_config();
  config.connect_timeout = 0.1;

  const auto status = session.connect(config);

  EXPECT_FALSE(status);
  EXPECT_EQ(fake.count("device_serial_number"), 0U);
  EXPECT_EQ(fake.count("disconnect"), 1U);
  EXPECT_EQ(fake.count("release"), 1U);
}

TEST(GloveSession, DeadlineExpiredByIdentityReadStopsBeforeNextIdentityCall) {
  FakeGloveSdk fake;
  ManualClock clock;
  fake.set_call_hook([&clock](std::string_view operation) {
    if (operation == "device_serial_number") {
      clock.value += 40ms;
    }
  });
  sdk::GloveSession session(fake.api(), clock.hooks());
  auto config = endpoint_config();
  config.connect_timeout = 0.1;

  const auto status = session.connect(config);

  EXPECT_FALSE(status);
  EXPECT_EQ(fake.count("device_model"), 0U);
  EXPECT_EQ(fake.count("disconnect"), 1U);
}

TEST(GloveSession, UnstableIdentityGetterStopsAtSharedDeadline) {
  FakeGloveSdk fake;
  fake.set_identity_from_connect_target(false);
  ManualClock clock;
  std::size_t serial_calls = 0;
  fake.set_call_hook([&clock, &fake, &serial_calls](std::string_view operation) {
    if (operation == "device_serial_number") {
      ++serial_calls;
      clock.value += 40ms;
      fake.set_serial_number(std::string(serial_calls, 'x'));
    }
  });
  sdk::GloveSession session(fake.api(), clock.hooks());
  auto config = endpoint_config();
  config.connect_timeout = 0.1;

  const auto status = session.connect(config);

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("connect_timeout"), std::string::npos);
  EXPECT_EQ(fake.count("device_serial_number"), 3U);
  EXPECT_EQ(fake.count("device_model"), 0U);
  EXPECT_EQ(fake.count("get_hand_side"), 0U);
  EXPECT_EQ(fake.count("device_is_connected"), 0U);
  EXPECT_EQ(fake.count("disconnect"), 1U);
  EXPECT_EQ(fake.count("release"), 1U);
  EXPECT_EQ(fake.count("shutdown"), 1U);
}

TEST(GloveSession, RejectsWrongModelAndRollsBack) {
  FakeGloveSdk fake;
  fake.set_model("0x0002");
  sdk::GloveSession session(fake.api());

  const auto status = session.connect(serial_config());

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("0x0001"), std::string::npos);
  EXPECT_EQ(fake.count("disconnect"), 1U);
  EXPECT_EQ(fake.count("release"), 1U);
}

TEST(GloveSession, RejectsConnectedSerialMismatch) {
  FakeGloveSdk fake;
  fake.set_identity_from_connect_target(false);
  fake.set_serial_number("WGLK000000000002");
  sdk::GloveSession session(fake.api());

  const auto status = session.connect(serial_config());

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("serial_number"), std::string::npos);
  EXPECT_EQ(fake.count("disconnect"), 1U);
}

TEST(GloveSession, RejectsConnectedSideMismatch) {
  FakeGloveSdk fake;
  fake.set_hand_side("right");
  sdk::GloveSession session(fake.api());
  auto config = serial_config();
  config.hand_side = "left";

  const auto status = session.connect(config);

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("hand_side"), std::string::npos);
  EXPECT_EQ(fake.count("disconnect"), 1U);
}

TEST(GloveSession, RejectsDisconnectedDeviceAndCleansUpInReverseOrder) {
  FakeGloveSdk fake;
  fake.set_connected(false);
  sdk::GloveSession session(fake.api());

  const auto status = session.connect(serial_config());

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("disconnected"), std::string::npos);
  const auto connected = std::find(fake.calls().begin(), fake.calls().end(), "device_is_connected");
  const auto disconnect = std::find(fake.calls().begin(), fake.calls().end(), "disconnect");
  const auto release = std::find(fake.calls().begin(), fake.calls().end(), "release");
  const auto shutdown = std::find(fake.calls().begin(), fake.calls().end(), "shutdown");
  ASSERT_NE(connected, fake.calls().end());
  ASSERT_NE(disconnect, fake.calls().end());
  ASSERT_NE(release, fake.calls().end());
  ASSERT_NE(shutdown, fake.calls().end());
  EXPECT_LT(connected, disconnect);
  EXPECT_LT(disconnect, release);
  EXPECT_LT(release, shutdown);
}

TEST(GloveSession, IdentityFailureCopiesErrorAndRollsBackInReverseOrder) {
  FakeGloveSdk fake;
  fake.set_serial_number_failure(WUJI_STATUS_ERR_INTERNAL, "serial unavailable");
  sdk::GloveSession session(fake.api());

  const auto status = session.connect(endpoint_config());

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("serial unavailable"), std::string::npos);
  const auto error = std::find(fake.calls().begin(), fake.calls().end(), "last_error");
  const auto disconnect = std::find(fake.calls().begin(), fake.calls().end(), "disconnect");
  const auto release = std::find(fake.calls().begin(), fake.calls().end(), "release");
  const auto shutdown = std::find(fake.calls().begin(), fake.calls().end(), "shutdown");
  ASSERT_NE(error, fake.calls().end());
  ASSERT_NE(disconnect, fake.calls().end());
  ASSERT_NE(release, fake.calls().end());
  ASSERT_NE(shutdown, fake.calls().end());
  EXPECT_LT(error, disconnect);
  EXPECT_LT(disconnect, release);
  EXPECT_LT(release, shutdown);
}

TEST(GloveSession, HandSideIdentityFailureRollsBack) {
  FakeGloveSdk fake;
  fake.set_hand_side_failure(WUJI_STATUS_ERR_INTERNAL, "side unavailable");
  sdk::GloveSession session(fake.api());

  const auto status = session.connect(serial_config());

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("side unavailable"), std::string::npos);
  EXPECT_EQ(fake.count("disconnect"), 1U);
  EXPECT_EQ(fake.count("release"), 1U);
}

TEST(GloveSession, FailedConnectReturningHandleRollsBackInReverseOrder) {
  FakeGloveSdk fake;
  fake.set_connect_failure(WUJI_STATUS_ERR_INTERNAL, "connect failed", true);
  sdk::GloveSession session(fake.api());

  const auto status = session.connect(endpoint_config());

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("connect failed"), std::string::npos);
  const std::vector<std::string> tail{"disconnect", "release", "shutdown"};
  ASSERT_GE(fake.calls().size(), tail.size());
  EXPECT_TRUE(std::equal(tail.begin(), tail.end(), fake.calls().end() - tail.size()));
}

TEST(GloveSession, RuntimeIsReferenceCountedAcrossSessions) {
  FakeGloveSdk fake;
  sdk::GloveSession first(fake.api());
  sdk::GloveSession second(fake.api());
  ASSERT_TRUE(first.connect(serial_config("WGLJ000000000001")));
  ASSERT_TRUE(second.connect(serial_config("WGLJ000000000002")));

  EXPECT_EQ(fake.count("init"), 1U);
  first.disconnect();
  EXPECT_EQ(fake.count("shutdown"), 0U);
  second.disconnect();
  EXPECT_EQ(fake.count("shutdown"), 1U);
}

TEST(GloveSession, DisconnectAndDestructorAreIdempotent) {
  FakeGloveSdk fake;
  {
    sdk::GloveSession session(fake.api());
    ASSERT_TRUE(session.connect(serial_config()));
    session.disconnect();
    session.disconnect();
  }

  EXPECT_EQ(fake.count("disconnect"), 1U);
  EXPECT_EQ(fake.count("release"), 1U);
  EXPECT_EQ(fake.count("shutdown"), 1U);
}

}  // namespace
}  // namespace wuji_glove_driver::test
