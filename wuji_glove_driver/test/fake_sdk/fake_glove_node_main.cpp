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

#include <wuji_sdk.h>

#include <atomic>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fake_sdk/fake_glove_sdk.hpp"
#include "wuji_glove_driver/glove_node.hpp"

namespace {

using namespace std::chrono_literals;
using FakeGloveSdk = wuji_glove_driver::test::FakeGloveSdk;
using TypedStream = FakeGloveSdk::TypedStream;

constexpr std::string_view kEventPrefix{"WUJI_GLOVE_TEST_EVENT "};

void print_event(std::string_view event) {
  std::fprintf(stdout, "%.*s%.*s\n", static_cast<int>(kEventPrefix.size()), kEventPrefix.data(),
               static_cast<int>(event.size()), event.data());
  std::fflush(stdout);
}

bool is_observed_call(std::string_view call) noexcept {
  return call.rfind("subscribe_", 0) == 0 || call.rfind("close:", 0) == 0 || call == "disconnect";
}

enum class Scenario { RequiredEnd, RequiredError, OptionalEnd, OptionalError, NormalShutdown };

bool is_optional(Scenario scenario) noexcept {
  return scenario == Scenario::OptionalEnd || scenario == Scenario::OptionalError;
}

bool read_scenario(Scenario& output) noexcept {
  const char* value = std::getenv("WUJI_GLOVE_FAKE_SCENARIO");
  if (value == nullptr) {
    return false;
  }
  if (std::strcmp(value, "required_end") == 0) {
    output = Scenario::RequiredEnd;
  } else if (std::strcmp(value, "required_error") == 0) {
    output = Scenario::RequiredError;
  } else if (std::strcmp(value, "optional_end") == 0) {
    output = Scenario::OptionalEnd;
  } else if (std::strcmp(value, "optional_error") == 0) {
    output = Scenario::OptionalError;
  } else if (std::strcmp(value, "normal_shutdown") == 0) {
    output = Scenario::NormalShutdown;
  } else {
    return false;
  }
  return true;
}

bool read_expected_stream_count(std::size_t& output) noexcept {
  const char* value = std::getenv("WUJI_GLOVE_EXPECTED_STREAM_COUNT");
  if (value == nullptr) {
    output = 10U;
    return true;
  }
  const std::string_view text{value};
  const auto result = std::from_chars(text.data(), text.data() + text.size(), output);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

struct SelectorExpectation {
  std::string kind;
  std::string value;
};

bool read_selector_expectation(SelectorExpectation& output) {
  const char* kind = std::getenv("WUJI_GLOVE_EXPECTED_SELECTOR_KIND");
  const char* value = std::getenv("WUJI_GLOVE_EXPECTED_SELECTOR_VALUE");
  if (value != nullptr && value[0] == '\0') {
    return false;
  }
  output.kind = kind == nullptr ? "unique_discovery" : kind;
  output.value = value == nullptr ? "WGLJ000000000001" : value;
  return output.kind == "serial_number" || output.kind == "endpoint" ||
         output.kind == "hand_side" || output.kind == "unique_discovery";
}

void configure_discoveries(FakeGloveSdk& fake, const SelectorExpectation& expectation) {
  std::vector<FakeGloveSdk::Discovery> discoveries{
      {"WGLJ000000000001", "WujiGlove", WUJI_DEVICE_TYPE_WUJI_GLOVE, WUJI_TRANSPORT_TYPE_UDP,
       "udp://127.0.0.1:41000"}};
  if (expectation.kind != "unique_discovery") {
    discoveries.push_back({"WGLK000000000002", "WujiGlove", WUJI_DEVICE_TYPE_WUJI_GLOVE,
                           WUJI_TRANSPORT_TYPE_UDP, "udp://127.0.0.1:41001"});
  }
  fake.set_scan_results_sequence({std::move(discoveries)});
}

bool selector_matches(const FakeGloveSdk& fake, const SelectorExpectation& expectation) {
  const auto& connections = fake.connect_calls();
  if (connections.size() != 1U) {
    return false;
  }
  const auto expected_target_kind =
      expectation.kind == "endpoint" ? WUJI_CONNECT_TARGET_KIND_ADDR : WUJI_CONNECT_TARGET_KIND_SN;
  if (connections.front().target_kind != expected_target_kind ||
      connections.front().target_value != expectation.value) {
    return false;
  }
  const bool expects_scan =
      expectation.kind == "hand_side" || expectation.kind == "unique_discovery";
  return (fake.count("scan") != 0U) == expects_scan;
}

void emit_scenario(FakeGloveSdk& fake, Scenario scenario) {
  switch (scenario) {
    case Scenario::RequiredEnd:
      fake.emit(TypedStream::JointAngles, WUJI_FRAME_KIND_END);
      break;
    case Scenario::RequiredError:
      fake.emit(TypedStream::JointAngles, WUJI_FRAME_KIND_ERROR);
      break;
    case Scenario::OptionalEnd:
      fake.emit(TypedStream::EmfPoses, WUJI_FRAME_KIND_END);
      break;
    case Scenario::OptionalError:
      fake.emit(TypedStream::EmfPoses, WUJI_FRAME_KIND_ERROR);
      break;
    case Scenario::NormalShutdown:
      break;
  }
}

}  // namespace

int main(int argc, char** argv) {
  Scenario scenario{Scenario::NormalShutdown};
  if (!read_scenario(scenario)) {
    std::fputs("WUJI_GLOVE_FAKE_SCENARIO is missing or invalid\n", stderr);
    return EXIT_FAILURE;
  }
  std::size_t expected_stream_count{0U};
  if (!read_expected_stream_count(expected_stream_count)) {
    std::fputs("WUJI_GLOVE_EXPECTED_STREAM_COUNT is invalid\n", stderr);
    return EXIT_FAILURE;
  }
  SelectorExpectation selector_expectation;
  if (!read_selector_expectation(selector_expectation)) {
    std::fputs(
        "WUJI_GLOVE_EXPECTED_SELECTOR_KIND or WUJI_GLOVE_EXPECTED_SELECTOR_VALUE is invalid\n",
        stderr);
    return EXIT_FAILURE;
  }

  rclcpp::init(argc, argv);
  FakeGloveSdk fake;
  configure_discoveries(fake, selector_expectation);
  fake.set_call_hook([](std::string_view call) {
    if (is_observed_call(call)) {
      print_event(call);
    }
  });

  auto node = std::make_shared<wuji_glove_driver::GloveNode>(
      fake.api(), wuji_glove_driver::GloveNodeHooks{}, rclcpp::NodeOptions{});
  const auto started = node->start();
  if (!started || fake.subscribed_streams().size() != expected_stream_count ||
      !selector_matches(fake, selector_expectation)) {
    node->stop();
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  print_event("selector_verified:" + selector_expectation.kind + ":" + selector_expectation.value);

  std::atomic<bool> injected{false};
  std::atomic<bool> optional_fault_processed{false};
  const auto injection_timer = node->create_wall_timer(10ms, [&fake, scenario, &injected]() {
    if (scenario != Scenario::NormalShutdown &&
        !injected.exchange(true, std::memory_order_acq_rel)) {
      emit_scenario(fake, scenario);
    }
  });
  const auto observation_timer =
      node->create_wall_timer(10ms, [&fake, scenario, &injected, &optional_fault_processed]() {
        if (is_optional(scenario) && injected.load(std::memory_order_acquire) &&
            !fake.closed_streams().empty() &&
            !optional_fault_processed.exchange(true, std::memory_order_acq_rel)) {
          print_event("optional_fault_processed");
        }
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  print_event("ready");
  executor.spin();
  executor.remove_node(node);
  static_cast<void>(injection_timer);
  static_cast<void>(observation_timer);
  node->stop();
  const int exit_code = node->exit_code();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
