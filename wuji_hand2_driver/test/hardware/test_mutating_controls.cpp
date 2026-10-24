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

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include "hardware_test_config.hpp"

#include "wuji_hand2_driver/sdk/hand2_sdk_api.hpp"
#include "wuji_hand2_driver/sdk/hand2_session.hpp"
#include "wuji_hand2_driver/sdk/sdk_status.hpp"

namespace wuji_hand2_driver::hardware_test {
namespace {

using sdk::Hand2SdkApi;
using sdk::Hand2Session;
using sdk::Status;

constexpr std::size_t kJointCount = WUJI_HAND_2_JOINT_COUNT;
constexpr std::size_t kFingertipCount = 5;
constexpr float kConservativeEffortLimitAmps = 0.1F;
constexpr float kConservativeKp = 0.0F;
constexpr float kConservativeKd = 0.0F;

sdk::Hand2ConnectConfig connect_config(const HardwareTestConfig& config) {
  sdk::Hand2ConnectConfig result;
  result.serial_number = config.serial_number;
  result.connect_timeout = config.connect_timeout_seconds;
  result.retry_count = 0;
  return result;
}

bool require_success(Hand2SdkApi& api, const char* operation, WujiStatus result) {
  const Status status = sdk::sdk_status(api, operation, result);
  if (!status) {
    std::cerr << "FAIL: " << status.message << std::endl;
    return false;
  }
  std::cout << "PASS: " << operation << std::endl;
  return true;
}

bool wait_for_tactile_calibration(Hand2SdkApi& api, WujiDevice* device, double timeout_seconds) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>{timeout_seconds});

  while (std::chrono::steady_clock::now() < deadline) {
    bool all_ready = true;
    for (std::size_t finger = 0; finger < kFingertipCount; ++finger) {
      WujiFingertipSensorStatus sensor{};
      const auto result = api.get_fingertip_status[finger](device, &sensor);
      if (result == WUJI_STATUS_ERR_TIMEOUT) {
        all_ready = false;
        continue;
      }
      if (!require_success(api, "wuji_hand_2_get_fingertip_status", result)) {
        return false;
      }
      if (sensor.state != WUJI_FINGERTIP_SENSOR_STATE_READY) {
        all_ready = false;
      }
    }
    if (all_ready) {
      std::cout << "PASS: all fingertip sensors report calibration ready" << std::endl;
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
  }

  std::cerr << "FAIL: tactile calibration did not complete before the bounded deadline"
            << std::endl;
  return false;
}

int run_mutating_controls(const HardwareTestConfig& config) {
  MutationNoticeGuard mutation_notice(std::cout);
  std::cout << "MUTATING HARDWARE TEST: requires a secured safety fixture, unloaded fingertip "
               "sensor surfaces, and an operator present for the entire run."
            << std::endl;
  std::cout << "Selected serial only: " << config.serial_number << std::endl;

  Hand2SdkApi api = Hand2SdkApi::production();
  Hand2Session session(api);
  const auto connect_status = session.connect_device(connect_config(config));
  if (!connect_status) {
    std::cerr << "FAIL: selected-serial connection: " << connect_status.message << std::endl;
    return 1;
  }
  auto disable_guard = make_cleanup_guard([&]() {
    if (session.device() == nullptr) {
      return;
    }
    const auto result = api.disable(session.device(), nullptr);
    if (result != WUJI_STATUS_OK) {
      std::cerr << "WARN: best-effort whole-hand disable failed during cleanup: "
                << sdk::sdk_status(api, "wuji_hand_2_disable", result).message << std::endl;
    } else {
      std::cout << "PASS: best-effort whole-hand disable completed" << std::endl;
    }
  });
  (void)disable_guard;

  if (!require_success(api, "wuji_hand_2_set_all_effort_limit",
                       api.set_effort_limit(session.device(), kConservativeEffortLimitAmps))) {
    return 1;
  }

  std::array<float, kJointCount> effort_limits{};
  effort_limits.fill(kConservativeEffortLimitAmps);
  if (!require_success(api, "wuji_hand_2_set_all_effort_limit_per_joint",
                       api.set_effort_limits(session.device(), effort_limits.data()))) {
    return 1;
  }

  std::array<float, kJointCount> kp{};
  std::array<float, kJointCount> kd{};
  kp.fill(kConservativeKp);
  kd.fill(kConservativeKd);
  if (!require_success(api, "wuji_hand_2_set_all_mit_params",
                       api.set_mit_params(session.device(), kp.data(), kd.data()))) {
    return 1;
  }
  if (!require_success(api, "wuji_hand_2_clear_fault",
                       api.clear_fault(session.device(), nullptr))) {
    return 1;
  }
  if (!require_success(api, "wuji_hand_2_enable", api.enable(session.device(), nullptr))) {
    return 1;
  }

  if (!require_success(api, "wuji_hand_2_emergency_stop", api.emergency_stop(session.device()))) {
    return 1;
  }
  if (!require_success(api, "wuji_hand_2_calibrate_fingertip",
                       api.calibrate_fingertip(session.device()))) {
    return 1;
  }
  if (!wait_for_tactile_calibration(api, session.device(), config.tactile_timeout_seconds)) {
    return 1;
  }

  std::cout << "PASS: mutating Hand 2 hardware controls completed" << std::endl;
  return 0;
}

}  // namespace
}  // namespace wuji_hand2_driver::hardware_test

int main() {
  return wuji_hand2_driver::hardware_test::run_hardware_exception_boundary(
      "mutating Hand 2 hardware test", std::cerr, []() {
        const auto config = wuji_hand2_driver::hardware_test::load_hardware_test_config(true);
        if (!config.config) {
          std::cerr << config.message << std::endl;
          return config.exit_code;
        }
        return wuji_hand2_driver::hardware_test::run_mutating_controls(*config.config);
      });
}
