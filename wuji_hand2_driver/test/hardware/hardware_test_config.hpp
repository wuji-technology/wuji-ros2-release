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

#ifndef HARDWARE__HARDWARE_TEST_CONFIG_HPP_
#define HARDWARE__HARDWARE_TEST_CONFIG_HPP_

#include <cerrno>
#include <cmath>
#include <cstdlib>

#include <optional>
#include <string>
#include <utility>

#include "hardware_test_validation.hpp"

namespace wuji_hand2_driver::hardware_test {

struct HardwareTestConfig {
  std::string serial_number;
  std::string endpoint;
  double connect_timeout_seconds{10.0};
  double stream_timeout_seconds{5.0};
  double tactile_timeout_seconds{30.0};
  // Fingertip streams are checked only on request, matching the driver's
  // publish_fingertip_tactile default: most hands have no fingertip modules.
  bool fingertip_tactile{false};
};

struct HardwareTestConfigResult {
  std::optional<HardwareTestConfig> config;
  int exit_code{0};
  std::string message;
};

inline std::string environment_value(const char* name) {
  const char* value = std::getenv(name);
  return value == nullptr ? std::string{} : std::string{value};
}

inline bool parse_bounded_seconds(const char* name, double default_value, double maximum,
                                  double& output, std::string& error) {
  const std::string text = environment_value(name);
  if (text.empty()) {
    output = default_value;
    return true;
  }

  char* end = nullptr;
  errno = 0;
  const double value = std::strtod(text.c_str(), &end);
  if (errno != 0 || end == text.c_str() || *end != '\0' || !std::isfinite(value) || value <= 0.0 ||
      value > maximum) {
    error = std::string{name} + " must be finite, positive, and at most " +
            std::to_string(maximum) + " seconds";
    return false;
  }
  output = value;
  return true;
}

inline HardwareTestConfigResult load_hardware_test_config(bool mutating) {
  if (environment_value("WUJI_HARDWARE_TESTS") != "1") {
    return {std::nullopt, 77, "hardware tests require WUJI_HARDWARE_TESTS=1"};
  }

  HardwareTestConfig config;
  config.serial_number = environment_value("WUJI_HARDWARE_SERIAL");
  config.endpoint = environment_value("WUJI_HARDWARE_ENDPOINT");
  config.fingertip_tactile = environment_value("WUJI_HARDWARE_FINGERTIP_TACTILE") == "1";
  if ((!config.serial_number.empty() && !is_valid_hardware_selector(config.serial_number)) ||
      (!config.endpoint.empty() && !is_valid_hardware_selector(config.endpoint))) {
    return {std::nullopt, 2, "hardware selectors must not contain whitespace or ASCII controls"};
  }
  if (config.serial_number.empty() && config.endpoint.empty()) {
    return {std::nullopt, 2,
            "hardware tests require WUJI_HARDWARE_SERIAL or WUJI_HARDWARE_ENDPOINT"};
  }

  if (mutating) {
    if (config.serial_number.empty() || !config.endpoint.empty()) {
      return {std::nullopt, 2,
              "mutating hardware tests require only an explicit WUJI_HARDWARE_SERIAL"};
    }
    if (environment_value("WUJI_MUTATING_HARDWARE_TESTS") != "1") {
      return {std::nullopt, 3, "mutating hardware tests require WUJI_MUTATING_HARDWARE_TESTS=1"};
    }
    if (environment_value("WUJI_HARDWARE_SAFETY_ACK") !=
        "fixture-unloaded-operator-state-not-restored") {
      return {std::nullopt, 3,
              "mutating hardware tests require the exact WUJI_HARDWARE_SAFETY_ACK"};
    }
  }

  std::string error;
  if (!parse_bounded_seconds("WUJI_HARDWARE_CONNECT_TIMEOUT_SECONDS", 10.0, 60.0,
                             config.connect_timeout_seconds, error) ||
      !parse_bounded_seconds("WUJI_HARDWARE_STREAM_TIMEOUT_SECONDS", 5.0, 30.0,
                             config.stream_timeout_seconds, error) ||
      !parse_bounded_seconds("WUJI_HARDWARE_TACTILE_TIMEOUT_SECONDS", 30.0, 60.0,
                             config.tactile_timeout_seconds, error)) {
    return {std::nullopt, 2, std::move(error)};
  }

  return {std::move(config), 0, {}};
}

}  // namespace wuji_hand2_driver::hardware_test

#endif  // HARDWARE__HARDWARE_TEST_CONFIG_HPP_
