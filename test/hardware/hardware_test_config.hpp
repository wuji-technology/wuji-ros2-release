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
#include "wuji_glove_driver/driver_config.hpp"

namespace wuji_glove_driver::hardware_test {

struct HardwareTestConfig {
  DriverConfig driver;
  double stream_timeout_seconds{5.0};
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

inline bool parse_bool(const char* name, bool default_value, bool& output, std::string& error) {
  const std::string value = environment_value(name);
  if (value.empty()) {
    output = default_value;
    return true;
  }
  if (value == "1" || value == "true") {
    output = true;
    return true;
  }
  if (value == "0" || value == "false") {
    output = false;
    return true;
  }
  error = std::string{name} + " must be 0, 1, false, or true";
  return false;
}

inline bool parse_seconds(const char* name, double default_value, double maximum, double& output,
                          std::string& error) {
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

inline HardwareTestConfigResult load_hardware_test_config() {
  if (environment_value("WUJI_GLOVE_HARDWARE_TESTS") != "1") {
    return {std::nullopt, 77, "Glove hardware tests require WUJI_GLOVE_HARDWARE_TESTS=1"};
  }

  HardwareTestConfig config;
  config.driver.serial_number = environment_value("WUJI_GLOVE_HARDWARE_SERIAL");
  config.driver.endpoint = environment_value("WUJI_GLOVE_HARDWARE_ENDPOINT");
  config.driver.hand_side = environment_value("WUJI_GLOVE_HARDWARE_HAND_SIDE");
  if (!is_valid_selector(config.driver.serial_number) ||
      !is_valid_selector(config.driver.endpoint)) {
    return {std::nullopt, 2, "Glove hardware selectors must not contain whitespace or controls"};
  }
  if (!config.driver.hand_side.empty() && config.driver.hand_side != "left" &&
      config.driver.hand_side != "right") {
    return {std::nullopt, 2, "WUJI_GLOVE_HARDWARE_HAND_SIDE must be left, right, or empty"};
  }

  std::string error;
  if (!parse_seconds("WUJI_GLOVE_HARDWARE_CONNECT_TIMEOUT_SECONDS", 10.0, 60.0,
                     config.driver.connect_timeout, error) ||
      !parse_seconds("WUJI_GLOVE_HARDWARE_STREAM_TIMEOUT_SECONDS", 5.0, 60.0,
                     config.stream_timeout_seconds, error) ||
      !parse_bool("WUJI_GLOVE_HARDWARE_PUBLISH_EMF_POSES", true, config.driver.publish_emf_poses,
                  error) ||
      !parse_bool("WUJI_GLOVE_HARDWARE_PUBLISH_TIP_POSES", true, config.driver.publish_tip_poses,
                  error) ||
      !parse_bool("WUJI_GLOVE_HARDWARE_PUBLISH_SKELETON", true, config.driver.publish_skeleton,
                  error) ||
      !parse_bool("WUJI_GLOVE_HARDWARE_PUBLISH_IMU", true, config.driver.publish_imu, error) ||
      !parse_bool("WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE", true, config.driver.publish_tactile,
                  error) ||
      !parse_bool("WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE_RESIDUAL", true,
                  config.driver.publish_tactile_residual, error) ||
      !parse_bool("WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE_BINARY", true,
                  config.driver.publish_tactile_binary, error) ||
      !parse_bool("WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE_ZONES", true,
                  config.driver.publish_tactile_zones, error) ||
      !parse_bool("WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE_POINTS", true,
                  config.driver.publish_tactile_points, error)) {
    return {std::nullopt, 2, std::move(error)};
  }
  return {std::move(config), 0, {}};
}

}  // namespace wuji_glove_driver::hardware_test

#endif  // HARDWARE__HARDWARE_TEST_CONFIG_HPP_
