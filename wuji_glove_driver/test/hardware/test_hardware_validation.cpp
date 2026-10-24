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

#include <array>
#include <cstdlib>
#include <optional>
#include <string>

#include "hardware_test_config.hpp"
#include "hardware_test_validation.hpp"

namespace wuji_glove_driver::hardware_test {
namespace {

class ScopedEnvironment {
 public:
  ScopedEnvironment() {
    for (std::size_t index = 0; index < kNames.size(); ++index) {
      if (const char* value = std::getenv(kNames[index]); value != nullptr) {
        original_[index] = value;
      }
      (void)unsetenv(kNames[index]);
    }
  }
  ~ScopedEnvironment() {
    for (std::size_t index = 0; index < kNames.size(); ++index) {
      if (original_[index]) {
        (void)setenv(kNames[index], original_[index]->c_str(), 1);
      } else {
        (void)unsetenv(kNames[index]);
      }
    }
  }
  void set(const char* name, const char* value) {
    ASSERT_EQ(setenv(name, value, 1), 0);
  }

 private:
  static constexpr std::array<const char*, 15> kNames{
      "WUJI_GLOVE_HARDWARE_TESTS",
      "WUJI_GLOVE_HARDWARE_SERIAL",
      "WUJI_GLOVE_HARDWARE_ENDPOINT",
      "WUJI_GLOVE_HARDWARE_HAND_SIDE",
      "WUJI_GLOVE_HARDWARE_CONNECT_TIMEOUT_SECONDS",
      "WUJI_GLOVE_HARDWARE_STREAM_TIMEOUT_SECONDS",
      "WUJI_GLOVE_HARDWARE_PUBLISH_EMF_POSES",
      "WUJI_GLOVE_HARDWARE_PUBLISH_TIP_POSES",
      "WUJI_GLOVE_HARDWARE_PUBLISH_SKELETON",
      "WUJI_GLOVE_HARDWARE_PUBLISH_IMU",
      "WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE",
      "WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE_RESIDUAL",
      "WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE_BINARY",
      "WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE_ZONES",
      "WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE_POINTS",
  };
  std::array<std::optional<std::string>, kNames.size()> original_{};
};

TEST(HardwareTestConfig, ReturnsSkipWithoutExplicitOptIn) {
  ScopedEnvironment environment;
  const auto result = load_hardware_test_config();
  EXPECT_FALSE(result.config);
  EXPECT_EQ(result.exit_code, 77);
}

TEST(HardwareTestConfig, AcceptsUniqueDiscoveryWithoutSelectors) {
  ScopedEnvironment environment;
  environment.set("WUJI_GLOVE_HARDWARE_TESTS", "1");
  const auto result = load_hardware_test_config();
  ASSERT_TRUE(result.config) << result.message;
  EXPECT_TRUE(result.config->driver.serial_number.empty());
  EXPECT_TRUE(result.config->driver.endpoint.empty());
  EXPECT_TRUE(result.config->driver.hand_side.empty());
}

TEST(HardwareTestConfig, AcceptsSerialEndpointAndHandSide) {
  ScopedEnvironment environment;
  environment.set("WUJI_GLOVE_HARDWARE_TESTS", "1");
  environment.set("WUJI_GLOVE_HARDWARE_SERIAL", "WUJKSN123");
  environment.set("WUJI_GLOVE_HARDWARE_ENDPOINT", "192.0.2.11");
  environment.set("WUJI_GLOVE_HARDWARE_HAND_SIDE", "right");
  const auto result = load_hardware_test_config();
  ASSERT_TRUE(result.config) << result.message;
  EXPECT_EQ(result.config->driver.serial_number, "WUJKSN123");
  EXPECT_EQ(result.config->driver.endpoint, "192.0.2.11");
  EXPECT_EQ(result.config->driver.hand_side, "right");
}

TEST(HardwareTestConfig, ValidatesSelectorsSideAndFlags) {
  ScopedEnvironment environment;
  environment.set("WUJI_GLOVE_HARDWARE_TESTS", "1");
  environment.set("WUJI_GLOVE_HARDWARE_SERIAL", "SN 123");
  EXPECT_EQ(load_hardware_test_config().exit_code, 2);
  environment.set("WUJI_GLOVE_HARDWARE_SERIAL", "SN123");
  environment.set("WUJI_GLOVE_HARDWARE_HAND_SIDE", "upper");
  EXPECT_EQ(load_hardware_test_config().exit_code, 2);
  environment.set("WUJI_GLOVE_HARDWARE_HAND_SIDE", "left");
  environment.set("WUJI_GLOVE_HARDWARE_PUBLISH_IMU", "yes");
  EXPECT_EQ(load_hardware_test_config().exit_code, 2);
}

TEST(HardwareTestConfig, AppliesRepresentativePublishFlagCombinations) {
  ScopedEnvironment environment;
  environment.set("WUJI_GLOVE_HARDWARE_TESTS", "1");
  environment.set("WUJI_GLOVE_HARDWARE_PUBLISH_IMU", "false");
  environment.set("WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE", "0");
  environment.set("WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE_POINTS", "1");
  auto result = load_hardware_test_config();
  ASSERT_TRUE(result.config) << result.message;
  EXPECT_FALSE(result.config->driver.publish_imu);
  EXPECT_FALSE(result.config->driver.publish_tactile);
  EXPECT_TRUE(result.config->driver.publish_tactile_points);

  environment.set("WUJI_GLOVE_HARDWARE_PUBLISH_IMU", "true");
  environment.set("WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE", "1");
  environment.set("WUJI_GLOVE_HARDWARE_PUBLISH_TACTILE_POINTS", "false");
  result = load_hardware_test_config();
  ASSERT_TRUE(result.config) << result.message;
  EXPECT_TRUE(result.config->driver.publish_imu);
  EXPECT_TRUE(result.config->driver.publish_tactile);
  EXPECT_FALSE(result.config->driver.publish_tactile_points);
}

TEST(HardwareTestValidation, OptionalSkipIsLimitedToExplicitAvailabilityStatuses) {
  EXPECT_TRUE(is_optional_stream_unavailable(WUJI_STATUS_ERR_UNSUPPORTED));
  EXPECT_TRUE(is_optional_stream_unavailable(WUJI_STATUS_ERR_NOT_FOUND));
  EXPECT_FALSE(is_optional_stream_unavailable(WUJI_STATUS_ERR_TIMEOUT));
  EXPECT_FALSE(is_optional_stream_unavailable(WUJI_STATUS_ERR_PROTOCOL));
}

}  // namespace
}  // namespace wuji_glove_driver::hardware_test
