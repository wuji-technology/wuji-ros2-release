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
#include <sstream>
#include <stdexcept>
#include <string>

#include "hardware_test_config.hpp"
#include "hardware_test_validation.hpp"

namespace wuji_hand2_driver::hardware_test {
namespace {

constexpr const char* kSafetyAck = "fixture-unloaded-operator-state-not-restored";

class ScopedHardwareEnvironment {
 public:
  ScopedHardwareEnvironment() {
    for (std::size_t index = 0; index < kNames.size(); ++index) {
      const char* value = std::getenv(kNames[index]);
      if (value != nullptr) {
        original_[index] = value;
      }
      (void)unsetenv(kNames[index]);
    }
  }

  ~ScopedHardwareEnvironment() {
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
  static constexpr std::array<const char*, 6> kNames = {
      "WUJI_HARDWARE_TESTS",  "WUJI_MUTATING_HARDWARE_TESTS", "WUJI_HARDWARE_SAFETY_ACK",
      "WUJI_HARDWARE_SERIAL", "WUJI_HARDWARE_ENDPOINT",       "WUJI_HARDWARE_RUNNER",
  };
  std::array<std::optional<std::string>, kNames.size()> original_{};
};

void enable_mutating_config(ScopedHardwareEnvironment& environment) {
  environment.set("WUJI_HARDWARE_TESTS", "1");
  environment.set("WUJI_MUTATING_HARDWARE_TESTS", "1");
  environment.set("WUJI_HARDWARE_SAFETY_ACK", kSafetyAck);
  environment.set("WUJI_HARDWARE_SERIAL", "SN123");
}

TEST(HardwareTestValidation, RejectsWhitespaceAndAsciiControlsInSelectors) {
  EXPECT_TRUE(is_valid_hardware_selector("SN123"));
  EXPECT_TRUE(is_valid_hardware_selector("192.0.2.10"));

  const std::array<std::string, 7> invalid_values = {
      "", " SN123", "SN123 ", "SN 123", "SN\t123", "SN\n123", std::string{"SN"} + '\x7f' + "123",
  };
  for (const std::string& value : invalid_values) {
    EXPECT_FALSE(is_valid_hardware_selector(value)) << value;
  }
}

TEST(HardwareTestValidation, OptionalAbsenceExcludesTimeoutAndCommunicationFailures) {
  EXPECT_TRUE(is_optional_stream_unavailable(WUJI_STATUS_ERR_NOT_FOUND));
  EXPECT_TRUE(is_optional_stream_unavailable(WUJI_STATUS_ERR_UNSUPPORTED));

  EXPECT_FALSE(is_optional_stream_unavailable(WUJI_STATUS_ERR_TIMEOUT));
  EXPECT_FALSE(is_optional_stream_unavailable(WUJI_STATUS_ERR_PROTOCOL));
  EXPECT_FALSE(is_optional_stream_unavailable(WUJI_STATUS_ERR_DISCONNECTED));
}

TEST(HardwareTestConfig, RejectsAmbiguousMutatingSelector) {
  ScopedHardwareEnvironment environment;
  enable_mutating_config(environment);
  environment.set("WUJI_HARDWARE_ENDPOINT", "192.0.2.10");

  const auto result = load_hardware_test_config(true);

  EXPECT_FALSE(result.config);
  EXPECT_EQ(result.exit_code, 2);
}

TEST(HardwareTestConfig, RejectsMalformedReadOnlySelector) {
  ScopedHardwareEnvironment environment;
  environment.set("WUJI_HARDWARE_TESTS", "1");
  environment.set("WUJI_HARDWARE_SERIAL", "SN 123");

  const auto result = load_hardware_test_config(false);

  EXPECT_FALSE(result.config);
  EXPECT_EQ(result.exit_code, 2);
}

TEST(HardwareTestConfig, RequiresExactMutatingSafetyAcknowledgement) {
  ScopedHardwareEnvironment environment;
  environment.set("WUJI_HARDWARE_TESTS", "1");
  environment.set("WUJI_MUTATING_HARDWARE_TESTS", "1");
  environment.set("WUJI_HARDWARE_SERIAL", "SN123");

  auto result = load_hardware_test_config(true);
  EXPECT_FALSE(result.config);
  EXPECT_EQ(result.exit_code, 3);

  environment.set("WUJI_HARDWARE_SAFETY_ACK", "fixture-unloaded-operator");
  result = load_hardware_test_config(true);
  EXPECT_FALSE(result.config);
  EXPECT_EQ(result.exit_code, 3);
}

TEST(HardwareTestConfig, AcceptsExactMutatingSafetyAcknowledgement) {
  ScopedHardwareEnvironment environment;
  enable_mutating_config(environment);

  const auto result = load_hardware_test_config(true);

  ASSERT_TRUE(result.config);
  EXPECT_EQ(result.config->serial_number, "SN123");
  EXPECT_TRUE(result.config->endpoint.empty());
}

TEST(MutationNoticeGuard, PrintsPreConnectAndFinalNonRestorationWarnings) {
  std::ostringstream output;
  {
    MutationNoticeGuard notice(output);
    const std::string before_connection = output.str();
    EXPECT_NE(before_connection.find("BEFORE CONNECTION"), std::string::npos);
    EXPECT_NE(
        before_connection.find(
            "effort limits, MIT gains, and tactile baseline will be changed and NOT restored"),
        std::string::npos);
    EXPECT_NE(before_connection.find("only whole-hand disable is best-effort"), std::string::npos);
    EXPECT_EQ(before_connection.find("FINAL MUTATION NOTICE"), std::string::npos);
  }

  const std::string final_output = output.str();
  EXPECT_NE(final_output.find("FINAL MUTATION NOTICE"), std::string::npos);
  EXPECT_NE(final_output.find(
                "effort limits, MIT gains, and tactile baseline are changed and NOT restored"),
            std::string::npos);
}

TEST(HardwareExceptionBoundary, UnwindsCleanupAndNoticeBeforeReportingException) {
  std::ostringstream output;
  std::ostringstream error;
  bool cleanup_called = false;

  const int result =
      run_hardware_exception_boundary("mutating Hand 2 hardware test", error, [&]() -> int {
        MutationNoticeGuard notice(output);
        auto cleanup = make_cleanup_guard([&]() noexcept {
          cleanup_called = true;
          output << "CLEANUP COMPLETE" << std::endl;
        });
        (void)cleanup;
        throw std::runtime_error("injected SDK failure");
      });

  EXPECT_EQ(result, 1);
  EXPECT_TRUE(cleanup_called);
  const std::string captured_output = output.str();
  const auto before = captured_output.find("MUTATION NOTICE BEFORE CONNECTION");
  const auto cleanup = captured_output.find("CLEANUP COMPLETE");
  const auto final = captured_output.find("FINAL MUTATION NOTICE");
  ASSERT_NE(before, std::string::npos);
  ASSERT_NE(cleanup, std::string::npos);
  ASSERT_NE(final, std::string::npos);
  EXPECT_LT(before, cleanup);
  EXPECT_LT(cleanup, final);
  EXPECT_EQ(error.str(), "FAIL: mutating Hand 2 hardware test exception: injected SDK failure\n");
}

TEST(HardwareExceptionBoundary, ReportsUnknownExceptionDeterministically) {
  std::ostringstream error;

  const int result = run_hardware_exception_boundary("mutating Hand 2 hardware test", error,
                                                     []() -> int { throw 17; });

  EXPECT_EQ(result, 1);
  EXPECT_EQ(error.str(), "FAIL: mutating Hand 2 hardware test unknown exception\n");
}

}  // namespace
}  // namespace wuji_hand2_driver::hardware_test
