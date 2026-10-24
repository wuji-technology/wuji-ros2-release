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

#ifndef HARDWARE__HARDWARE_TEST_VALIDATION_HPP_
#define HARDWARE__HARDWARE_TEST_VALIDATION_HPP_

#include <wuji_sdk.h>

#include <exception>
#include <ostream>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

namespace wuji_hand2_driver::hardware_test {

inline bool is_valid_hardware_selector(std::string_view value) noexcept {
  if (value.empty()) {
    return false;
  }
  for (const char character : value) {
    const auto byte = static_cast<unsigned char>(character);
    if (byte <= 0x20U || byte == 0x7fU) {
      return false;
    }
  }
  return true;
}

class MutationNoticeGuard {
 public:
  explicit MutationNoticeGuard(std::ostream& output) : output_(output) {
    output_ << "MUTATION NOTICE BEFORE CONNECTION: effort limits, MIT gains, and tactile "
               "baseline will be changed and NOT restored; only whole-hand disable is "
               "best-effort."
            << std::endl;
  }

  ~MutationNoticeGuard() noexcept {
    try {
      output_ << "FINAL MUTATION NOTICE: effort limits, MIT gains, and tactile baseline "
                 "are changed and NOT restored; only whole-hand disable is best-effort."
              << std::endl;
    } catch (...) {
    }
  }

  MutationNoticeGuard(const MutationNoticeGuard&) = delete;
  MutationNoticeGuard& operator=(const MutationNoticeGuard&) = delete;

 private:
  std::ostream& output_;
};

template <typename Cleanup>
class CleanupGuard {
 public:
  explicit CleanupGuard(Cleanup cleanup) : cleanup_(std::move(cleanup)) {}

  ~CleanupGuard() noexcept {
    try {
      cleanup_();
    } catch (...) {
    }
  }

  CleanupGuard(const CleanupGuard&) = delete;
  CleanupGuard& operator=(const CleanupGuard&) = delete;
  CleanupGuard(CleanupGuard&&) = delete;
  CleanupGuard& operator=(CleanupGuard&&) = delete;

 private:
  Cleanup cleanup_;
};

template <typename Cleanup>
auto make_cleanup_guard(Cleanup&& cleanup) {
  return CleanupGuard<std::decay_t<Cleanup>>{std::forward<Cleanup>(cleanup)};
}

template <typename Body>
int run_hardware_exception_boundary(std::string_view test_name, std::ostream& error, Body&& body) {
  try {
    return std::forward<Body>(body)();
  } catch (const std::exception& exception) {
    error << "FAIL: " << test_name << " exception: " << exception.what() << std::endl;
  } catch (...) {
    error << "FAIL: " << test_name << " unknown exception" << std::endl;
  }
  return 1;
}

inline bool is_optional_stream_unavailable(WujiStatus result) noexcept {
  return result == WUJI_STATUS_ERR_NOT_FOUND || result == WUJI_STATUS_ERR_UNSUPPORTED;
}

}  // namespace wuji_hand2_driver::hardware_test

#endif  // HARDWARE__HARDWARE_TEST_VALIDATION_HPP_
