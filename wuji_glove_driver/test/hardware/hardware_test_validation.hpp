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
#include <string_view>

namespace wuji_glove_driver::hardware_test {

inline bool is_valid_selector(std::string_view value) noexcept {
  for (const char character : value) {
    const auto byte = static_cast<unsigned char>(character);
    if (byte <= 0x20U || byte == 0x7fU) {
      return false;
    }
  }
  return true;
}

inline bool is_optional_stream_unavailable(WujiStatus status) noexcept {
  return status == WUJI_STATUS_ERR_UNSUPPORTED || status == WUJI_STATUS_ERR_NOT_FOUND;
}

template <typename Body>
int run_hardware_exception_boundary(std::ostream& error, Body&& body) {
  try {
    return body();
  } catch (const std::exception& exception) {
    error << "FAIL: read-only Glove hardware test exception: " << exception.what() << '\n';
  } catch (...) {
    error << "FAIL: read-only Glove hardware test unknown exception\n";
  }
  return 1;
}

}  // namespace wuji_glove_driver::hardware_test

#endif  // HARDWARE__HARDWARE_TEST_VALIDATION_HPP_
