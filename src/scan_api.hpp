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

#ifndef SCAN_API_HPP_
#define SCAN_API_HPP_

#include <wuji_sdk.h>

#include <iosfwd>
#include <string_view>
#include <vector>

namespace wuji_tools {

struct ScanApi {
  decltype(&wuji_init) init;
  decltype(&wuji_shutdown) shutdown;
  decltype(&wuji_scan) scan;
  decltype(&wuji_discovered_free) discovered_free;
  decltype(&wuji_last_error) last_error;
};

ScanApi production_scan_api() noexcept;

int run_wuji_list(const std::vector<std::string_view>& args, const ScanApi& api,
                  std::ostream& output, std::ostream& errors);

}  // namespace wuji_tools

#endif  // SCAN_API_HPP_
