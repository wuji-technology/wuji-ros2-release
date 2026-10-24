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

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <optional>
#include <ostream>
#include <string>
#include <tuple>
#include <utility>

#include "scan_api.hpp"

namespace wuji_tools {
namespace {

constexpr std::size_t k_max_error_length = 4096;

class DiscoveredResults {
 public:
  DiscoveredResults(const ScanApi& api, WujiDiscovered* devices, std::size_t count) noexcept
      : api_(api), devices_(devices), count_(count) {}

  ~DiscoveredResults() noexcept {
    if (devices_ != nullptr) {
      try {
        api_.discovered_free(devices_, count_);
      } catch (...) {
      }
    }
  }

  DiscoveredResults(const DiscoveredResults&) = delete;
  DiscoveredResults& operator=(const DiscoveredResults&) = delete;

 private:
  const ScanApi& api_;
  WujiDiscovered* devices_;
  std::size_t count_;
};

class SdkRuntime {
 public:
  explicit SdkRuntime(const ScanApi& api)
      : api_(api), active_(api.init(nullptr) == WUJI_STATUS_OK) {}

  ~SdkRuntime() noexcept {
    if (active_) {
      try {
        api_.shutdown();
      } catch (...) {
      }
    }
  }

  bool initialized() const noexcept {
    return active_;
  }

 private:
  const ScanApi& api_;
  bool active_{false};
};

struct ListedDevice {
  std::string device_type;
  std::string serial_number;
  std::string hand_side;
  std::string endpoint;
};

std::string copy_last_error(const ScanApi& api) {
  const char* const error = api.last_error();
  if (error == nullptr) {
    return "SDK did not provide an error message";
  }

  const void* const terminator = std::memchr(error, '\0', k_max_error_length);
  if (terminator == nullptr) {
    return "SDK returned a malformed error message";
  }
  return std::string(error, static_cast<const char*>(terminator));
}

template <std::size_t Size>
std::optional<std::string> fixed_string(const char (&field)[Size]) {
  const void* const terminator = std::memchr(field, '\0', Size);
  if (terminator == nullptr) {
    return std::nullopt;
  }
  return std::string(field, static_cast<const char*>(terminator));
}

const char* device_type_name(WujiDeviceType device_type) noexcept {
  switch (device_type) {
    case WUJI_DEVICE_TYPE_WUJI_GLOVE:
      return "WujiGlove";
    case WUJI_DEVICE_TYPE_WUJI_HAND_2:
      return "WujiHand2";
    case WUJI_DEVICE_TYPE_WUJI_HAND:
      return "WujiHand";
    case WUJI_DEVICE_TYPE_UNKNOWN:
    default:
      return "Unknown";
  }
}

const char* hand_side_name(WujiDeviceType device_type, const std::string& serial_number) noexcept {
  if (device_type != WUJI_DEVICE_TYPE_WUJI_HAND_2 && device_type != WUJI_DEVICE_TYPE_WUJI_GLOVE) {
    return "unknown";
  }
  if (serial_number.size() <= 3U) {
    return "unknown";
  }
  return serial_number[3] == 'J' ? "left" : serial_number[3] == 'K' ? "right" : "unknown";
}

bool contains_control_character(const std::string& value) noexcept {
  return std::any_of(value.begin(), value.end(), [](const unsigned char character) {
    return character < 0x20U || character == 0x7FU;
  });
}

void print_usage(std::ostream& output) {
  output << "Usage: wuji_list [--help]\n";
}

void print_error(std::ostream& errors, const std::string& message) {
  errors << "wuji_list: " << message << '\n';
}

}  // namespace

int run_wuji_list(const std::vector<std::string_view>& args, const ScanApi& api,
                  std::ostream& output, std::ostream& errors) {
  try {
    if (args.size() == 1 && args.front() == "--help") {
      print_usage(output);
      return EXIT_SUCCESS;
    }
    if (!args.empty()) {
      print_error(errors, "unsupported argument");
      return EXIT_FAILURE;
    }

    SdkRuntime runtime(api);
    if (!runtime.initialized()) {
      print_error(errors, "wuji_init failed: " + copy_last_error(api));
      return EXIT_FAILURE;
    }

    WujiDiscovered* devices = nullptr;
    std::size_t count = 0;
    const WujiStatus status = api.scan(&devices, &count);
    DiscoveredResults results(api, devices, count);
    if (status != WUJI_STATUS_OK) {
      print_error(errors, "scan failed: " + copy_last_error(api));
      return EXIT_FAILURE;
    }
    if (devices == nullptr && count != 0) {
      print_error(errors, "scan returned inconsistent device pointer and count");
      return EXIT_FAILURE;
    }

    std::vector<ListedDevice> listed_devices;
    listed_devices.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
      const WujiDiscovered& device = devices[index];
      const auto model = fixed_string(device.model);
      const auto serial_number = fixed_string(device.serial_number);
      const auto address = fixed_string(device.address);
      if (!model || !serial_number || !address) {
        print_error(errors, "scan returned a field that is not NUL-terminated");
        return EXIT_FAILURE;
      }
      if (contains_control_character(*model) || contains_control_character(*serial_number) ||
          contains_control_character(*address)) {
        print_error(errors, "scan returned a field with a control character");
        return EXIT_FAILURE;
      }
      const std::string side = hand_side_name(device.device_id, *serial_number);
      listed_devices.push_back(
          ListedDevice{model->empty() ? device_type_name(device.device_id) : std::move(*model),
                       std::move(*serial_number), side, std::move(*address)});
    }

    std::sort(listed_devices.begin(), listed_devices.end(),
              [](const ListedDevice& left, const ListedDevice& right) {
                return std::tie(left.device_type, left.serial_number, left.endpoint) <
                       std::tie(right.device_type, right.serial_number, right.endpoint);
              });
    for (std::size_t index = 0; index < listed_devices.size(); ++index) {
      const ListedDevice& device = listed_devices[index];
      output << "device_type: " << device.device_type << '\n';
      output << "serial_number: " << device.serial_number << '\n';
      output << "hand_side: " << device.hand_side << '\n';
      output << "endpoint: " << device.endpoint << '\n';
      if (index + 1 < listed_devices.size()) {
        output << '\n';
      }
    }
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    try {
      print_error(errors, std::string("unexpected failure: ") + error.what());
    } catch (...) {
    }
    return EXIT_FAILURE;
  } catch (...) {
    try {
      print_error(errors, "unexpected failure");
    } catch (...) {
    }
    return EXIT_FAILURE;
  }
}

}  // namespace wuji_tools

#ifndef WUJI_TOOLS_NO_MAIN
int main(int argc, char** argv) {
  std::vector<std::string_view> args;
  args.reserve(argc > 1 ? static_cast<std::size_t>(argc - 1) : 0U);
  for (int index = 1; index < argc; ++index) {
    args.emplace_back(argv[index]);
  }
  return wuji_tools::run_wuji_list(args, wuji_tools::production_scan_api(), std::cout, std::cerr);
}
#endif
