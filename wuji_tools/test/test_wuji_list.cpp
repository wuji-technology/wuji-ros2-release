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

#include <cstddef>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "scan_api.hpp"

namespace wuji_tools {
namespace test {

class FakeScanApi {
 public:
  struct Device {
    std::string model{"WujiHand2"};
    std::string serial_number;
    WujiDeviceType device_id{WUJI_DEVICE_TYPE_WUJI_HAND_2};
    WujiTransportType transport{WUJI_TRANSPORT_TYPE_UDP};
    std::string address;
  };

  FakeScanApi() {
    instance_ = this;
  }

  ~FakeScanApi() {
    instance_ = nullptr;
  }

  FakeScanApi(const FakeScanApi&) = delete;
  FakeScanApi& operator=(const FakeScanApi&) = delete;

  ScanApi api() noexcept {
    return ScanApi{&FakeScanApi::init, &FakeScanApi::shutdown, &FakeScanApi::scan,
                   &FakeScanApi::discovered_free, &FakeScanApi::last_error};
  }

  static Device device(std::string model, std::string serial_number, std::string address) {
    return Device{std::move(model), std::move(serial_number), WUJI_DEVICE_TYPE_WUJI_HAND_2,
                  WUJI_TRANSPORT_TYPE_UDP, std::move(address)};
  }

  std::vector<Device> devices;
  WujiStatus status{WUJI_STATUS_OK};
  WujiStatus init_status{WUJI_STATUS_OK};
  std::string error{"scan failure"};
  bool return_result_on_error{false};
  bool null_result{false};
  bool malformed_model{false};
  bool malformed_serial_number{false};
  bool malformed_address{false};
  std::size_t null_result_count{0};
  std::size_t scan_calls{0};
  std::size_t init_calls{0};
  std::size_t shutdown_calls{0};
  std::size_t free_calls{0};
  std::size_t freed_count{0};
  static WujiStatus init(const WujiInitOptions*) {
    ++instance_->init_calls;
    return instance_->init_status;
  }

  static void shutdown() {
    ++instance_->shutdown_calls;
  }

 private:
  static FakeScanApi* instance_;

  static WujiStatus scan(WujiDiscovered** out, std::size_t* out_count) {
    ++instance_->scan_calls;
    if (instance_->null_result) {
      *out = nullptr;
      *out_count = instance_->null_result_count;
      return instance_->status;
    }

    const bool return_result =
        instance_->status == WUJI_STATUS_OK || instance_->return_result_on_error;
    if (!return_result) {
      *out = nullptr;
      *out_count = 0;
      return instance_->status;
    }

    const std::size_t size = instance_->devices.size();
    auto result = std::make_unique<WujiDiscovered[]>(size == 0 ? 1 : size);
    for (std::size_t index = 0; index < size; ++index) {
      const Device& source = instance_->devices[index];
      WujiDiscovered& destination = result[index];
      std::memset(&destination, 0, sizeof(destination));
      std::strncpy(destination.model, source.model.c_str(), sizeof(destination.model) - 1);
      std::strncpy(destination.serial_number, source.serial_number.c_str(),
                   sizeof(destination.serial_number) - 1);
      std::strncpy(destination.address, source.address.c_str(), sizeof(destination.address) - 1);
      destination.device_id = source.device_id;
      destination.transport = source.transport;
    }
    if (size > 0 && instance_->malformed_model) {
      std::memset(result[0].model, 'M', sizeof(result[0].model));
    }
    if (size > 0 && instance_->malformed_serial_number) {
      std::memset(result[0].serial_number, 'S', sizeof(result[0].serial_number));
    }
    if (size > 0 && instance_->malformed_address) {
      std::memset(result[0].address, 'A', sizeof(result[0].address));
    }
    *out = result.release();
    *out_count = size;
    return instance_->status;
  }

  static void discovered_free(WujiDiscovered* result, std::size_t count) {
    ++instance_->free_calls;
    instance_->freed_count = count;
    instance_->error = "result freed";
    delete[] result;
  }

  static const char* last_error() {
    return instance_->error.c_str();
  }
};

inline FakeScanApi* FakeScanApi::instance_ = nullptr;

}  // namespace test

namespace {

using test::FakeScanApi;

class ThrowingBuffer : public std::streambuf {
 protected:
  std::streamsize xsputn(const char*, std::streamsize) override {
    throw std::runtime_error("output failed");
  }

  int_type overflow(int_type) override {
    throw std::runtime_error("output failed");
  }
};

TEST(WujiList, PrintsMultipleDevicesInStableOrder) {
  FakeScanApi fake;
  fake.devices = {FakeScanApi::device("WujiHand2", "HAND-2", "192.0.2.20"),
                  FakeScanApi::device("WujiGlove", "GLOVE-1", "192.0.2.10")};
  fake.devices[1].device_id = WUJI_DEVICE_TYPE_WUJI_GLOVE;
  std::ostringstream output;
  std::ostringstream errors;

  EXPECT_EQ(EXIT_SUCCESS, run_wuji_list({}, fake.api(), output, errors));
  EXPECT_EQ(
      "device_type: WujiGlove\n"
      "serial_number: GLOVE-1\n"
      "hand_side: unknown\n"
      "endpoint: 192.0.2.10\n\n"
      "device_type: WujiHand2\n"
      "serial_number: HAND-2\n"
      "hand_side: unknown\n"
      "endpoint: 192.0.2.20\n",
      output.str());
  EXPECT_TRUE(errors.str().empty());
  EXPECT_EQ(1U, fake.scan_calls);
  EXPECT_EQ(1U, fake.free_calls);
}

TEST(WujiList, DerivesHandSideFromSupportedDeviceSerialMarker) {
  struct Case {
    WujiDeviceType type;
    std::string model;
    std::string serial_number;
    std::string expected_side;
  };
  const std::vector<Case> cases{
      {WUJI_DEVICE_TYPE_WUJI_HAND_2, "WujiHand2", "WH2J000000000001", "left"},
      {WUJI_DEVICE_TYPE_WUJI_HAND_2, "WujiHand2", "WH2K000000000001", "right"},
      {WUJI_DEVICE_TYPE_WUJI_GLOVE, "WujiGlove", "WGLJ000000000001", "left"},
      {WUJI_DEVICE_TYPE_WUJI_GLOVE, "WujiGlove", "WGLK000000000001", "right"},
  };

  for (const auto& test_case : cases) {
    FakeScanApi fake;
    auto device = FakeScanApi::device(test_case.model, test_case.serial_number, "udp/192.0.2.1");
    device.device_id = test_case.type;
    fake.devices = {device};
    std::ostringstream output;
    std::ostringstream errors;

    EXPECT_EQ(EXIT_SUCCESS, run_wuji_list({}, fake.api(), output, errors));
    EXPECT_NE(std::string::npos, output.str().find("hand_side: " + test_case.expected_side))
        << test_case.serial_number;
    EXPECT_TRUE(errors.str().empty());
  }
}

TEST(WujiList, CopiesLastErrorBeforeFreeOnScanFailure) {
  FakeScanApi fake;
  fake.status = WUJI_STATUS_ERR_INTERNAL;
  fake.error = "network scan failed";
  fake.return_result_on_error = true;
  fake.devices = {FakeScanApi::device("WujiHand2", "ABC", "192.0.2.1")};
  std::ostringstream output;
  std::ostringstream errors;

  EXPECT_EQ(EXIT_FAILURE, run_wuji_list({}, fake.api(), output, errors));
  EXPECT_TRUE(output.str().empty());
  EXPECT_NE(std::string::npos, errors.str().find("network scan failed"));
  EXPECT_EQ(1U, fake.free_calls);
}

TEST(WujiList, FreesANonnullResultReturnedWithScanError) {
  FakeScanApi fake;
  fake.status = WUJI_STATUS_ERR_INTERNAL;
  fake.return_result_on_error = true;
  fake.devices = {FakeScanApi::device("WujiHand2", "ABC", "192.0.2.1")};
  std::ostringstream output;
  std::ostringstream errors;

  EXPECT_EQ(EXIT_FAILURE, run_wuji_list({}, fake.api(), output, errors));
  EXPECT_EQ(1U, fake.free_calls);
  EXPECT_EQ(1U, fake.freed_count);
}

TEST(WujiList, RejectsPointerAndCountMismatchAndFreesOnlyTheReturnedPointer) {
  FakeScanApi fake;
  fake.null_result = true;
  fake.null_result_count = 1;
  std::ostringstream output;
  std::ostringstream errors;

  EXPECT_EQ(EXIT_FAILURE, run_wuji_list({}, fake.api(), output, errors));
  EXPECT_NE(std::string::npos, errors.str().find("inconsistent"));
  EXPECT_EQ(0U, fake.free_calls);
}

TEST(WujiList, RejectsControlCharactersWithoutWritingPartialOutput) {
  const std::vector<FakeScanApi::Device> unsafe_devices{
      FakeScanApi::device("Wuji\nGlove", "UNSAFE", "192.0.2.2"),
      FakeScanApi::device("WujiHand2", "BAD\rSERIAL", "192.0.2.2"),
      FakeScanApi::device("WujiHand2", "UNSAFE", "192.0.2.2\x7F")};

  for (const FakeScanApi::Device& unsafe_device : unsafe_devices) {
    FakeScanApi fake;
    fake.devices = {FakeScanApi::device("WujiHand2", "SAFE", "192.0.2.1"), unsafe_device};
    std::ostringstream output;
    std::ostringstream errors;

    EXPECT_EQ(EXIT_FAILURE, run_wuji_list({}, fake.api(), output, errors));
    EXPECT_TRUE(output.str().empty());
    EXPECT_NE(std::string::npos, errors.str().find("control character"));
    EXPECT_EQ(1U, fake.free_calls);
  }
}

TEST(WujiList, HelpDoesNotScanAndUnsupportedArgumentsFailDeterministically) {
  FakeScanApi fake;
  std::ostringstream help_output;
  std::ostringstream help_errors;

  EXPECT_EQ(EXIT_SUCCESS, run_wuji_list({"--help"}, fake.api(), help_output, help_errors));
  EXPECT_NE(std::string::npos, help_output.str().find("Usage: wuji_list"));
  EXPECT_EQ(0U, fake.scan_calls);

  std::ostringstream bad_output;
  std::ostringstream bad_errors;
  EXPECT_EQ(EXIT_FAILURE, run_wuji_list({"--unsupported"}, fake.api(), bad_output, bad_errors));
  EXPECT_TRUE(bad_output.str().empty());
  EXPECT_NE(std::string::npos, bad_errors.str().find("unsupported argument"));
  EXPECT_EQ(0U, fake.scan_calls);
}

TEST(WujiList, InitializesAndShutsDownForAnEmptyScan) {
  FakeScanApi fake;
  std::ostringstream output;
  std::ostringstream errors;

  EXPECT_EQ(EXIT_SUCCESS, run_wuji_list({}, fake.api(), output, errors));
  EXPECT_TRUE(output.str().empty());
  EXPECT_TRUE(errors.str().empty());
  EXPECT_EQ(1U, fake.init_calls);
  EXPECT_EQ(1U, fake.scan_calls);
  EXPECT_EQ(1U, fake.free_calls);
  EXPECT_EQ(1U, fake.shutdown_calls);
}

TEST(WujiList, ReportsInitFailureWithoutScanningOrShuttingDown) {
  FakeScanApi fake;
  fake.init_status = WUJI_STATUS_ERR_INTERNAL;
  fake.error = "initialization failed";
  std::ostringstream output;
  std::ostringstream errors;

  EXPECT_EQ(EXIT_FAILURE, run_wuji_list({}, fake.api(), output, errors));
  EXPECT_TRUE(output.str().empty());
  EXPECT_NE(std::string::npos, errors.str().find("wuji_init failed: initialization failed"));
  EXPECT_EQ(1U, fake.init_calls);
  EXPECT_EQ(0U, fake.scan_calls);
  EXPECT_EQ(0U, fake.shutdown_calls);
}

TEST(WujiList, FreesResultsAndShutsDownWhenOutputThrows) {
  FakeScanApi fake;
  fake.devices = {FakeScanApi::device("WujiHand2", "HAND-2", "192.0.2.20")};
  ThrowingBuffer buffer;
  std::ostream output(&buffer);
  output.exceptions(std::ios::badbit);
  std::ostringstream errors;

  EXPECT_EQ(EXIT_FAILURE, run_wuji_list({}, fake.api(), output, errors));
  EXPECT_NE(std::string::npos, errors.str().find("unexpected failure: output failed"));
  EXPECT_EQ(1U, fake.init_calls);
  EXPECT_EQ(1U, fake.scan_calls);
  EXPECT_EQ(1U, fake.free_calls);
  EXPECT_EQ(1U, fake.shutdown_calls);
}

}  // namespace
}  // namespace wuji_tools
