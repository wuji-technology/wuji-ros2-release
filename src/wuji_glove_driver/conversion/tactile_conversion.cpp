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
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "wuji_glove_driver/conversion/tactile_conversion.hpp"

#include <array>
#include <cstddef>
#include <utility>

#include "wuji_glove_driver/conversion/ros_header_conversion.hpp"

namespace wuji_glove_driver::conversion {
namespace {
constexpr std::size_t kTactileRows = 24U;
constexpr std::size_t kTactileColumns = 31U;
constexpr std::size_t kTactileValueCount = kTactileRows * kTactileColumns;

template <typename TactileFrame>
Status convert_grid(const TactileFrame& input, wuji_msgs::msg::TactileGrid& output) {
  if (input.data_len != kTactileValueCount) {
    return Status::failure("tactile data array must contain 744 values");
  }
  if (input.data == nullptr) {
    return Status::failure("tactile data array is null");
  }
  wuji_msgs::msg::TactileGrid converted;
  const Status header_status = to_ros_header(input.header, converted.header);
  if (!header_status) {
    return header_status;
  }
  converted.rows = kTactileRows;
  converted.columns = kTactileColumns;
  converted.data.assign(input.data, input.data + input.data_len);
  output = std::move(converted);
  return Status::success();
}
}  // namespace

Status to_tactile_grid(const WujiTactileFrame& input, wuji_msgs::msg::TactileGrid& output) {
  return convert_grid(input, output);
}
Status to_tactile_grid(const WujiTactileResidual& input, wuji_msgs::msg::TactileGrid& output) {
  return convert_grid(input, output);
}
Status to_tactile_grid(const WujiTactileBinary& input, wuji_msgs::msg::TactileGrid& output) {
  return convert_grid(input, output);
}
Status to_tactile_zones(const WujiTactileZones& input, wuji_msgs::msg::TactileZoneArray& output) {
  const std::array<const float*, 6> values{input.palm,   input.thumb, input.index,
                                           input.middle, input.ring,  input.pinky};
  const std::array<std::size_t, 6> lengths{input.palm_len,   input.thumb_len, input.index_len,
                                           input.middle_len, input.ring_len,  input.pinky_len};
  constexpr std::array<const char*, 6> names{"palm", "thumb", "index", "middle", "ring", "pinky"};
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (lengths[index] != 0U && values[index] == nullptr) {
      return Status::failure("tactile zone array is null");
    }
  }
  wuji_msgs::msg::TactileZoneArray converted;
  const Status header_status = to_ros_header(input.header, converted.header);
  if (!header_status) {
    return header_status;
  }
  converted.zones.resize(names.size());
  for (std::size_t index = 0; index < names.size(); ++index) {
    converted.zones[index].name = names[index];
    if (lengths[index] != 0U) {
      converted.zones[index].values.assign(values[index], values[index] + lengths[index]);
    }
  }
  output = std::move(converted);
  return Status::success();
}
}  // namespace wuji_glove_driver::conversion
