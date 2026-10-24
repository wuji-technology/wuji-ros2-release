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

#include "wuji_glove_driver/conversion/point_cloud_conversion.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <limits>
#include <string>
#include <utility>

#include <sensor_msgs/msg/point_field.hpp>
#include "wuji_glove_driver/conversion/ros_header_conversion.hpp"

namespace wuji_glove_driver::conversion {
namespace {
Status point_field_datatype(std::uint8_t input, std::uint8_t& output) {
  switch (input) {
    case 1U:
      output = sensor_msgs::msg::PointField::UINT8;
      return Status::success();
    case 2U:
      output = sensor_msgs::msg::PointField::INT8;
      return Status::success();
    case 3U:
      output = sensor_msgs::msg::PointField::UINT16;
      return Status::success();
    case 4U:
      output = sensor_msgs::msg::PointField::INT16;
      return Status::success();
    case 5U:
      output = sensor_msgs::msg::PointField::UINT32;
      return Status::success();
    case 6U:
      output = sensor_msgs::msg::PointField::INT32;
      return Status::success();
    case 7U:
      output = sensor_msgs::msg::PointField::FLOAT32;
      return Status::success();
    case 8U:
      output = sensor_msgs::msg::PointField::FLOAT64;
      return Status::success();
    default:
      return Status::failure("unknown SDK point field datatype");
  }
}

template <std::size_t Size>
Status fixed_string(const char (&input)[Size], std::string& output) {
  const auto terminator = std::find(std::begin(input), std::end(input), '\0');
  if (terminator == std::end(input)) {
    return Status::failure("SDK string is not NUL-terminated");
  }
  output.assign(input, terminator);
  return Status::success();
}
}  // namespace

Status to_point_cloud2(const WujiPointCloud& input, sensor_msgs::msg::PointCloud2& output) {
  if (input.point_stride == 0U) {
    return Status::failure("point stride must be nonzero");
  }
  if (input.data_len % input.point_stride != 0U) {
    return Status::failure("point cloud data length is not divisible by point stride");
  }
  if (input.fields_len != 0U && input.fields == nullptr) {
    return Status::failure("point cloud fields array is null");
  }
  if (input.data_len != 0U && input.data == nullptr) {
    return Status::failure("point cloud data array is null");
  }
  const std::size_t point_count = input.data_len / input.point_stride;
  if (input.data_len > std::numeric_limits<std::uint32_t>::max() ||
      point_count > std::numeric_limits<std::uint32_t>::max()) {
    return Status::failure("point cloud dimensions exceed ROS uint32 fields");
  }

  sensor_msgs::msg::PointCloud2 converted;
  std_msgs::msg::Header converted_header;
  const Status header_status = to_ros_header(input.header, converted_header);
  if (!header_status) {
    return header_status;
  }
  const Status frame_status = fixed_string(input.frame_id, converted.header.frame_id);
  if (!frame_status) {
    return frame_status;
  }
  converted.header.stamp = converted_header.stamp;
  converted.height = 1U;
  converted.width = static_cast<std::uint32_t>(point_count);
  converted.is_bigendian = false;
  converted.point_step = input.point_stride;
  converted.row_step = static_cast<std::uint32_t>(input.data_len);
  converted.is_dense = false;
  converted.fields.reserve(input.fields_len);
  for (std::size_t index = 0; index < input.fields_len; ++index) {
    sensor_msgs::msg::PointField field;
    const Status name_status = fixed_string(input.fields[index].name, field.name);
    if (!name_status) {
      return name_status;
    }
    field.offset = input.fields[index].offset;
    const Status datatype_status = point_field_datatype(input.fields[index].type_, field.datatype);
    if (!datatype_status) {
      return datatype_status;
    }
    field.count = 1U;
    converted.fields.push_back(std::move(field));
  }
  if (input.data_len != 0U) {
    converted.data.assign(input.data, input.data + input.data_len);
  }
  output = std::move(converted);
  return Status::success();
}
}  // namespace wuji_glove_driver::conversion
