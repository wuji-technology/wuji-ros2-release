# Copyright 2026 Wuji Technology

add_library(
  wuji_driver_common_stream STATIC
  src/stream/stream_controller.cpp
  src/stream/stream_registry.cpp
)

target_include_directories(
  wuji_driver_common_stream
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(wuji_driver_common_stream PUBLIC cxx_std_17)

target_link_libraries(
  wuji_driver_common_stream
  PUBLIC wuji_sdk_vendor::wuji_sdk_c
)

set_target_properties(
  wuji_driver_common_stream
  PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    POSITION_INDEPENDENT_CODE ON
    VISIBILITY_INLINES_HIDDEN YES
)
