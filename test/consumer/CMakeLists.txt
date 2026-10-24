cmake_minimum_required(VERSION 3.16)
project(wuji_sdk_vendor_consumer LANGUAGES CXX)

if(NOT WUJI_SDK_VENDOR_CONFIG_DIR)
  message(FATAL_ERROR "WUJI_SDK_VENDOR_CONFIG_DIR is required")
endif()
get_filename_component(
  _wuji_sdk_vendor_config_dir "${WUJI_SDK_VENDOR_CONFIG_DIR}" ABSOLUTE)
set(
  wuji_sdk_vendor_DIR "${_wuji_sdk_vendor_config_dir}"
  CACHE PATH "Hermetic wuji_sdk_vendor config directory" FORCE)
find_package(
  wuji_sdk_vendor CONFIG REQUIRED
  PATHS "${_wuji_sdk_vendor_config_dir}"
  NO_DEFAULT_PATH)

add_executable(wuji_sdk_vendor_consumer main.cpp)
target_compile_features(wuji_sdk_vendor_consumer PRIVATE cxx_std_17)
set_target_properties(
  wuji_sdk_vendor_consumer
  PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN YES)
target_link_libraries(wuji_sdk_vendor_consumer PRIVATE wuji_sdk_vendor::wuji_sdk_c)
