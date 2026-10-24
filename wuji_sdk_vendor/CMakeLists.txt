cmake_minimum_required(VERSION 3.20)
project(wuji_sdk_vendor LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/wuji_sdk_manifest.cmake")

set(WUJI_SDK_ROOT "" CACHE PATH "Path to an extracted Wuji SDK C release")

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _wuji_sdk_processor)
if(_wuji_sdk_processor MATCHES "^(x86_64|amd64)$")
  set(_wuji_sdk_asset "${WUJI_SDK_X86_64_ASSET}")
  set(_wuji_sdk_sha256 "${WUJI_SDK_X86_64_SHA256}")
  set(_wuji_sdk_machine "Advanced Micro Devices X86-64")
elseif(_wuji_sdk_processor MATCHES "^(aarch64|arm64)$")
  set(_wuji_sdk_asset "${WUJI_SDK_AARCH64_ASSET}")
  set(_wuji_sdk_sha256 "${WUJI_SDK_AARCH64_SHA256}")
  set(_wuji_sdk_machine "AArch64")
else()
  message(
    FATAL_ERROR
      "Unsupported Wuji SDK target architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(WUJI_SDK_ROOT)
  get_filename_component(_wuji_sdk_root "${WUJI_SDK_ROOT}" ABSOLUTE)
  message(STATUS "Using Wuji SDK override: ${_wuji_sdk_root}")
else()
  set(_wuji_sdk_download_dir "${CMAKE_CURRENT_BINARY_DIR}/wuji_sdk_download")
  set(_wuji_sdk_archive "${_wuji_sdk_download_dir}/${_wuji_sdk_asset}")
  set(_wuji_sdk_extract_dir "${CMAKE_CURRENT_BINARY_DIR}/wuji_sdk_extract")
  set(
    _wuji_sdk_url
    "https://github.com/wuji-technology/wuji-sdk/releases/download/v${WUJI_SDK_VERSION}/${_wuji_sdk_asset}")

  file(MAKE_DIRECTORY "${_wuji_sdk_download_dir}")
  message(STATUS "Downloading Wuji SDK ${WUJI_SDK_VERSION} from ${_wuji_sdk_url}")
  file(
    DOWNLOAD "${_wuji_sdk_url}" "${_wuji_sdk_archive}"
    EXPECTED_HASH "SHA256=${_wuji_sdk_sha256}"
    STATUS _wuji_sdk_download_status
    TLS_VERIFY ON)
  list(GET _wuji_sdk_download_status 0 _wuji_sdk_download_code)
  list(GET _wuji_sdk_download_status 1 _wuji_sdk_download_message)
  if(NOT _wuji_sdk_download_code EQUAL 0)
    message(FATAL_ERROR "Wuji SDK download failed: ${_wuji_sdk_download_message}")
  endif()

  file(REMOVE_RECURSE "${_wuji_sdk_extract_dir}")
  file(MAKE_DIRECTORY "${_wuji_sdk_extract_dir}")
  file(
    ARCHIVE_EXTRACT
    INPUT "${_wuji_sdk_archive}"
    DESTINATION "${_wuji_sdk_extract_dir}")
  string(REGEX REPLACE "\\.tar\\.gz$" "" _wuji_sdk_directory "${_wuji_sdk_asset}")
  set(_wuji_sdk_root "${_wuji_sdk_extract_dir}/${_wuji_sdk_directory}")
endif()

set(_wuji_sdk_header "${_wuji_sdk_root}/include/wuji_sdk.h")
set(_wuji_sdk_library "${_wuji_sdk_root}/lib/libwuji_sdk_c.so")
set(_wuji_sdk_license "${_wuji_sdk_root}/LICENSE")
foreach(_wuji_sdk_file IN ITEMS
    "${_wuji_sdk_header}" "${_wuji_sdk_library}" "${_wuji_sdk_license}")
  if(NOT EXISTS "${_wuji_sdk_file}")
    message(FATAL_ERROR "Wuji SDK is incomplete: missing ${_wuji_sdk_file}")
  endif()
endforeach()

find_program(_wuji_sdk_readelf NAMES readelf llvm-readelf REQUIRED)
execute_process(
  COMMAND "${_wuji_sdk_readelf}" -h "${_wuji_sdk_library}"
  RESULT_VARIABLE _wuji_sdk_readelf_result
  OUTPUT_VARIABLE _wuji_sdk_elf_header
  ERROR_VARIABLE _wuji_sdk_readelf_error)
if(NOT _wuji_sdk_readelf_result EQUAL 0)
  message(FATAL_ERROR "Unable to inspect Wuji SDK library: ${_wuji_sdk_readelf_error}")
endif()
string(REGEX MATCH "Machine:[ \t]+${_wuji_sdk_machine}" _wuji_sdk_machine_match
  "${_wuji_sdk_elf_header}")
if(NOT _wuji_sdk_machine_match)
  message(
    FATAL_ERROR
      "Wuji SDK library architecture does not match ${CMAKE_SYSTEM_PROCESSOR}")
endif()

find_program(_wuji_sdk_nm NAMES nm llvm-nm REQUIRED)
execute_process(
  COMMAND "${_wuji_sdk_nm}" -D --defined-only "${_wuji_sdk_library}"
  RESULT_VARIABLE _wuji_sdk_nm_result
  OUTPUT_VARIABLE _wuji_sdk_symbols
  ERROR_VARIABLE _wuji_sdk_nm_error)
if(NOT _wuji_sdk_nm_result EQUAL 0)
  message(FATAL_ERROR "Unable to inspect Wuji SDK symbols: ${_wuji_sdk_nm_error}")
endif()
if(NOT _wuji_sdk_symbols MATCHES "[ \t]wuji_version([\r\n]|$)")
  message(FATAL_ERROR "Wuji SDK library does not define wuji_version")
endif()

install(FILES "${_wuji_sdk_header}" DESTINATION include/wuji_sdk_vendor)
install(FILES "${_wuji_sdk_library}" DESTINATION lib)
install(FILES "${_wuji_sdk_license}" DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_test(
    NAME vendor_install_consumer
    COMMAND
      "${CMAKE_COMMAND}"
      "-DVENDOR_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}"
      "-DCONSUMER_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/consumer"
      "-DTEST_WORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/vendor_install_consumer"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/test_vendor_install.cmake")
endif()

ament_package(CONFIG_EXTRAS "cmake/wuji_sdk_vendor-extras.cmake.in")
