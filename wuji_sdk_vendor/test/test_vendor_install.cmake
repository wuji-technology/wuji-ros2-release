# Copyright 2026 Wuji Technology
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

cmake_minimum_required(VERSION 3.16)

foreach(required_variable VENDOR_BINARY_DIR CONSUMER_SOURCE_DIR TEST_WORK_DIR)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} is required")
  endif()
endforeach()

function(_wuji_loader_confirms_library loader_output expected_library output_variable)
  set(loader_confirmation "calling init: ${expected_library}")
  string(FIND "${loader_output}" "${loader_confirmation}" library_offset)
  if(library_offset EQUAL -1)
    set(library_confirmed FALSE)
  else()
    set(library_confirmed TRUE)
  endif()
  set(${output_variable} "${library_confirmed}" PARENT_SCOPE)
endfunction()

set(install_prefix "${TEST_WORK_DIR}/install")
set(consumer_binary_dir "${TEST_WORK_DIR}/consumer-build")
file(REMOVE_RECURSE "${TEST_WORK_DIR}")

execute_process(
  COMMAND "${CMAKE_COMMAND}" --install "${VENDOR_BINARY_DIR}" --prefix "${install_prefix}"
  RESULT_VARIABLE install_result
  OUTPUT_VARIABLE install_output
  ERROR_VARIABLE install_error)
if(NOT install_result EQUAL 0)
  message(FATAL_ERROR "Vendor install failed:\n${install_output}${install_error}")
endif()

foreach(installed_file IN ITEMS
    include/wuji_sdk_vendor/wuji_sdk.h
    lib/libwuji_sdk_c.so
    share/wuji_sdk_vendor/LICENSE
    share/wuji_sdk_vendor/cmake/wuji_sdk_vendor-extras.cmake)
  if(NOT EXISTS "${install_prefix}/${installed_file}")
    message(
      FATAL_ERROR "Vendor install is missing ${install_prefix}/${installed_file}")
  endif()
endforeach()

set(rogue_config_dir "${TEST_WORK_DIR}/rogue-prefix/share/wuji_sdk_vendor/cmake")
file(MAKE_DIRECTORY "${rogue_config_dir}")
file(
  WRITE "${rogue_config_dir}/wuji_sdk_vendorConfig.cmake"
  "message(FATAL_ERROR \"Rogue wuji_sdk_vendorConfig.cmake was selected\")\n")
set(expected_config_dir
  "${install_prefix}/share/wuji_sdk_vendor/cmake")

execute_process(
  COMMAND
    "${CMAKE_COMMAND}"
    -S "${CONSUMER_SOURCE_DIR}"
    -B "${consumer_binary_dir}"
    "-DWUJI_SDK_VENDOR_CONFIG_DIR=${expected_config_dir}"
    "-Dwuji_sdk_vendor_DIR=${rogue_config_dir}"
    -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF
    -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF
  RESULT_VARIABLE configure_result
  OUTPUT_VARIABLE configure_output
  ERROR_VARIABLE configure_error)
if(NOT configure_result EQUAL 0)
  message(
    FATAL_ERROR
      "Consumer configure failed:\n${configure_output}${configure_error}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${consumer_binary_dir}"
  RESULT_VARIABLE build_result
  OUTPUT_VARIABLE build_output
  ERROR_VARIABLE build_error)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR "Consumer build failed:\n${build_output}${build_error}")
endif()

set(consumer_executable "${consumer_binary_dir}/wuji_sdk_vendor_consumer")
execute_process(
  COMMAND
    "${CMAKE_COMMAND}" -E env
    --unset=LD_LIBRARY_PATH
    LD_DEBUG=libs
    "${consumer_executable}"
  RESULT_VARIABLE run_result
  OUTPUT_VARIABLE run_output
  ERROR_VARIABLE loader_output)
if(NOT run_result EQUAL 0)
  message(FATAL_ERROR "Consumer execution failed:\n${run_output}${loader_output}")
endif()

set(expected_library "${install_prefix}/lib/libwuji_sdk_c.so")
set(candidate_only_loader_output
  "123: trying file=${expected_library}\n")
_wuji_loader_confirms_library(
  "${candidate_only_loader_output}" "${expected_library}" candidate_only_confirmed)
if(candidate_only_confirmed)
  message(
    FATAL_ERROR
      "Candidate-only loader output was accepted as an actual library load")
endif()

_wuji_loader_confirms_library(
  "${loader_output}" "${expected_library}" loaded_library_confirmed)
if(NOT loaded_library_confirmed)
  message(
    FATAL_ERROR
      "Consumer did not load ${expected_library}. Loader output:\n${loader_output}")
endif()

message(STATUS "Consumer loaded ${expected_library}")
