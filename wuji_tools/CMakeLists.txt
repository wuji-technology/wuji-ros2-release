cmake_minimum_required(VERSION 3.20)
project(wuji_tools LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(wuji_sdk_vendor REQUIRED)

add_library(wuji_tools_impl STATIC src/scan_api.cpp src/wuji_list.cpp)
target_compile_features(wuji_tools_impl PUBLIC cxx_std_17)
target_compile_definitions(wuji_tools_impl PRIVATE WUJI_TOOLS_NO_MAIN)
target_include_directories(
  wuji_tools_impl
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(wuji_tools_impl PUBLIC wuji_sdk_vendor::wuji_sdk_c)
set_target_properties(
  wuji_tools_impl
  PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    POSITION_INDEPENDENT_CODE ON
    VISIBILITY_INLINES_HIDDEN YES
)

add_executable(wuji_list src/wuji_list.cpp)
target_compile_features(wuji_list PRIVATE cxx_std_17)
target_link_libraries(wuji_list PRIVATE wuji_tools_impl)
install(TARGETS wuji_list DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_wuji_list test/test_wuji_list.cpp)
  target_compile_features(test_wuji_list PRIVATE cxx_std_17)
  target_include_directories(
    test_wuji_list
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(test_wuji_list wuji_tools_impl)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
