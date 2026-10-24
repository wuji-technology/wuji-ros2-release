cmake_minimum_required(VERSION 3.20)
project(wuji_glove_driver LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wuji_sdk_vendor REQUIRED)
find_package(wuji_msgs REQUIRED)

add_subdirectory(driver_common)

add_library(
  wuji_glove_driver_impl STATIC
  src/wuji_glove_driver/driver_config.cpp
  src/wuji_glove_driver/glove_node.cpp
  src/wuji_glove_driver/sdk/glove_sdk_api.cpp
  src/wuji_glove_driver/sdk/glove_session.cpp
  src/wuji_glove_driver/conversion/joint_angles_conversion.cpp
  src/wuji_glove_driver/conversion/imu_conversion.cpp
  src/wuji_glove_driver/conversion/point_cloud_conversion.cpp
  src/wuji_glove_driver/conversion/pose_conversion.cpp
  src/wuji_glove_driver/conversion/ros_header_conversion.cpp
  src/wuji_glove_driver/conversion/tactile_conversion.cpp
  src/wuji_glove_driver/stream/emf_poses_stream.cpp
  src/wuji_glove_driver/stream/imu_stream.cpp
  src/wuji_glove_driver/stream/joint_angles_stream.cpp
  src/wuji_glove_driver/stream/skeleton_stream.cpp
  src/wuji_glove_driver/stream/tactile_binary_stream.cpp
  src/wuji_glove_driver/stream/tactile_point_cloud_stream.cpp
  src/wuji_glove_driver/stream/tactile_residual_stream.cpp
  src/wuji_glove_driver/stream/tactile_stream.cpp
  src/wuji_glove_driver/stream/tactile_zones_stream.cpp
  src/wuji_glove_driver/stream/tip_poses_stream.cpp
)
target_include_directories(
  wuji_glove_driver_impl
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_compile_features(wuji_glove_driver_impl PUBLIC cxx_std_17)
target_link_libraries(
  wuji_glove_driver_impl
  PUBLIC
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    wuji_driver_common_stream
    std_msgs::std_msgs
    wuji_sdk_vendor::wuji_sdk_c
    wuji_msgs::wuji_msgs
)
set_target_properties(
  wuji_glove_driver_impl
  PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    POSITION_INDEPENDENT_CODE ON
    VISIBILITY_INLINES_HIDDEN YES
)

add_executable(wuji_glove_node src/main.cpp)
target_compile_features(wuji_glove_node PRIVATE cxx_std_17)
target_link_libraries(wuji_glove_node PRIVATE rclcpp::rclcpp wuji_glove_driver_impl)

install(TARGETS wuji_glove_node RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(
    test_glove_conversion
    test/unit/test_glove_conversion.cpp
  )
  target_compile_features(test_glove_conversion PRIVATE cxx_std_17)
  target_include_directories(
    test_glove_conversion
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(
    test_glove_conversion
    wuji_glove_driver_impl
    sensor_msgs::sensor_msgs
    std_msgs::std_msgs
    wuji_msgs::wuji_msgs
  )

  ament_add_gtest(
    test_glove_sdk_api
    test/unit/test_glove_sdk_api.cpp
  )
  target_compile_features(test_glove_sdk_api PRIVATE cxx_std_17)
  target_include_directories(
    test_glove_sdk_api
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(
    test_glove_sdk_api
    wuji_glove_driver_impl
    wuji_driver_common_stream
    wuji_sdk_vendor::wuji_sdk_c
  )

  ament_add_gtest(
    test_glove_session
    test/fake_sdk/fake_glove_sdk.cpp
    test/unit/test_glove_session.cpp
  )
  target_compile_features(test_glove_session PRIVATE cxx_std_17)
  target_include_directories(
    test_glove_session
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(
    test_glove_session
    rclcpp::rclcpp
    wuji_glove_driver_impl
  )
  set_tests_properties(test_glove_session PROPERTIES RUN_SERIAL TRUE)

  ament_add_gtest(
    test_glove_streams
    test/unit/test_glove_streams.cpp
  )
  target_compile_features(test_glove_streams PRIVATE cxx_std_17)
  target_include_directories(
    test_glove_streams
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(
    test_glove_streams
    rclcpp::rclcpp
    wuji_glove_driver_impl
    wuji_msgs::wuji_msgs
  )
  set_tests_properties(test_glove_streams PROPERTIES RUN_SERIAL TRUE)

  ament_add_gtest(
    test_glove_node
    test/fake_sdk/fake_glove_sdk.cpp
    test/fake_sdk/test_glove_node.cpp
  )
  target_compile_features(test_glove_node PRIVATE cxx_std_17)
  target_include_directories(
    test_glove_node
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(
    test_glove_node
    rclcpp::rclcpp
    wuji_glove_driver_impl
  )
  set_tests_properties(test_glove_node PROPERTIES RUN_SERIAL TRUE)

  add_executable(
    wuji_glove_test_node
    test/fake_sdk/fake_glove_node_main.cpp
    test/fake_sdk/fake_glove_sdk.cpp
  )
  target_compile_features(wuji_glove_test_node PRIVATE cxx_std_17)
  target_include_directories(
    wuji_glove_test_node
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(
    wuji_glove_test_node
    PRIVATE
      rclcpp::rclcpp
      wuji_glove_driver_impl
  )
  install(TARGETS wuji_glove_test_node RUNTIME DESTINATION lib/${PROJECT_NAME})

  ament_add_pytest_test(
    test_glove_test_node_subprocess
    test/fake_sdk/test_glove_test_node.py
    ENV "WUJI_GLOVE_TEST_NODE=$<TARGET_FILE:wuji_glove_test_node>"
      "WUJI_GLOVE_NODE=$<TARGET_FILE:wuji_glove_node>"
    APPEND_ENV
      "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:wuji_sdk_vendor::wuji_sdk_c>:${CMAKE_INSTALL_PREFIX}/../wuji_msgs/lib"
    TIMEOUT 30
  )

  ament_add_gtest(
    test_glove_hardware_validation
    test/hardware/test_hardware_validation.cpp
    test/hardware/test_hardware_api_surface.cpp
    test/hardware/test_read_only_stream_probe.cpp
  )
  target_compile_features(test_glove_hardware_validation PRIVATE cxx_std_17)
  target_include_directories(
    test_glove_hardware_validation
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test/hardware
  )
  target_link_libraries(test_glove_hardware_validation wuji_glove_driver_impl)

  add_executable(
    wuji_glove_hardware_read_only
    test/hardware/test_read_only_smoke.cpp
  )
  target_compile_features(wuji_glove_hardware_read_only PRIVATE cxx_std_17)
  target_include_directories(
    wuji_glove_hardware_read_only
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test/hardware
  )
  target_link_libraries(
    wuji_glove_hardware_read_only
    PRIVATE
      sensor_msgs::sensor_msgs
      wuji_glove_driver_impl
      wuji_msgs::wuji_msgs
  )

  add_test(
    NAME hardware_glove_read_only
    COMMAND
      ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/run_hardware_tests.sh
      --glove-read-only
  )
  set_tests_properties(
    hardware_glove_read_only
    PROPERTIES
      ENVIRONMENT "WUJI_GLOVE_HARDWARE_RUNNER='$<TARGET_FILE:wuji_glove_hardware_read_only>'"
      LABELS "hardware;read-only;glove"
      SKIP_RETURN_CODE 77
      TIMEOUT 130
  )

  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
