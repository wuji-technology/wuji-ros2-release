cmake_minimum_required(VERSION 3.20)
project(wuji_hand2_driver LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wuji_sdk_vendor REQUIRED)
find_package(wuji_msgs REQUIRED)

add_subdirectory(driver_common)

add_library(
  wuji_hand2_driver_impl STATIC
  src/wuji_hand2_driver/driver_config.cpp
  src/wuji_hand2_driver/hand2_node.cpp
  src/wuji_hand2_driver/control/control_plane.cpp
  src/wuji_hand2_driver/control/hand_action_services.cpp
  src/wuji_hand2_driver/control/hand_configuration_services.cpp
  src/wuji_hand2_driver/control/joint_command_controller.cpp
  src/wuji_hand2_driver/control/tactile_calibration_service.cpp
  src/wuji_hand2_driver/stream/fingertip_stream.cpp
  src/wuji_hand2_driver/stream/imu_stream.cpp
  src/wuji_hand2_driver/stream/joint_diagnostics_stream.cpp
  src/wuji_hand2_driver/stream/joint_state_stream.cpp
  src/wuji_hand2_driver/conversion/tactile_decoder.cpp
  src/wuji_hand2_driver/conversion/imu_conversion.cpp
  src/wuji_hand2_driver/conversion/joint_command_conversion.cpp
  src/wuji_hand2_driver/conversion/joint_diagnostics_conversion.cpp
  src/wuji_hand2_driver/conversion/joint_state_conversion.cpp
  src/wuji_hand2_driver/conversion/ros_header_conversion.cpp
  src/wuji_hand2_driver/sdk/hand2_sdk_api.cpp
  src/wuji_hand2_driver/sdk/hand2_session.cpp
)
target_include_directories(
  wuji_hand2_driver_impl
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_compile_features(wuji_hand2_driver_impl PUBLIC cxx_std_17)
target_link_libraries(
  wuji_hand2_driver_impl
  PUBLIC
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    std_msgs::std_msgs
    std_srvs::std_srvs
    wuji_driver_common_stream
    wuji_sdk_vendor::wuji_sdk_c
    wuji_msgs::wuji_msgs
)
target_link_libraries(
  wuji_hand2_driver_impl
  PRIVATE
    nlohmann_json::nlohmann_json
)
set_target_properties(
  wuji_hand2_driver_impl
  PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    POSITION_INDEPENDENT_CODE ON
    VISIBILITY_INLINES_HIDDEN YES
)

add_executable(wuji_hand2_node src/main.cpp)
target_compile_features(wuji_hand2_node PRIVATE cxx_std_17)
target_link_libraries(
  wuji_hand2_node
  PRIVATE
    rclcpp::rclcpp
    wuji_hand2_driver_impl
)
install(TARGETS wuji_hand2_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(
    test_stream_controller
    test/common/test_stream_controller.cpp
  )
  target_compile_features(test_stream_controller PRIVATE cxx_std_17)
  target_link_libraries(
    test_stream_controller
    wuji_driver_common_stream
  )

  ament_add_gtest(
    test_stream_registry
    test/common/test_stream_registry.cpp
  )
  target_compile_features(test_stream_registry PRIVATE cxx_std_17)
  target_link_libraries(
    test_stream_registry
    wuji_driver_common_stream
  )

  ament_add_gtest(
    test_hand2_session
    test/fake_sdk/fake_hand2_sdk.cpp
    test/unit/test_hand2_session.cpp
  )

  target_compile_features(test_hand2_session PRIVATE cxx_std_17)
  target_include_directories(
    test_hand2_session
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(
    test_hand2_session
    wuji_hand2_driver_impl
  )
  set_tests_properties(
    test_hand2_session
    PROPERTIES RUN_SERIAL TRUE
  )

  ament_add_gtest(
    test_joint_conversion
    test/unit/test_joint_conversion.cpp
  )
  target_compile_features(test_joint_conversion PRIVATE cxx_std_17)
  target_include_directories(
    test_joint_conversion
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(
    test_joint_conversion
    wuji_hand2_driver_impl
  )

  ament_add_gtest(
    test_tactile_decoder
    test/unit/test_tactile_decoder.cpp
  )
  target_compile_features(test_tactile_decoder PRIVATE cxx_std_17)
  target_include_directories(
    test_tactile_decoder
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(
    test_tactile_decoder
    wuji_hand2_driver_impl
  )

  ament_add_gtest(
    test_command_and_actions
    test/fake_sdk/fake_hand2_sdk.cpp
    test/unit/test_command_and_actions.cpp
  )
  target_compile_features(test_command_and_actions PRIVATE cxx_std_17)
  target_include_directories(
    test_command_and_actions
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(
    test_command_and_actions
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    std_srvs::std_srvs
    wuji_hand2_driver_impl
    wuji_msgs::wuji_msgs
  )
  set_tests_properties(
    test_command_and_actions
    PROPERTIES RUN_SERIAL TRUE
  )

  ament_add_gtest(
    test_configuration_and_calibration
    test/fake_sdk/fake_hand2_sdk.cpp
    test/unit/test_configuration_and_calibration.cpp
  )
  target_compile_features(test_configuration_and_calibration PRIVATE cxx_std_17)
  target_include_directories(
    test_configuration_and_calibration
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(
    test_configuration_and_calibration
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    std_srvs::std_srvs
    wuji_hand2_driver_impl
    wuji_msgs::wuji_msgs
  )
  set_tests_properties(
    test_configuration_and_calibration
    PROPERTIES RUN_SERIAL TRUE
  )

  ament_add_gtest(
    test_fingertip_stream
    test/fake_sdk/fake_hand2_sdk.cpp
    test/unit/test_fingertip_stream.cpp
  )
  target_compile_features(test_fingertip_stream PRIVATE cxx_std_17)
  target_include_directories(
    test_fingertip_stream
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(
    test_fingertip_stream
    rclcpp::rclcpp
    wuji_hand2_driver_impl
  )
  set_tests_properties(
    test_fingertip_stream
    PROPERTIES RUN_SERIAL TRUE
  )

  ament_add_gtest(
    test_hand2_streams
    test/fake_sdk/fake_hand2_sdk.cpp
    test/unit/test_hand2_streams.cpp
  )
  target_compile_features(test_hand2_streams PRIVATE cxx_std_17)
  target_include_directories(
    test_hand2_streams
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(
    test_hand2_streams
    rclcpp::rclcpp
    wuji_hand2_driver_impl
  )
  set_tests_properties(
    test_hand2_streams
    PROPERTIES RUN_SERIAL TRUE
  )

  ament_add_gtest(
    test_hand2_startup
    test/fake_sdk/fake_hand2_sdk.cpp
    test/unit/test_hand2_startup.cpp
  )
  target_compile_features(test_hand2_startup PRIVATE cxx_std_17)
  target_include_directories(
    test_hand2_startup
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(
    test_hand2_startup
    rclcpp::rclcpp
    wuji_hand2_driver_impl
  )
  set_tests_properties(test_hand2_startup PROPERTIES RUN_SERIAL TRUE)

  ament_add_gtest(
    test_hand2_node
    test/fake_sdk/fake_hand2_sdk.cpp
    test/fake_sdk/test_hand2_node.cpp
  )
  target_compile_features(test_hand2_node PRIVATE cxx_std_17)
  target_include_directories(
    test_hand2_node
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(
    test_hand2_node
    rclcpp::rclcpp
    wuji_hand2_driver_impl
  )
  set_tests_properties(test_hand2_node PROPERTIES RUN_SERIAL TRUE)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_uncrustify_FOUND TRUE)

  add_executable(
    wuji_hand2_test_node
    test/fake_sdk/fake_hand2_node_main.cpp
    test/fake_sdk/fake_hand2_sdk.cpp
  )
  target_compile_features(wuji_hand2_test_node PRIVATE cxx_std_17)
  target_include_directories(
    wuji_hand2_test_node
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(
    wuji_hand2_test_node
    PRIVATE
      rclcpp::rclcpp
      wuji_hand2_driver_impl
  )
  install(TARGETS wuji_hand2_test_node DESTINATION lib/${PROJECT_NAME})

  ament_add_pytest_test(
    test_hand2_test_node_subprocess
    test/fake_sdk/test_hand2_test_node.py
    ENV "WUJI_HAND2_TEST_NODE=$<TARGET_FILE:wuji_hand2_test_node>"
      "WUJI_HAND2_NODE=$<TARGET_FILE:wuji_hand2_node>"
    APPEND_ENV
      "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:wuji_sdk_vendor::wuji_sdk_c>:${CMAKE_INSTALL_PREFIX}/../wuji_msgs/lib"
    TIMEOUT 10
  )

  ament_add_gtest(
    test_hand2_hardware_validation
    test/hardware/test_hardware_validation.cpp
    test/hardware/test_read_only_hardware_api.cpp
  )
  target_compile_features(test_hand2_hardware_validation PRIVATE cxx_std_17)
  target_include_directories(
    test_hand2_hardware_validation
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/test/hardware
  )
  target_link_libraries(test_hand2_hardware_validation wuji_hand2_driver_impl)

  add_executable(
    wuji_hand2_hardware_read_only
    test/hardware/test_read_only_smoke.cpp
  )
  target_compile_features(wuji_hand2_hardware_read_only PRIVATE cxx_std_17)
  target_include_directories(
    wuji_hand2_hardware_read_only
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test/hardware
  )
  target_link_libraries(
    wuji_hand2_hardware_read_only
    PRIVATE
      sensor_msgs::sensor_msgs
      wuji_hand2_driver_impl
      wuji_msgs::wuji_msgs
  )

  add_executable(
    wuji_hand2_hardware_mutating
    test/hardware/test_mutating_controls.cpp
  )
  target_compile_features(wuji_hand2_hardware_mutating PRIVATE cxx_std_17)
  target_include_directories(
    wuji_hand2_hardware_mutating
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test/hardware
  )
  target_link_libraries(
    wuji_hand2_hardware_mutating
    PRIVATE
      wuji_hand2_driver_impl
  )

  add_test(
    NAME hardware_hand2_read_only
    COMMAND
      ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/run_hardware_tests.sh
      --read-only
  )
  set_tests_properties(
    hardware_hand2_read_only
    PROPERTIES
      ENVIRONMENT "WUJI_HARDWARE_RUNNER='$<TARGET_FILE:wuji_hand2_hardware_read_only>'"
      LABELS "hardware;read-only"
      SKIP_RETURN_CODE 77
      TIMEOUT 90
  )

  add_test(
    NAME hardware_hand2_mutating
    COMMAND
      ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/run_hardware_tests.sh
      --mutating
  )
  set_tests_properties(
    hardware_hand2_mutating
    PROPERTIES
      ENVIRONMENT "WUJI_HARDWARE_RUNNER='$<TARGET_FILE:wuji_hand2_hardware_mutating>'"
      LABELS "hardware;mutating"
      SKIP_RETURN_CODE 77
      TIMEOUT 180
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
