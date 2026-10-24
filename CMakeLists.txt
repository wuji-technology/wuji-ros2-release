cmake_minimum_required(VERSION 3.20)
project(wuji_bringup LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS demo/hand2_status_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hand2_status_demo
)

install(
  PROGRAMS demo/hand2_wave_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hand2_wave_demo
)

install(
  PROGRAMS demo/glove_status_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME glove_status_demo
)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_pytest_test(
    test_hand2_launch_description
    test/test_hand2_launch_description.py
    TIMEOUT 60
  )
  add_launch_test(
    test/test_hand2_launch.py
    ARGS
      "hand_side:=left"
      "model_revision:=1"
      "namespace:=hand_under_test"
    TIMEOUT 45
  )

  ament_add_pytest_test(
    test_glove_launch_description
    test/test_glove_launch_description.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_glove_status_demo
    test/test_glove_status_demo.py
    TIMEOUT 60
  )

  function(
    add_glove_launch_test
    target
    case_name
    expected_stream_count
    selector_kind
    selector_value
  )
    add_launch_test(
      test/test_glove_launch.py
      TARGET ${target}
      ARGS ${ARGN}
      TIMEOUT 45
    )
    set_property(
      TEST ${target}
      APPEND
      PROPERTY ENVIRONMENT
        "WUJI_GLOVE_FAKE_SCENARIO=normal_shutdown"
        "WUJI_GLOVE_EXPECTED_STREAM_COUNT=${expected_stream_count}"
        "WUJI_GLOVE_LAUNCH_CASE=${case_name}"
        "WUJI_GLOVE_EXPECTED_SELECTOR_KIND=${selector_kind}"
        "WUJI_GLOVE_EXPECTED_SELECTOR_VALUE=${selector_value}"
    )
  endfunction()

  add_glove_launch_test(
    test_glove_launch_serial_number
    serial_number
    10
    serial_number
    WGLJ000000000001
    "serial_number:=WGLJ000000000001"
  )
  add_glove_launch_test(
    test_glove_launch_endpoint
    endpoint
    10
    endpoint
    udp://127.0.0.1:41000
    "endpoint:=udp://127.0.0.1:41000"
  )
  add_glove_launch_test(
    test_glove_launch_hand_side
    hand_side
    10
    hand_side
    WGLJ000000000001
    "hand_side:=left"
  )
  add_glove_launch_test(
    test_glove_launch_unique_discovery
    unique_discovery
    10
    unique_discovery
    WGLJ000000000001
  )
  add_glove_launch_test(
    test_glove_launch_namespace
    namespace
    10
    unique_discovery
    WGLJ000000000001
    "namespace:=glove_under_test"
  )
  add_glove_launch_test(
    test_glove_launch_publish_flags
    publish_flags
    1
    unique_discovery
    WGLJ000000000001
    "namespace:=flags_under_test"
    "publish_emf_poses:=false"
    "publish_tip_poses:=false"
    "publish_skeleton:=false"
    "publish_imu:=false"
    "publish_tactile:=false"
    "publish_tactile_residual:=false"
    "publish_tactile_binary:=false"
    "publish_tactile_zones:=false"
    "publish_tactile_points:=false"
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
