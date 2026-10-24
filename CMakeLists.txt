cmake_minimum_required(VERSION 3.20)
project(wuji_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FingertipTactile.msg"
  "msg/TrackedPose.msg"
  "msg/TrackedPoseArray.msg"
  "msg/FingerJointAngles.msg"
  "msg/HandJointAngles.msg"
  "msg/TactileGrid.msg"
  "msg/TactileZone.msg"
  "msg/TactileZoneArray.msg"
  "msg/JointDiagnostics.msg"
  "msg/JointDiagnosticsArray.msg"
  "srv/SetEnabled.srv"
  "srv/ClearFaults.srv"
  "srv/SetEffortLimit.srv"
  "srv/SetEffortLimits.srv"
  "srv/SetMitParams.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_interfaces test/test_interfaces.py)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
