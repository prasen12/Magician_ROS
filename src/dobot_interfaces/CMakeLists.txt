cmake_minimum_required(VERSION 3.8)
project(dobot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"srv/ClearAllAlarmsState.srv"
"srv/GetAlarmsState.srv"
"srv/GetARCParams.srv"
"srv/GetColorSensor.srv"
"srv/GetCPCmd.srv"
"srv/GetCPParams.srv"
"srv/GetDeviceName.srv"
"srv/GetDeviceSN.srv"
"srv/GetDeviceVersion.srv"
"srv/GetEndEffectorGripper.srv"
"srv/GetEndEffectorLaser.srv"
"srv/GetEndEffectorParams.srv"
"srv/GetEndEffectorSuctionCup.srv"
"srv/GetHOMEParams.srv"
"srv/GetInfraredSensor.srv"
"srv/GetIOADC.srv"
"srv/GetIODI.srv"
"srv/GetIODO.srv"
"srv/GetIOMultiplexing.srv"
"srv/GetIOPWM.srv"
"srv/GetJOGCommonParams.srv"
"srv/GetJOGCoordinateParams.srv"
"srv/GetJOGJointParams.srv"
"srv/GetPose.srv"
"srv/GetPTPCommonParams.srv"
"srv/GetPTPCoordinateParams.srv"
"srv/GetPTPJointParams.srv"
"srv/GetPTPJumpParams.srv"
"srv/SetARCCmd.srv"
"srv/SetARCParams.srv"
"srv/SetCmdTimeout.srv"
"srv/SetColorSensor.srv"
"srv/SetCPCmd.srv"
"srv/SetCPParams.srv"
"srv/SetDeviceName.srv"
"srv/SetEMotor.srv"
"srv/SetEndEffectorGripper.srv"
"srv/SetEndEffectorLaser.srv"
"srv/SetEndEffectorParams.srv"
"srv/SetEndEffectorSuctionCup.srv"
"srv/SetHOMECmd.srv"
"srv/SetHOMEParams.srv"
"srv/SetInfraredSensor.srv"
"srv/SetIODO.srv"
"srv/SetIOMultiplexing.srv"
"srv/SetIOPWM.srv"
"srv/SetJOGCmd.srv"
"srv/SetJOGCommonParams.srv"
"srv/SetJOGCoordinateParams.srv"
"srv/SetJOGJointParams.srv"
"srv/SetPTPCmd.srv"
"srv/SetPTPCommonParams.srv"
"srv/SetPTPCoordinateParams.srv"
"srv/SetPTPJointParams.srv"
"srv/SetPTPJumpParams.srv"
"srv/SetQueuedCmdClear.srv"
"srv/SetQueuedCmdForceStopExec.srv"
"srv/SetQueuedCmdStartExec.srv"
"srv/SetQueuedCmdStopExec.srv"
"srv/SetTRIGCmd.srv"
"srv/SetWAITCmd.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
