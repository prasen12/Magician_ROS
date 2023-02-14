
cmake_minimum_required(VERSION 3.8)
project(ros2_dobot)
set(CMAKE_COLOR_MAKEFILE off)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE BUILD_ARCH)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)

include_directories(include ${Qt5Core_INCLUDE_DIRS})
link_directories(lib/${BUILD_ARCH})

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

ament_export_dependencies(rosidl_default_runtime)

add_executable(DobotServer src/DobotServer.cpp)

ament_target_dependencies(DobotServer rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(DobotServer DobotDll "${cpp_typesupport_target}")

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

install(TARGETS 
  DobotServer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
