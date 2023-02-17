
cmake_minimum_required(VERSION 3.8)
project(ros2_dobot)
set(CMAKE_COLOR_MAKEFILE off)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) 
endif()

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE BUILD_ARCH)

if (BUILD_ARCH  STREQUAL "x86_64")
  add_compile_options(-fPIC)
  add_definitions(-DQT_NO_VERSION_TAGGING)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(dobot_interfaces REQUIRED)

include_directories(include ${Qt5Core_INCLUDE_DIRS})
link_directories(lib/${BUILD_ARCH} ${Boost_LIBRARY_DIRS})


ament_export_dependencies(rosidl_default_runtime)

# Dobot Server
add_executable(DobotServer src/DobotServer.cpp)
if (BUILD_ARCH  STREQUAL "x86_64")
  set_target_properties(DobotServer PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
endif()
ament_target_dependencies(DobotServer rclcpp std_msgs dobot_interfaces)
# get_typesupport not needed since srv moved to its own package
# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(DobotServer DobotDll "${cpp_typesupport_target}")

# Dobot PTP Client
add_executable(DobotClient_PTP src/DobotClient_PTP.cpp)
ament_target_dependencies(DobotClient_PTP rclcpp std_msgs Boost dobot_interfaces)
# get_typesupport not needed since srv moved to its own package
# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(DobotClient_PTP "${cpp_typesupport_target}")

# Dobot JOG Client
add_executable(DobotClient_JOG src/DobotClient_JOG.cpp)
ament_target_dependencies(DobotClient_JOG rclcpp std_msgs geometry_msgs dobot_interfaces)
# get_typesupport not needed since srv moved to its own package
# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(DobotClient_JOG  ${Boost_LIBRARIES} "${cpp_typesupport_target}")

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
  DobotClient_PTP
  DobotClient_JOG
  DESTINATION lib/${PROJECT_NAME})

ament_package()
