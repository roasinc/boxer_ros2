cmake_minimum_required(VERSION 3.8)
project(boxer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(
  DEPENDENCIES
  rosidl_default_generators
  builtin_interfaces
  std_msgs
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

set(
  MSG_FILES
  "msg/Drive.msg"
  "msg/Lift.msg"
  "msg/LiftPosition.msg"
  "msg/MCUResetReason.msg"
  "msg/MotorFeedback.msg"
  "msg/Pid.msg"
  "msg/SafetyMode.msg"
  "msg/Status.msg"
  "msg/TractionStatus.msg"
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
