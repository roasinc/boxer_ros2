cmake_minimum_required(VERSION 3.8)
project(boxer_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(
  DEPENDENCIES
  controller_interface
  controller_manager
  nav_msgs
  boxer_msgs
  std_msgs
  robot_localization
  joy_linux
  teleop_twist_joy
  twist_mux
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
