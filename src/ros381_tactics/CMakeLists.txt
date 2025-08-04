cmake_minimum_required(VERSION 3.8)
project(ros381_tactics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros381_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclpy REQUIRED)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(global src/global.cpp)
ament_target_dependencies(global rclcpp rclcpp_action example_interfaces ros381_interfaces)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  talker
  global
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/listener.py
  DESTINATION lib/${PROJECT_NAME}/scripts)

ament_package()
