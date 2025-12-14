cmake_minimum_required(VERSION 3.8)
project(ros381_tactics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros381_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclpy REQUIRED)
find_package(pybind11 REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)

find_package(Python REQUIRED COMPONENTS Interpreter Development)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(global src/global.cpp src/pybinding.cpp)
ament_target_dependencies(global 
    rclcpp 
    rclcpp_action 
    example_interfaces 
    ros381_interfaces 
    ament_index_cpp
    dynamixel_sdk_custom_interfaces
)

target_link_libraries(global
    pybind11::embed
    Python::Python
)

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

ament_python_install_package(${PROJECT_NAME})
ament_package()