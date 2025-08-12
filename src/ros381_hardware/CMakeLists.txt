cmake_minimum_required(VERSION 3.8)
project(ros381_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros381_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(uc src/uc.cpp)
ament_target_dependencies(uc rclcpp ros381_interfaces example_interfaces)

install(TARGETS
  uc
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
