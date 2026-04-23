cmake_minimum_required(VERSION 3.8)
project(ros381_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros381_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(miniMBP src/miniMBP.cpp src/signal.cpp)
ament_target_dependencies(miniMBP rclcpp ros381_interfaces nav_msgs tf2 tf2_geometry_msgs example_interfaces)

add_executable(control_loop src/control_loop.cpp src/signal.cpp)
ament_target_dependencies(control_loop rclcpp ros381_interfaces nav_msgs tf2 tf2_geometry_msgs example_interfaces)

add_executable(odometry src/odometry.cpp src/signal.cpp)
ament_target_dependencies(odometry rclcpp ros381_interfaces nav_msgs tf2)

add_executable(gridmap src/gridmap.cpp)
ament_target_dependencies(gridmap rclcpp ros381_interfaces nav_msgs sensor_msgs)

add_executable(obstacle src/obstacle.cpp src/signal.cpp)
ament_target_dependencies(obstacle rclcpp ros381_interfaces nav_msgs sensor_msgs example_interfaces)

install(TARGETS
  miniMBP
  control_loop
  odometry
  gridmap
  obstacle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
