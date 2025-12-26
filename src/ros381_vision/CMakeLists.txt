cmake_minimum_required(VERSION 3.8)
project(ros381_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

add_executable(aruco_detection src/aruco_detection.cpp)
target_include_directories(aruco_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(aruco_detection PUBLIC c_std_99 cxx_std_17)
target_link_libraries(aruco_detection Eigen3::Eigen)
ament_target_dependencies(
  aruco_detection
  "rclcpp"
  "image_transport"
  "cv_bridge"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
)

install(TARGETS
  aruco_detection
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
