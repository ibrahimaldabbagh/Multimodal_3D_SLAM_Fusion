cmake_minimum_required(VERSION 3.10)
project(mvslam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(mvslam_core_node src/mvslam_core_node.cpp)
ament_target_dependencies(mvslam_core_node
  rclcpp sensor_msgs geometry_msgs nav_msgs cv_bridge message_filters tf2 tf2_geometry_msgs
)
target_link_libraries(mvslam_core_node
  ${OpenCV_LIBS}
)

install(TARGETS
  mvslam_core_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
