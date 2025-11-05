cmake_minimum_required(VERSION 3.8)
project(step_counter)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)

# Define the executable
add_executable(step_counter_node src/step_counter_node.cpp)

# Link dependencies
ament_target_dependencies(step_counter_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  serial
)

# Install target
install(TARGETS step_counter_node
  DESTINATION lib/${PROJECT_NAME})

# Package definition
ament_package()
