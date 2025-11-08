cmake_minimum_required(VERSION 3.8)
project(sensor_fusion)

# Default to C++14 (or 17) if not already set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)  # For Eigen

# Create the executable from your node source
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)

# Link against ROS2 + Eigen
ament_target_dependencies(sensor_fusion_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  message_filters
)

target_link_libraries(sensor_fusion_node
  Eigen3::Eigen
)

# Install
install(
  TARGETS sensor_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(dummy_test_node src/dummy_test_node.cpp)
ament_target_dependencies(dummy_test_node
    rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
    tf2_ros
    tf2_geometry_msgs
)
install(
  TARGETS dummy_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Optional: enable tests or linters if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # e.g. set(ament_cmake_cpplint_FOUND TRUE)
  # e.g. set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

