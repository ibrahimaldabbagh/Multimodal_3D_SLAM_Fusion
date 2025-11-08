cmake_minimum_required(VERSION 3.5)
project(pressure_sensor_node)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pressure_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

