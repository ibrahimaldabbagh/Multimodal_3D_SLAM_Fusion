cmake_minimum_required(VERSION 3.8)
project(mv_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(mv_msg REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)


# Find FFmpeg libraries
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVDEVICE REQUIRED libavdevice)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)
pkg_check_modules(SWRESAMPLE REQUIRED libswresample)

# Include directories
include_directories(
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVDEVICE_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
  ${SWRESAMPLE_INCLUDE_DIRS}
)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${cv_bridge_INCLUDE_DIRS})

# Add executable
add_executable(mv_publisher src/mv_publisher.cpp)
ament_target_dependencies(mv_publisher rclcpp std_msgs mv_msg OpenCV cv_bridge)

add_executable(imu_odometry src/mv_imu_odometry.cpp)
ament_target_dependencies(imu_odometry rclcpp std_msgs nav_msgs geometry_msgs tf2_geometry_msgs mv_msg tf2 OpenCV cv_bridge)

add_executable(mv_viewer src/mv_viewer.cpp)
ament_target_dependencies(mv_viewer rclcpp std_msgs mv_msg OpenCV cv_bridge)


# Link libraries
target_link_libraries(mv_publisher
    avdevice
    avformat
    avcodec
    avfilter
    avutil
    swscale
    swresample
)

target_link_libraries(mv_publisher ${OpenCV_LIBRARIES} ${cv_bridge_LIBRARIES})

# Install executable
install(TARGETS mv_publisher
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS imu_odometry
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS mv_viewer
    DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )


ament_package()
