cmake_minimum_required(VERSION 3.8)
project(Bender_Location)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#include_directories(include)
#link_directories(/usr/local/lib)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)



include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${ti_mmwave_rospkg_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${geometry_msgs_INLCUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${tf2_eigen_INCLUDE_DIRS}
  
)


include_directories(${cpd_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})

add_executable(bender_radar_processing src/bender_radar_processing.cpp)
target_link_libraries(bender_radar_processing ${PCL_LIBRARIES} ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${Boost_LIBRARIES})

add_executable(bender_radar_filter src/bender_radar_filter.cpp)
target_link_libraries(bender_radar_filter ${PCL_LIBRARIES} ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(bender_radar_icp src/bender_radar_icp.cpp)
target_link_libraries(bender_radar_icp ${PCL_LIBRARIES} ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${Boost_LIBRARIES} ${tf2_eigen_LIBRARIES} ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})

add_executable(bender_radar_visualziation src/bender_radar_visualziation.cpp)
target_link_libraries(bender_radar_visualziation ${PCL_LIBRARIES} ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${nav_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${tf2_ros_LIBRARIES})

add_executable(bender_csv src/bender_imu_csv.cpp)
target_link_libraries(bender_csv   ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${Boost_LIBRARIES})







target_link_libraries(bender_radar_processing
)

target_link_libraries(bender_radar_filter
)

target_link_libraries(bender_radar_icp
)

target_link_libraries(bender_radar_visualziation
)



install(TARGETS 
  bender_radar_processing 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  bender_radar_filter
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  bender_radar_icp
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  bender_radar_visualziation
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
bender_csv
DESTINATION lib/${PROJECT_NAME})


add_executable(radar_occupancy_node src/radar_occupancy_node.cpp)
ament_target_dependencies(radar_occupancy_node rclcpp sensor_msgs nav_msgs pcl_conversions)
install(TARGETS radar_occupancy_node DESTINATION lib/${PROJECT_NAME})





ament_package()
