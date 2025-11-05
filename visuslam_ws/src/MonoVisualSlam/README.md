# 🚀 Installation

### Clone Repository
```
```
### Install Requirements

*ROS 2 Humble, follow installation instructions:*
```
https://docs.ros.org/en/humble/Installation.html
```
*This repository already includes the OpenCV 4.2.0 and Serial-ROS package.*

*Build all packages:*
```
cd my_workspace
colcon build
```


# 💻 Run the Application

- For the publisher, use the launch file to automatically open cameras and IMU.
```
ros2 launch mv_cpp mv_imu.py
```
- For running the odometry, start the publisher first, then use the following command.
```
ros2 run mv_cpp imu_odometry
```
- For running one camera individually, provide the URL to the device, like this:
```
ros2 run mv_cpp mv_publisher /dev/video6
```
- For running just the IMU node, use:
```
ros2 run imu_cpp imu_node
```
- To view the motion vectors on the image, start the publisher first, then use:
```
ros2 run mv_cpp mv_viewer
```
- For recording a ROS-Bag, start the publisher und then use this command:
```
ros2 bag record -s mcap /mvs/dev/video6/image /mvs/dev/video10/image /mvs/dev/video6/motion_vector /mvs/dev/video10/motion_vector /mvs/imu/data_BNO85
```
- The images and path can be visualized with RVIZ2, there is also a config-file provided.
```
rviz2
```

# 🔧 Key Features

✅  Visual Simultaneous Localization and Mapping (vSLAM) system

✅ Designed to protect Firefighters in narrow, dark, and low-contrast environments

✅ Uses motion vectors, extracted from the H.264 camera encoder

✅ Low system requirements, works on portable hardware

✅ Internal error correction, works for short time on a single camera

✅ Real-time capable

# 📷 Examples

*Hardware setup, including both cameras and an IMU*

![](src/mv_python/mv_python/images/Readme/hardware.png)

*Hardware mounted to the oxygen tank*

![](src/mv_python/mv_python/images/Readme/hardware_mounted.png)

*The extracted motion vectors visualized, using the publisher & viewer nodes*

![](src/mv_python/mv_python/images/Readme/viewer_example.png)

*The test area in building 20, on the Dräger campus. This is the ground truth measured:*

![](src/mv_python/mv_python/images/Readme/map.png)

*This is the output of RVIZ after a typical testrun:*

![](src/mv_python/mv_python/images/Readme/rviz_output.png)

# 📝 Credits

This Project was created by Christoph Osterloh and Steffen Häusler @Dräger.

In cooperation with University of Lübeck, this system is part of a master thesis.