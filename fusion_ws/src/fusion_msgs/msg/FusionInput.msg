# fusion_msgs/msg/FusionInput.msg

std_msgs/Header header

sensor_msgs/Imu imu
geometry_msgs/PoseWithCovarianceStamped radar
geometry_msgs/PoseWithCovarianceStamped camera
geometry_msgs/PoseWithCovarianceStamped altitude
std_msgs/Int32 step

bool has_radar
bool has_camera
bool has_altitude
bool has_step

