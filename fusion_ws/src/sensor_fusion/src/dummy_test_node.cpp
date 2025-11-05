#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <random>

class DummyNode : public rclcpp::Node
{
public:
    DummyNode() : Node("dummy_square_path_node"), generator(std::random_device{}()), noise_distribution(0.0, 0.03)
    {
        RCLCPP_INFO(this->get_logger(), "Dummy Square Path Node initialized");

        // Declare and initialize publishing rates and step length
        pub_rate_imu_ = this->declare_parameter<double>("pub_rate_imu", 10.0);
        pub_rate_radar_ = this->declare_parameter<double>("pub_rate_radar", 10.0);
        pub_rate_camera_ = this->declare_parameter<double>("pub_rate_camera", 10.0);
        pub_rate_altitude_ = this->declare_parameter<double>("pub_rate_altitude", 10.0);
        pub_rate_step_ = 2.0;  // Step count publishes every 500 ms (realistic walking pace)
        step_length_ = 0.6;    // Average human step length in meters

        // Initialize publishers for different sensors
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_BNO8x", 10);
        radar_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose_estimation", 10);
        camera_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/mvs/pose", 10);
        altitude_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pressure/altitude", 10);
        step_pub_ = this->create_publisher<std_msgs::msg::Int32>("/step_count", 10);

        // Timers trigger publishing functions at specified rates
        imu_timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / pub_rate_imu_)),
            std::bind(&DummyNode::publishIMU, this));

        radar_timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / pub_rate_radar_)),
            std::bind(&DummyNode::publishRadar, this));

        camera_timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / pub_rate_camera_)),
            std::bind(&DummyNode::publishCamera, this));

        altitude_timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / pub_rate_altitude_)),
            std::bind(&DummyNode::publishAltitude, this));

        step_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), // Publish every 500 ms
            std::bind(&DummyNode::publishStepCount, this));

        // Initialize simulation state variables
        sim_time_start_ = this->now();
        last_update_time_ = this->now();
        last_step_time_ = this->now();

        total_steps_ = 0;
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        distance_in_current_leg_ = 0.0;
    }

private:
    // Define ROS2 publishers for different topics
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr radar_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr altitude_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr step_pub_;

    // Timers for publishing data at regular intervals
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr radar_timer_;
    rclcpp::TimerBase::SharedPtr camera_timer_;
    rclcpp::TimerBase::SharedPtr altitude_timer_;
    rclcpp::TimerBase::SharedPtr step_timer_;

    // State variables for tracking positions and orientation
    rclcpp::Time last_update_time_;
    rclcpp::Time last_step_time_;
    rclcpp::Time sim_time_start_;

    double current_x_;
    double current_y_;
    double current_yaw_;
    double distance_in_current_leg_;  // Tracks distance traveled in current leg of the square

    double pub_rate_imu_, pub_rate_radar_, pub_rate_camera_, pub_rate_altitude_, pub_rate_step_;
    double base_linear_speed_ = 0.3;
    double step_length_;
    int total_steps_;

    // Random noise generator for simulating realistic measurement errors
    std::default_random_engine generator;
    std::normal_distribution<double> noise_distribution;

    void publishIMU() {
        auto now = this->now();
        sensor_msgs::msg::Imu msg_imu;
        msg_imu.header.stamp = now;
        msg_imu.header.frame_id = "base_link";
        imu_pub_->publish(msg_imu);
    }

    void publishRadar() {
        publishPose(radar_pub_, "Radar");
    }

    void publishCamera() {
        publishPose(camera_pub_, "Camera");
    }

    void publishAltitude() {
        auto now = this->now();
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = now;
        msg.header.frame_id = "odom";
        msg.pose.pose.position.z = 0.0;
        altitude_pub_->publish(msg);
    }

    void publishStepCount() {
        auto now = this->now();
        total_steps_ += 1;

        // Simulate actual movement for each step
        double delta = step_length_;  // Move 0.6m per step
        current_x_ += delta * cos(current_yaw_);
        current_y_ += delta * sin(current_yaw_);
        distance_in_current_leg_ += delta;

        // If 5 meters have been traveled, turn 90 degrees (pi/2 radians)
        if (distance_in_current_leg_ >= 5.0) {
            distance_in_current_leg_ = 0.0;
            current_yaw_ += M_PI / 2;
            if (current_yaw_ >= 2 * M_PI) {
                current_yaw_ -= 2 * M_PI;  // Keep yaw within 0-2pi
            }
        }

        std_msgs::msg::Int32 msg;
        msg.data = total_steps_;
        step_pub_->publish(msg);
    }

    void publishPose(rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub, const std::string& sensor_name)
    {
        auto now = this->now();
        double dt = (now - last_update_time_).seconds();
        if (dt <= 0.0) return;
        last_update_time_ = now;

        // Add random noise to simulate measurement errors
        double noise_x = noise_distribution(generator);
        double noise_y = noise_distribution(generator);

        // Prepare pose message with simulated errors and covariance
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = now;
        msg.header.frame_id = "odom";

        msg.pose.pose.position.x = current_x_ + noise_x;
        msg.pose.pose.position.y = current_y_ + noise_y;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_yaw_);
        msg.pose.pose.orientation = tf2::toMsg(q);

        // Define covariance for pose estimation
        for (int i = 0; i < 36; ++i) {
            msg.pose.covariance[i] = 0.0;
        }
        msg.pose.covariance[0] = 0.05; // x covariance
        msg.pose.covariance[7] = 0.1;  // y covariance (camera should have more noise later)
        msg.pose.covariance[14] = 0.01; // z covariance
        msg.pose.covariance[35] = 0.02; // yaw covariance

        pub->publish(msg);
        RCLCPP_INFO(this->get_logger(), "[%s] Published Pose X=%.2f, Y=%.2f, Yaw=%.2f", sensor_name.c_str(), msg.pose.pose.position.x, msg.pose.pose.position.y, current_yaw_);
    }
};

// Main function initializes the ROS2 node and starts spinning
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DummyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
