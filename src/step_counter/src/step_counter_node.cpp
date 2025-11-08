#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "serial/serial.h"
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class StepCountPublisher : public rclcpp::Node
{
public:
    StepCountPublisher()
        : Node("step_count_publisher"),
          serial_port_("/dev/ttyACM0", 230400, serial::Timeout::simpleTimeout(1000)),
          step_length_(0.5),   // typical human step length in meters
          yaw_(1.57),
          pos_x_(0.0),
          pos_y_(0.0),
          step_offset_(-1)
    {
        RCLCPP_INFO(this->get_logger(), "StepCountPublisher node started.");

        step_pub_ = this->create_publisher<std_msgs::msg::Int32>("/step_count", 1);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/step_only/pose", 1);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 100,
            std::bind(&StepCountPublisher::imu_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&StepCountPublisher::timer_callback, this));
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        yaw_ = yaw; // raw IMU yaw in rad
    }

    void timer_callback()
    {
        if (serial_port_.available() == 0)
        {
            RCLCPP_INFO(this->get_logger(), "No serial data available.");
            return;
        }

        while (serial_port_.available() > 0)
        {
            std::string line = serial_port_.readline();

            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
            line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());

            std::vector<std::string> data;
            std::istringstream iss(line);
            std::string item;

            while (std::getline(iss, item, ','))
            {
                data.push_back(item);
            }

            if (data.size() == 5)
            {
                try
                {
                    int raw_step_count = std::stoi(data[4]);

                    // Record initial step count as offset
                    if (step_offset_ == -1) {
                        step_offset_ = raw_step_count;
                        RCLCPP_INFO(this->get_logger(), "Initial step offset set to: %d", step_offset_);
                    }

                    int relative_steps = raw_step_count - step_offset_;
                    if (relative_steps < 0) return;

                    std_msgs::msg::Int32 step_msg;
                    step_msg.data = relative_steps;
                    step_pub_->publish(step_msg);

                    RCLCPP_INFO(this->get_logger(), "Published step count: %d (raw=%d)", relative_steps, raw_step_count);

                    int steps_taken = relative_steps - prev_step_count_;
                    if (steps_taken <= 0) return;

                    double displacement = step_length_ * steps_taken;

                    // -------------------------------------------
                    // FIX: Use -yaw_ to match camera frame
                    // -------------------------------------------
                    double corrected_yaw = -yaw_;

                    pos_x_ += displacement * cos(corrected_yaw);
                    pos_y_ += displacement * sin(corrected_yaw);

                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.stamp = this->now();
                    pose.header.frame_id = "odom";
                    pose.pose.position.x = pos_x_;
                    pose.pose.position.y = pos_y_;
                    pose.pose.position.z = 0.0;

                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, corrected_yaw);
                    pose.pose.orientation = tf2::toMsg(q);

                    pose_pub_->publish(pose);

                    RCLCPP_INFO(this->get_logger(),
                                "Published pose: X=%.2f, Y=%.2f, Yaw=%.2f rad",
                                pos_x_, pos_y_, corrected_yaw);

                    prev_step_count_ = relative_steps;
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse step count: %s", e.what());
                }
            }
        }
    }

    serial::Serial serial_port_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr step_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double step_length_;
    double yaw_;

    double pos_x_;
    double pos_y_;

    int step_offset_;       // to normalize sensor startup step count to zero
    int prev_step_count_;   // track previous relative step count
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StepCountPublisher>());
    rclcpp::shutdown();
    return 0;
}

