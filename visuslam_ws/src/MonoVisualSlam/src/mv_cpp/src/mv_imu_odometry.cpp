#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include "mv_msg/msg/motion_vector_combined.hpp"
#include "mv_msg/msg/motion_vector.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

class Odometry : public rclcpp::Node
{
public:
    Odometry() : Node("mv_imu_odometry")
    {
        mv_1_subscription_ = this->create_subscription<mv_msg::msg::MotionVectorCombined>(
            "mvs/dev/video2/motion_vector", 1, std::bind(&Odometry::mv_1_callback, this, std::placeholders::_1));

        mv_2_subscription_ = this->create_subscription<mv_msg::msg::MotionVectorCombined>(
            "mvs/dev/video6/motion_vector", 1, std::bind(&Odometry::mv_2_callback, this, std::placeholders::_1));

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 1, std::bind(&Odometry::imu_callback, this, std::placeholders::_1));

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mvs/pose", 1);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("mvs/path", 1);

        std::cout << "IMU MV Odometry is running." << std::endl;
        std::cout << std::fixed << std::setprecision(1);
    }

    struct Point {
        double x, y;
    };

private:
    bool prints = true;
    double min_mv = 0.5;
    double pitch_offset = 0.0;
    double yaw_offset = -0.09;
    int translation_scale = 1200;

    double x1, y1, x2, y2 = 0;
    int n1, n2, n_max = 1;
    std::vector<double> position = {0,0,0};
    double xy_distance = 0;
    static const int translation_buffer_size = 20;
    static const int imu_buffer_size = 90;
    int translation_buffer[translation_buffer_size] = {0};
    std::vector<std::vector<double>> imu_RPY_buffer{imu_buffer_size, std::vector<double>(3, 0)};

    bool cam1_valid_ = true;
    bool cam2_valid_ = true;
    double cam1_confidence_ = 1.0;
    double cam2_confidence_ = 1.0;

    rclcpp::Time last_mv_stamp_;
    rclcpp::Subscription<mv_msg::msg::MotionVectorCombined>::SharedPtr mv_1_subscription_;
    rclcpp::Subscription<mv_msg::msg::MotionVectorCombined>::SharedPtr mv_2_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
        tf2::Quaternion q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        tf2::Matrix3x3 m(q);
        std::vector<double> imu_RPY(3);
        m.getRPY(imu_RPY[2], imu_RPY[1], imu_RPY[0]);
        imu_RPY[1] = -imu_RPY[1];
        imu_RPY[2] = -imu_RPY[2] + M_PI;
        if (imu_RPY[0] > M_PI / 2 || imu_RPY[0] < -M_PI / 2) {
            imu_RPY[1] = (imu_RPY[1] > 0) ? (M_PI - imu_RPY[1]) : (-M_PI - imu_RPY[1]);
        }
        std::rotate(imu_RPY_buffer.rbegin(), imu_RPY_buffer.rbegin() + 1, imu_RPY_buffer.rend());
        imu_RPY_buffer[0] = imu_RPY;
    }

    void mv_1_callback(const mv_msg::msg::MotionVectorCombined::SharedPtr msg){
        last_mv_stamp_ = msg->header.stamp;
        std::vector<double> translation = {0,0};
        double pitch = imu_RPY_buffer[imu_buffer_size-1][1] + pitch_offset;
        double yaw = imu_RPY_buffer[imu_buffer_size-1][2] + yaw_offset;
        yaw = fmod(yaw, 2*M_PI);

        cam1_confidence_ = median(msg->mv, &x1, &y1, &n1, 1);

        translation.at(0) = (-cos(pitch)*x1 + sin(pitch)*y1)/translation_scale;
        translation.at(1) = ( sin(pitch)*x1 + cos(pitch)*y1)/translation_scale;

        position.at(0) += cos(yaw) * translation.at(0);
        position.at(1) += sin(yaw) * translation.at(0);
        position.at(2) += translation.at(1);
        xy_distance += translation.at(0);

        publish();
    }

    void mv_2_callback(const mv_msg::msg::MotionVectorCombined::SharedPtr msg){
        last_mv_stamp_ = msg->header.stamp;
        std::vector<double> translation = {0,0};
        double pitch = imu_RPY_buffer[imu_buffer_size-1][1] + pitch_offset;
        double yaw = imu_RPY_buffer[imu_buffer_size-1][2] + yaw_offset;
        yaw = fmod(yaw, 2*M_PI);

        cam2_confidence_ = median(msg->mv, &x2, &y2, &n2, 2);

        translation.at(0) = ( cos(pitch)*x2 + sin(pitch)*y2)/translation_scale;
        translation.at(1) = (-sin(pitch)*x2 + cos(pitch)*y2)/translation_scale;

        position.at(0) += cos(yaw) * translation.at(0);
        position.at(1) += sin(yaw) * translation.at(0);
        position.at(2) += translation.at(1);
        xy_distance += translation.at(0);

        publish();
    }

    void publish(){
        if (prints){
            std::cout << "-------" << std::endl;
            std::cout << "x1: "<< int(x1) << std::endl;
            std::cout << "x2: "<< int(x2) << std::endl;
            std::cout << "y1: "<< int(y1) << std::endl;
            std::cout << "y2: "<< int(y2) << std::endl;
            std::cout << "n1: "<< int(n1) << std::endl;
            std::cout << "n2: "<< int(n2) << std::endl;
            std::cout << "n_max: "<< n_max << std::endl;
            std::cout << "avg_tx "<< avg_translation() << std::endl;
            std::cout << "alpha: " << int(imu_RPY_buffer[imu_buffer_size-1][2] * 180 / M_PI) << "°" << std::endl;
            std::cout << "x: " << position.at(0) << "m" << std::endl;
            std::cout << "y: " << position.at(1) << "m" << std::endl;
            std::cout << "z: " << position.at(2) << "m" << std::endl;
            std::cout << "xy_dist: " << xy_distance << "m" << std::endl;
        }

        geometry_msgs::msg::PoseWithCovarianceStamped message;
        message.header.stamp = last_mv_stamp_;
        message.header.frame_id = "odom";

        message.pose.pose.position.x = position.at(0);
        message.pose.pose.position.y = position.at(1);
        message.pose.pose.position.z = position.at(2);

        tf2::Quaternion q;
        q.setRPY(imu_RPY_buffer[imu_buffer_size-1][0], 
                imu_RPY_buffer[imu_buffer_size-1][1] + pitch_offset,
                imu_RPY_buffer[imu_buffer_size-1][2] + yaw_offset);

        message.pose.pose.orientation = tf2::toMsg(q);

        for (int i = 0; i < 36; ++i) {
            message.pose.covariance[i] = 0.0;
        }

        // Dynamically scale covariance based on confidence
        double avg_confidence = (cam1_confidence_ + cam2_confidence_) / 2.0;
        avg_confidence = std::clamp(avg_confidence, 0.01, 1.0);
        double scale = 1.0 / avg_confidence;

        message.pose.covariance[0]  = 2.0 * scale;
        message.pose.covariance[7]  = 2.0 * scale;
        message.pose.covariance[14] = 3.0 * scale;
        message.pose.covariance[21] = 0.05;
        message.pose.covariance[28] = 0.05;
        message.pose.covariance[35] = 0.05;

        geometry_msgs::msg::PoseStamped ps_for_path;
        ps_for_path.header = message.header;
        ps_for_path.pose = message.pose.pose;
        path_.push_back(ps_for_path);

        nav_msgs::msg::Path path_message;
        path_message.header.stamp = last_mv_stamp_;
        path_message.header.frame_id = "odom";
        path_message.poses = path_;

        pose_publisher_->publish(message);
        path_publisher_->publish(path_message);
    }

    double median(const std::vector<mv_msg::msg::MotionVector>& mv, double* x, double* y, int* n_mv, int cam_id){
        std::vector<double> x_values, y_values;
        int n = mv.size();
        *n_mv = n;
        if (n > n_max){ n_max = n; }
        for (int i = 0; i < n; i++) {
            x_values.push_back(mv[i].motion_x);
            y_values.push_back(mv[i].motion_y);
        }
        std::sort(x_values.begin(), x_values.end());
        std::sort(y_values.begin(), y_values.end());
        *y = -y_values[y_values.size() / 2];

        double confidence = (double)n / n_max;
        bool valid = true;

        if (abs(x_values[x_values.size() / 2]) < 10 && confidence < min_mv){
            valid = false;
            if (cam_id == 1){
                if (prints){std::cout << "cam 1 false reading: " << confidence << std::endl;}
                *x = -avg_translation();
            }
            if (cam_id == 2){
                if (prints){std::cout << "cam 2 false reading: " << confidence << std::endl;}
                *x = avg_translation();
            }
        } else {
            std::rotate(translation_buffer, translation_buffer + translation_buffer_size - 1, translation_buffer + translation_buffer_size);
            if (cam_id == 1){
                translation_buffer[0] = -x_values[x_values.size() / 2];
                if (prints){std::cout << "cam 1 set "<< translation_buffer[0] << std::endl;}
            }
            if (cam_id == 2){
                translation_buffer[0] = x_values[x_values.size() / 2];
                if (prints){std::cout << "cam 2 set "<< translation_buffer[0] << std::endl;}
            }
            *x = x_values[x_values.size() / 2];
        }

        if (cam_id == 1) cam1_valid_ = valid;
        if (cam_id == 2) cam2_valid_ = valid;

        return confidence;
    }

    int avg_translation(){
        int avg_x = 0;
        for (int i = 0; i < translation_buffer_size; i++) {
            avg_x += translation_buffer[i];
        }
        return int(avg_x / translation_buffer_size);
    }

    void average(const std::vector<mv_msg::msg::MotionVector>& mv, double* x, double* y, int* n_mv){
        double mean_x = 0.0;
        double mean_y = 0.0;
        int n = mv.size();
        *n_mv = n;
        for (int i = 0; i < n; i++) {
            mean_x += mv[i].motion_x;
            mean_y += mv[i].motion_y;
        }
        *x = mean_x / n;
        *y = -mean_y / n;
    }

    void update_and_calculate_rms_ate(double current_x, double current_y) {
        double x_offset = -1.0;
        double y_offset = 0.0;
        static std::vector<Point> ground_truth = {
            {0.0 + x_offset, 0.0 + y_offset}, //A
            {0.0 + x_offset, 28.1 + y_offset}, //H
            {15.7 + x_offset, 28.1 + y_offset}, //G
            {15.7 + x_offset, 11.9 + y_offset}, //F
            {31.7 + x_offset, 11.9 + y_offset}, //E
            {31.7 + x_offset, 2.6 + y_offset}, //C
            {8.4 + x_offset, 2.6 + y_offset}, //B
            {0.0 + x_offset, 0.0 + y_offset} //A
        };

        static int num_positions = 0;
        static double sum_squared_errors = 0.0;

        Point current_position = {current_x, current_y};

        double min_dist = INFINITY;
        for (int i = 0; i < int(ground_truth.size() - 1); i++) {
            const Point& p1 = ground_truth[i];
            const Point& p2 = ground_truth[i+1];

            double dist = calculate_perpendicular_distance(current_position, p1, p2);
            if (dist < min_dist) {
                min_dist = dist;
            }
        }

        sum_squared_errors += min_dist * min_dist;
        num_positions++;

        double rms_ate = sqrt(sum_squared_errors / num_positions);
        // Use the calculated RMS ATE value as needed
        std::cout << "RMS ATE: " << rms_ate << "m"<<std::endl;
    }

    double calculate_perpendicular_distance(const Point& p, const Point& a, const Point& b) {
        // Calculate vector AB and AP
        double ABx = b.x - a.x;
        double ABy = b.y - a.y;
        double APx = p.x - a.x;
        double APy = p.y - a.y;

        // Calculate the dot product of AB and AP
        double dot_product = ABx * APx + ABy * APy;

        // Calculate the squared magnitude of AB
        double mag_AB_sq = ABx * ABx + ABy * ABy;

        // Calculate the projection parameter t
        double t = dot_product / mag_AB_sq;

        // Clamp t to the range [0, 1]
        t = std::max(0.0, std::min(1.0, t));

        // Calculate the projection point P'
        Point P_prime = {a.x + t * ABx, a.y + t * ABy};

        // Calculate the distance between P and P'
        double dx = p.x - P_prime.x;
        double dy = p.y - P_prime.y;
        return sqrt(dx * dx + dy * dy);
    }


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odometry>());
    rclcpp::shutdown();
    return 0;
}


