#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <queue>
#include <tuple>
#include <memory>

using namespace std;
using namespace Eigen;

// ----------- New: Message Types and Buffer -----------
enum class SensorType { IMU, RADAR, CAMERA, ALTITUDE, STEP };

// Generic buffer entry for any sensor type
struct TimedMessage {
    rclcpp::Time stamp;
    SensorType type;
    std::shared_ptr<void> msg;

    // For priority_queue: min-heap by time
    bool operator<(const TimedMessage& other) const {
        return stamp > other.stamp;
    }
};

class EKFNode : public rclcpp::Node
{
public:
    EKFNode() : Node("ekf_fusion_node")
    {
        RCLCPP_INFO(get_logger(), "[EKF] Initializing...");

        // ------------ Parameters ------------
        process_accel_noise_      = declare_parameter<double>("process_accel_noise", 0.4);
        process_yaw_accel_noise_  = declare_parameter<double>("process_yaw_accel_noise", 0.06);
        yaw_noise_ = declare_parameter<double>("yaw_noise", 0.01);
        step_variance_ = declare_parameter<double>("step_variance", 5.0);
        step_length_   = declare_parameter<double>("step_length", 0.5);

        imu_delay_      = declare_parameter<double>("imu_delay", 0.0);
        radar_delay_    = declare_parameter<double>("radar_delay", 0.07);
        camera_delay_   = declare_parameter<double>("camera_delay", 0.2);
        altitude_delay_ = declare_parameter<double>("altitude_delay", 0.05);
        step_delay_     = declare_parameter<double>("step_delay", 0.0);

        use_radar_        = declare_parameter<bool>("use_radar", true);
        use_camera_       = declare_parameter<bool>("use_camera", true);
        use_altitude_     = declare_parameter<bool>("use_altitude", true);
        use_step_counter_ = declare_parameter<bool>("use_step_counter", true);

        base_frame_       = declare_parameter<std::string>("base_frame", "base_link");
        world_frame_      = declare_parameter<std::string>("world_frame", "odom");

        // ------------ EKF State ------------
        X_.setZero(8);
        P_.setIdentity(8, 8);
        P_ *= 2.0;
        last_imu_time_ = rclcpp::Time();


        // ------------ Subscribers: now buffer incoming messages ------------
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 100,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                fusion_buffer_.push(TimedMessage{msg->header.stamp, SensorType::IMU, msg});
            });

        radar_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose_estimation", 20,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                fusion_buffer_.push(TimedMessage{msg->header.stamp, SensorType::RADAR, msg});
            });

        camera_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/mvs/pose", 25,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                fusion_buffer_.push(TimedMessage{msg->header.stamp, SensorType::CAMERA, msg});
            });

        altitude_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pressure/pose", 20,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                fusion_buffer_.push(TimedMessage{msg->header.stamp, SensorType::ALTITUDE, msg});
            });

        step_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/step_count", 1,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                // No timestamp in Int32; use node time
                fusion_buffer_.push(TimedMessage{last_imu_time_, SensorType::STEP, msg});
            });

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fused/odom", 10);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/fused/pose", 10);

        // -------------- Timer for Buffer Processing --------------
        fusion_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&EKFNode::processFusionBuffer, this));
       // last_fusion_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
        last_fusion_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME); 
        //last_fusion_time_ = rclcpp::Time();   

        RCLCPP_INFO(get_logger(), "[EKF] Node ready (buffered mode).");
    }

private:
    // ---------------- State & Covariance ----------------
    VectorXd X_;        // [x, y, z, vx, vy, vz, yaw, yaw_rate]
    MatrixXd P_;        // 8x8 covariance

    double process_accel_noise_;     
    double process_yaw_accel_noise_; 
    double radar_pos_noise_;
    double camera_pos_noise_;
    double altitude_noise_;
    double yaw_noise_;
    double step_length_;
    double step_variance_;
    double imu_delay_;
    double radar_delay_;
    double camera_delay_;
    double altitude_delay_;
    double step_delay_;
    bool use_radar_;
    bool use_camera_;
    bool use_altitude_;
    bool use_step_counter_;
    double last_radar_x_ = NAN;
    double last_radar_y_ = NAN;
    double last_camera_x_ = NAN;
    double last_camera_y_ = NAN;
    double last_pressure_z_ = NAN;
    double last_imu_yaw_ = NAN;
    int    last_step_count_ = -1;
    std::string base_frame_;
    std::string world_frame_;
    rclcpp::Time last_imu_time_;
    double reference_x_ = 0.0;  
    double reference_y_ = 0.0;  
    int reference_step_count_ = 0;

    // ------------ New: Fusion Buffer -------------
    std::priority_queue<TimedMessage> fusion_buffer_;
    rclcpp::TimerBase::SharedPtr fusion_timer_;
    rclcpp::Time last_fusion_time_;

    // ------------ ROS2 interfaces ------------
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr radar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr altitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr step_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    // ------------------------------------------------------------------
    //                  Fusion Buffer Processor (NEW)
    // ------------------------------------------------------------------
    void processFusionBuffer() {

        while (!fusion_buffer_.empty()) {
            TimedMessage msg = fusion_buffer_.top();
            // Only process strictly increasing timestamps


            fusion_buffer_.pop();
            last_fusion_time_ = msg.stamp;

            switch (msg.type) {
                case SensorType::IMU:
                    processImuMsg(std::static_pointer_cast<sensor_msgs::msg::Imu>(msg.msg));
                    break;
                case SensorType::RADAR:
                    processRadarMsg(std::static_pointer_cast<geometry_msgs::msg::PoseWithCovarianceStamped>(msg.msg));
                    break;
                case SensorType::CAMERA:
                    processCameraMsg(std::static_pointer_cast<geometry_msgs::msg::PoseWithCovarianceStamped>(msg.msg));
                    break;
                case SensorType::ALTITUDE:
                    processAltitudeMsg(std::static_pointer_cast<geometry_msgs::msg::PoseWithCovarianceStamped>(msg.msg));
                    break;
                case SensorType::STEP:
                    processStepMsg(std::static_pointer_cast<std_msgs::msg::Int32>(msg.msg), msg.stamp);
                    break;
            }
        }
    }

    // ------------------------------------------------------------------
    //                    Motion Model & Covariances
    // ------------------------------------------------------------------

    Matrix<double, 8, 8> computeF(double dt) const
    {
        Matrix<double, 8, 8> F = Matrix<double, 8, 8>::Identity();
        F(0,3) = dt;  F(1,4) = dt;  F(2,5) = dt;
        F(6,7) = dt;
        return F;
    }

    Matrix<double, 8, 8> computeQ(double dt) const
    {
        double svx2   = process_accel_noise_ * process_accel_noise_;    // x
        double svy2   = process_accel_noise_ * process_accel_noise_;    // y
        double svz2   = process_accel_noise_ * process_accel_noise_;    // z
        double syawr2 = process_yaw_accel_noise_ * process_yaw_accel_noise_; // yaw_rate

        Matrix<double, 8, 8> Q = Matrix<double, 8, 8>::Zero();

        // X / vx
        Q(0,0) = svx2 * (dt*dt*dt) / 3.0;
        Q(0,3) = svx2 * (dt*dt)    / 2.0;
        Q(3,0) = Q(0,3);
        Q(3,3) = svx2 * dt;

        // Y / vy
        Q(1,1) = svy2 * (dt*dt*dt) / 3.0;
        Q(1,4) = svy2 * (dt*dt)    / 2.0;
        Q(4,1) = Q(1,4);
        Q(4,4) = svy2 * dt;

        // Z / vz
        Q(2,2) = svz2 * (dt*dt*dt) / 3.0;
        Q(2,5) = svz2 * (dt*dt)    / 2.0;
        Q(5,2) = Q(2,5);
        Q(5,5) = svz2 * dt;

        // Yaw/yaw_rate
        Q(6,6) = syawr2 * (dt*dt*dt) / 3.0;
        Q(6,7) = syawr2 * (dt*dt)    / 2.0;
        Q(7,6) = Q(6,7);
        Q(7,7) = syawr2 * dt;

        return Q;
    }
<<<<<<< HEAD
=======


    /**
     * @brief EKF prediction step
     * @details
     *   X_pred = F * X
     *   P_pred = F * P * F^T + Q
     *
     * @param dt time interval for prediction
     */
>>>>>>> 62aa6bfc0c491f7791b0d879228070aea8eacd29
    void ekfPredict(double dt)
    {
        if (dt < 0.0) dt = 0.0;
        Matrix<double, 8, 8> F = computeF(dt);
        Matrix<double, 8, 8> Q = computeQ(dt);
        X_ = F * X_;
        P_ = F * P_ * F.transpose() + Q;
    }

    void ekfUpdate(const VectorXd &z,
                   const MatrixXd &H,
                   const MatrixXd &R,
                   const VectorXd &h_of_x = VectorXd())
    {
        if (z.size() != H.rows()) return;
        VectorXd h = (h_of_x.size() == 0) ? (H * X_) : h_of_x;
        VectorXd y = z - h;
        MatrixXd S = H * P_ * H.transpose() + R;
        MatrixXd K = P_ * H.transpose() * S.inverse();
        X_ += K * y;
        P_ = (MatrixXd::Identity(8,8) - K * H) * P_;
    }

    // ------------------------------------------------------------------
    //               Per-Sensor Processing Functions
    // ------------------------------------------------------------------
    void processImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        static rclcpp::Time prev_stamp;
        rclcpp::Time msg_time(msg->header.stamp); // FIXED
        if (last_imu_time_.nanoseconds() == 0) {
            last_imu_time_ = msg->header.stamp;
            return;
        }
        double dt = (msg_time - last_imu_time_).seconds() - imu_delay_;
        if (dt < 0.0) dt = 0.0;
        else last_imu_time_ = msg->header.stamp;

        ekfPredict(dt);

        tf2::Quaternion tf_q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        last_imu_yaw_ = yaw;

        VectorXd z(1); z << yaw;
        MatrixXd H(1,8); H.setZero(); H(0,6) = 1.0;
        MatrixXd R(1,1); R(0,0) = yaw_noise_ * yaw_noise_;
        VectorXd h_of_x(1); h_of_x << X_(6);
        ekfUpdate(z, H, R, h_of_x);
        publishOdomAndPose(msg->header.stamp);
        logFusionSummary("IMU", last_radar_x_, last_radar_y_, last_camera_x_, last_camera_y_, last_pressure_z_, last_imu_yaw_, last_step_count_);
    }

    void processRadarMsg(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!use_radar_) return;
        rclcpp::Time msg_time(msg->header.stamp);  // FIXED
        double dt = (msg_time - last_imu_time_).seconds() - radar_delay_;
        if (dt < 0.0) dt = 0.0;
        else last_imu_time_ = msg->header.stamp;
        ekfPredict(dt);
        VectorXd z(2); z << msg->pose.pose.position.x, msg->pose.pose.position.y;
        MatrixXd H(2,8); H.setZero(); H(0,0) = 1.0; H(1,1) = 1.0;
        MatrixXd R(2,2); R.setZero(); R(0,0) = msg->pose.covariance[0]; R(1,1) = msg->pose.covariance[7];
        VectorXd h_of_x(2); h_of_x << X_(0), X_(1);
        ekfUpdate(z, H, R, h_of_x);
        publishOdomAndPose(msg->header.stamp);
        last_radar_x_ = msg->pose.pose.position.x;
        last_radar_y_ = msg->pose.pose.position.y;
        logFusionSummary("Radar", last_radar_x_, last_radar_y_, last_camera_x_, last_camera_y_, last_pressure_z_, last_imu_yaw_, last_step_count_);
    }

    void processCameraMsg(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!use_camera_) return;
        rclcpp::Time msg_time(msg->header.stamp);  // FIXED
        double dt = (msg_time- last_imu_time_).seconds() - camera_delay_;
        if (dt < 0.0) dt = 0.0;
        else last_imu_time_ = msg->header.stamp;
        ekfPredict(dt);
        double cam_x = msg->pose.pose.position.x;
        double cam_y = msg->pose.pose.position.y;
        VectorXd z(2); z << cam_x, cam_y;
        MatrixXd H(2,8); H.setZero(); H(0,0) = 1.0; H(1,1) = 1.0;
        MatrixXd R(2,2); R.setZero(); R(0,0) = msg->pose.covariance[0]; R(1,1) = msg->pose.covariance[7];
        VectorXd h_of_x(2); h_of_x << X_(0), X_(1);
        ekfUpdate(z, H, R, h_of_x);
        publishOdomAndPose(msg->header.stamp);
        last_camera_x_ = cam_x; last_camera_y_ = cam_y;
        logFusionSummary("Camera", last_radar_x_, last_radar_y_, last_camera_x_, last_camera_y_, last_pressure_z_, last_imu_yaw_, last_step_count_);
    }

    void processAltitudeMsg(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!use_altitude_) return;
        rclcpp::Time msg_time(msg->header.stamp);  // FIXED
        double dt = (msg_time- last_imu_time_).seconds() - altitude_delay_;
        if (dt < 0.0) dt = 0.0;
        else last_imu_time_ = msg->header.stamp;
        ekfPredict(dt);
        VectorXd z(1); z << msg->pose.pose.position.z;
        MatrixXd H(1,8); H.setZero(); H(0,2) = 1.0;
        MatrixXd R(1,1); R(0,0) = msg->pose.covariance[14];
        VectorXd h_of_x(1); h_of_x << X_(2);
        ekfUpdate(z, H, R, h_of_x);
        publishOdomAndPose(msg->header.stamp);
        last_pressure_z_ = msg->pose.pose.position.z;
        logFusionSummary("Altitude", last_radar_x_, last_radar_y_, last_camera_x_, last_camera_y_, last_pressure_z_, last_imu_yaw_, last_step_count_);
    }

    void processStepMsg(const std_msgs::msg::Int32::SharedPtr msg, rclcpp::Time stamp)
    {
        if (!use_step_counter_) return;
        double dt = (stamp - last_imu_time_).seconds() - step_delay_;
        if (dt < 0.0) dt = 0.0;
        else last_imu_time_ = stamp;
        ekfPredict(dt);
        int current_steps = msg->data;
        int steps_taken = current_steps - reference_step_count_;
        if (steps_taken <= 0) return;
        double expected_displacement = step_length_ * steps_taken;
        double dx = X_(0) - reference_x_;
        double dy = X_(1) - reference_y_;
        double measured_displacement = std::sqrt(dx * dx + dy * dy);
        if (measured_displacement < 1e-3) return;
        VectorXd z(1); z << expected_displacement;
        MatrixXd H(1,8); H.setZero(); H(0,0) = dx / measured_displacement; H(0,1) = dy / measured_displacement;
        MatrixXd R(1,1); R(0,0) = step_variance_;
        VectorXd h_of_x(1); h_of_x << measured_displacement;
        ekfUpdate(z, H, R, h_of_x);
        if (measured_displacement >= (step_length_ * 0.7)) {
            reference_x_ = X_(0);
            reference_y_ = X_(1);
            reference_step_count_ = current_steps;
        }
        publishOdomAndPose(stamp);
        last_step_count_ = current_steps;
        logFusionSummary("Step Counter", last_radar_x_, last_radar_y_, last_camera_x_, last_camera_y_, last_pressure_z_, last_imu_yaw_, last_step_count_);
    }

    // ---------- Utility (Unchanged) ----------
    double extractYaw(const geometry_msgs::msg::Quaternion &q) const
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    void publishOdomAndPose(const rclcpp::Time &stamp)
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = world_frame_;
        odom_msg.child_frame_id = base_frame_;
        odom_msg.pose.pose.position.x = X_(0);
        odom_msg.pose.pose.position.y = X_(1);
        odom_msg.pose.pose.position.z = X_(2);
        tf2::Quaternion q; q.setRPY(0.0, 0.0, X_(6));
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        odom_msg.twist.twist.linear.x  = X_(3);
        odom_msg.twist.twist.linear.y  = X_(4);
        odom_msg.twist.twist.linear.z  = X_(5);
        odom_msg.twist.twist.angular.z = X_(7);
        for (int i = 0; i < 36; i++) odom_msg.pose.covariance[i] = 0.0;
        odom_msg.pose.covariance[0]   = P_(0,0);   odom_msg.pose.covariance[7]   = P_(1,1);   odom_msg.pose.covariance[14]  = P_(2,2);   odom_msg.pose.covariance[35]  = P_(6,6);
        for (int i = 0; i < 36; i++) odom_msg.twist.covariance[i] = 0.0;
        odom_msg.twist.covariance[0]  = P_(3,3);   odom_msg.twist.covariance[7]  = P_(4,4);   odom_msg.twist.covariance[14] = P_(5,5);   odom_msg.twist.covariance[35] = P_(7,7);
        odom_pub_->publish(odom_msg);
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = odom_msg.header; pose_msg.pose = odom_msg.pose;
        pose_pub_->publish(pose_msg);
    }
    void logFusionSummary(const std::string &source, double radar_x, double radar_y, double cam_x, double cam_y, double pressure_z, double imu_yaw, int step_count)
    {
        double distance = std::sqrt(X_(0) * X_(0) + X_(1) * X_(1));
        RCLCPP_INFO(get_logger(),
            "\n[Source: %s]"
            "\n  Radar   : X=%.2f, Y=%.2f"
            "\n  Camera  : X=%.2f, Y=%.2f"
            "\n  Pressure: Z=%.2f"
            "\n  IMU     : Yaw=%.2f rad"
            "\n  Step    : #%d"
            "\n  FUSED   : X=%.2f, Y=%.2f, Z=%.2f, Yaw=%.2f"
            "\n  Distance: %.2fm | Cov: x=%.3f, y=%.3f, z=%.3f\n",
            source.c_str(), radar_x, radar_y, cam_x, cam_y, pressure_z, imu_yaw, step_count,
            X_(0), X_(1), X_(2), X_(6), distance, P_(0,0), P_(1,1), P_(2,2));
    }

}; // end class EKFNode

// ---------------------------------------------------------------------
//                           main()
// ---------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
