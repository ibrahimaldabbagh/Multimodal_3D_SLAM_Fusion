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

using namespace std;
using namespace Eigen;

/**

ros2 run tf2_ros static_transform_publisher   0 0 0 0 0 0   odom base_link

 *
 * State: X_ = [ x, y, z, vx, vy, vz, yaw, yaw_rate ]
 * 
 * Sensors:
 *  - IMU (used for prediction and optional yaw_rate update)
 *  - Radars subsystem [x, y] (no z or yaw here)
 *  - Cameras subsystem [x, y, z] (no yaw)
 *  - Altitude (pressure sensor) [z]
 *  - Step Counter [distance traveled horizontally in a certain time window]
 *
 * Delays:
 *  - IMU, Radar, Camera, Altitude, Step have constant delays (imu_delay_, etc.).
 *    we subtract the delay from dt
 *    before calling the prediction/update. 
 */
class EKFNode : public rclcpp::Node
{
public:
    EKFNode() : Node("ekf_fusion_node")
    {
        RCLCPP_INFO(get_logger(), "[EKF] Initializing...");

        // ------------- Declare & Get Parameters -------------
        process_accel_noise_      = declare_parameter<double>("process_accel_noise", 0.4);
        process_yaw_accel_noise_  = declare_parameter<double>("process_yaw_accel_noise", 0.06);
        

        yaw_noise_ = declare_parameter<double>("yaw_noise", 0.01);
        step_variance_ = declare_parameter<double>("step_variance", 5.0);
        step_length_   = declare_parameter<double>("step_length", 0.5);

        // Delays
        imu_delay_      = declare_parameter<double>("imu_delay", 0.0);
        radar_delay_    = declare_parameter<double>("radar_delay", 0.0);
        camera_delay_   = declare_parameter<double>("camera_delay", 0.2); // USB buffer delay
        altitude_delay_ = declare_parameter<double>("altitude_delay", 0.05);
        step_delay_     = declare_parameter<double>("step_delay", 0.0);

        use_radar_        = declare_parameter<bool>("use_radar", true);
        use_camera_       = declare_parameter<bool>("use_camera", true);
        use_altitude_     = declare_parameter<bool>("use_altitude", true);
        use_step_counter_ = declare_parameter<bool>("use_step_counter", true);

        base_frame_       = declare_parameter<std::string>("base_frame", "base_link");
        world_frame_      = declare_parameter<std::string>("world_frame", "odom");

        // ------------- Initialize EKF State -------------
        // State: [x, y, z, vx, vy, vz, yaw, yaw_rate]
        X_.setZero(8);
        P_.setIdentity(8, 8);
        P_ *= 2.0; // initial uncertainty

        //last_imu_time_   = this->now();
        last_imu_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);



        // ------------- Create Subscribers -------------
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 100,
            std::bind(&EKFNode::imuCallback, this, std::placeholders::_1));

        radar_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose_estimation", 20,
            std::bind(&EKFNode::radarCallback, this, std::placeholders::_1));

        camera_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/mvs/pose", 25,
            std::bind(&EKFNode::cameraCallback, this, std::placeholders::_1));

        altitude_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pressure/pose", 20,
            std::bind(&EKFNode::altitudeCallback, this, std::placeholders::_1));

        step_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/step_count", 1,
            std::bind(&EKFNode::stepCallback, this, std::placeholders::_1));



        // ------------- Create Publishers -------------
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fused/odom", 10);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/fused/pose", 10);

        RCLCPP_INFO(get_logger(), "[EKF] Node ready.");
    }

private:
    // ---------------- State & Covariance ----------------
    VectorXd X_;        // [x, y, z, vx, vy, vz, yaw, yaw_rate]
    MatrixXd P_;        // 8x8 covariance

    // --------------- EKF Parameters ---------------
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
    
    // Cached sensor values for logging
    double last_radar_x_ = NAN;
    double last_radar_y_ = NAN;
    double last_camera_x_ = NAN;
    double last_camera_y_ = NAN;
    double last_pressure_z_ = NAN;
    double last_imu_yaw_ = NAN;
    int    last_step_count_ = -1;

    
    std::string base_frame_;
    std::string world_frame_;

    // ---------------- Time Tracking ----------------
    rclcpp::Time last_imu_time_;

    // ---------------- Step Measurement References ----------------
    double reference_x_ = 0.0;  
    double reference_y_ = 0.0;  
    int reference_step_count_ = 0; 

    // --------------- ROS2 Interfaces ---------------
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr radar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr altitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr step_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;


    // ------------------------------------------------------------------
    //                    Motion Model & Covariances
    // ------------------------------------------------------------------

    /**
     * @brief Compute the discrete-time transition matrix F (8x8)
     *        for a constant velocity + constant yaw_rate model.
     *
     * X = [ x, y, z, vx, vy, vz, yaw, yaw_rate ]
     */
    Matrix<double, 8, 8> computeF(double dt) const
    {
        Matrix<double, 8, 8> F = Matrix<double, 8, 8>::Identity();

        // Positions updated by velocities
        F(0,3) = dt;  // x depends on vx
        F(1,4) = dt;  // y depends on vy
        F(2,5) = dt;  // z depends on vz

        // Yaw updated by yaw_rate
        F(6,7) = dt;  // yaw depends on yaw_rate

        return F;
    }

    /**
     * @brief Compute the process noise matrix Q (8x8).
     * 
     * @details
     *  process_accel_noise_ is std dev for linear accel in each axis
     *  process_yaw_accel_noise_ is std dev for yaw acceleration
     */
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


    /**
     * @brief EKF prediction step
     * @details
     *   X_pred = F * X
     *   P_pred = F * P * F^T + Q
     *
     * @param dt time interval for prediction
     */
    void ekfPredict(double dt)
    {
        
        if (dt < 0.0) {
            //RCLCPP_WARN(get_logger(), "[EKF] dt < 0.0 => forcing dt=0.0");
            dt = 0.0;
        }
        Matrix<double, 8, 8> F = computeF(dt);
        Matrix<double, 8, 8> Q = computeQ(dt);

        X_ = F * X_;
        P_ = F * P_ * F.transpose() + Q;
    }

    /**
     * @brief EKF update for linear measurement z = H * x + noise
     *
     * @param z         measurement vector
     * @param H         measurement matrix (or Jacobian if nonlinear)
     * @param R         measurement covariance
     * @param h_of_x    optional predicted measurement if needed for nonlinear
     */
    void ekfUpdate(const VectorXd &z,
                   const MatrixXd &H,
                   const MatrixXd &R,
                   const VectorXd &h_of_x = VectorXd())
    {
        if (z.size() != H.rows()) {
            //RCLCPP_ERROR(get_logger(), "[EKF] Dimension mismatch in update!");
            return;
        }

        VectorXd h = (h_of_x.size() == 0) ? (H * X_) : h_of_x;  // predicted measurement
        VectorXd y = z - h;                                     // innovation
        MatrixXd S = H * P_ * H.transpose() + R;                // innovation covariance
        MatrixXd K = P_ * H.transpose() * S.inverse();          // Kalman gain

        X_ += K * y;  // state update
        P_ = (MatrixXd::Identity(8,8) - K * H) * P_;

        //RCLCPP_DEBUG(get_logger(), "[EKF] Update done. z(0)=%.3f", z[0]);
        //RCLCPP_INFO(get_logger(), "Cov: x=%.4f y=%.4f yaw=%.4f", P_(0,0), P_(1,1), P_(6,6));
    }

    // ------------------------------------------------------------------
    //                    Sensor Callbacks
    // ------------------------------------------------------------------

    /**
     * @brief IMU Callback
     * @details
     *   - Compute dt = (IMU stamp - last_imu_time_) - imu_delay_
     *   - ekfPredict(dt)
     *
     *   (No direct yaw or yaw_rate update here, you can add if needed.)
     */
	void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
	{
	    // 1) Compute dt minus IMU delay
	    rclcpp::Time now_stamp = msg->header.stamp;

	    if (last_imu_time_.nanoseconds() == 0) {
		last_imu_time_ = now_stamp;
		return;  // skip prediction on the very first message
	    }

	    double dt = (now_stamp - last_imu_time_).seconds() - imu_delay_;
	    if (dt < 0.0) dt = 0.0;
	    else last_imu_time_ = now_stamp;


	    // 2) EKF Prediction step
	    ekfPredict(dt);

	    // 3) OPTIONAL: Now do an EKF Update for yaw from the IMU orientation.
	    //    The IMU typically provides a stable absolute yaw (within its error),
	    //    so let's treat that as a single-scalar measurement: z = yaw_imu.
	    {
		// Extract yaw from the IMU quaternion
		tf2::Quaternion tf_q(
		    msg->orientation.x,
		    msg->orientation.y,
		    msg->orientation.z,
		    msg->orientation.w
		);
		double roll, pitch, yaw;
		tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        last_imu_yaw_ = yaw;

		// Construct z(1) = [ yaw_imu ]
		VectorXd z(1);
		z << yaw;

		// Measurement matrix: picks out X_(6) from the state
		MatrixXd H(1,8);
		H.setZero();
		H(0,6) = 1.0;

		// Covariance for yaw
		MatrixXd R(1,1);
		R(0,0) = yaw_noise_ * yaw_noise_;

		// Predicted measurement is just the state’s current yaw
		VectorXd h_of_x(1);
		h_of_x << X_(6);

		ekfUpdate(z, H, R, h_of_x);

		    //RCLCPP_INFO(get_logger(), 
        	//"[IMU] Yaw update => measured=%.3f, fused=%.3f (rad)", 
        	//yaw, X_(6));
		publishOdomAndPose(now_stamp);
		logFusionSummary("IMU",
            last_radar_x_, last_radar_y_,
            last_camera_x_, last_camera_y_,
            last_pressure_z_,
            last_imu_yaw_,
            last_step_count_);

	    }

	    //RCLCPP_DEBUG(get_logger(), "[IMU] dt=%.3f => predict+yaw-update done.", dt);
	}


    /**
     * @brief Radar Callback (only [x, y] in your final config)
     */
    void radarCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!use_radar_) return;

        rclcpp::Time sensor_stamp(msg->header.stamp);
        double dt = (sensor_stamp - last_imu_time_).seconds() - radar_delay_;

	if (dt < 0.0) {
	    //RCLCPP_WARN(get_logger(), "[Radar] Negative dt=%.6f => forcing dt=0.0", dt);
	    dt = 0.0;
	} else {
	    last_imu_time_ = sensor_stamp;
	}
	ekfPredict(dt);


        // Radar provides x, y
        VectorXd z(2);
        z << msg->pose.pose.position.x,
             msg->pose.pose.position.y;

        MatrixXd H(2,8);
        H.setZero();
        H(0,0) = 1.0; // X_(0)
        H(1,1) = 1.0; // X_(1)

        MatrixXd R(2,2);
        R.setZero();
        R(0,0) = msg->pose.covariance[0];
        R(1,1) = msg->pose.covariance[7];

        VectorXd h_of_x(2);
        h_of_x << X_(0), X_(1);

        ekfUpdate(z, H, R, h_of_x);

        // Publish
        publishOdomAndPose(sensor_stamp);
        last_radar_x_ = msg->pose.pose.position.x;
	last_radar_y_ = msg->pose.pose.position.y;

	logFusionSummary("Radar",
    	    last_radar_x_, last_radar_y_,
    	    last_camera_x_, last_camera_y_,
    	    last_pressure_z_,
    	    last_imu_yaw_,
    	    last_step_count_);


        // Only have x,y from radar, so no z/yaw in the log:
        //RCLCPP_INFO(get_logger(),
                    //"[Radar] Update: X=%.2f, Y=%.2f",
                    //X_(0), X_(1));
    }

    /**
     * @brief Camera Callback: [x, y, z]
     */
    void cameraCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	{
	    if (!use_camera_) return;

	    // 1) Compute dt minus camera_delay
	    rclcpp::Time sensor_stamp = msg->header.stamp;
	    double dt = (sensor_stamp - last_imu_time_).seconds() - camera_delay_;

	    if (dt < 0.0) {
		   // RCLCPP_WARN(get_logger(), "[Camera] Negative dt=%.6f => forcing dt=0.0", dt);
		    dt = 0.0;
	    } else {
		    last_imu_time_ = sensor_stamp;
	    }
            ekfPredict(dt);
	
	    // 2) Extract camera x, y
	    double cam_x = msg->pose.pose.position.x;
	    double cam_y = msg->pose.pose.position.y;

	    // 3) Build the 2D measurement z = [x, y]
	    VectorXd z(2);
	    z << cam_x, cam_y;

	    // 4) Build the measurement matrix H(2x8).
	    //    We pick out X_(0), X_(1) from [x, y, z, vx, vy, vz, yaw, yaw_rate].
	    MatrixXd H(2, 8);
	    H.setZero();
	    H(0,0) = 1.0;  // x
	    H(1,1) = 1.0;  // y

	    // 5) Build the measurement covariance R(2x2)
	    MatrixXd R(2,2);
	    R.setZero();
        R(0,0) = msg->pose.covariance[0];  // dynamic x covariance from camera
        R(1,1) = msg->pose.covariance[7];  // dynamic y covariance from camera
        

	    // 6) Predicted measurement h_of_x
	    VectorXd h_of_x(2);
	    h_of_x << X_(0),  // predicted x
		      X_(1);  // predicted y

	    // 7) ekfUpdate
	    ekfUpdate(z, H, R, h_of_x);
	        // ------- Optional: Uncomment to also update Z and/or Yaw ---------
		/*
		VectorXd z_ext(4);
		z_ext << msg->pose.pose.position.x,
		         msg->pose.pose.position.y,
		         msg->pose.pose.position.z,
		         extractYaw(msg->pose.pose.orientation);

		MatrixXd H_ext(4,8);
		H_ext.setZero();
		H_ext(0,0) = 1.0;
		H_ext(1,1) = 1.0;
		H_ext(2,2) = 1.0;
		H_ext(3,6) = 1.0;

		MatrixXd R_ext = MatrixXd::Zero(4,4);
		R_ext(0,0) = camera_pos_noise_ * camera_pos_noise_;
		R_ext(1,1) = camera_pos_noise_ * camera_pos_noise_;
		R_ext(2,2) = camera_pos_noise_ * camera_pos_noise_;
		R_ext(3,3) = yaw_noise_ * yaw_noise_;

		VectorXd h_ext(4);
		h_ext << X_(0), X_(1), X_(2), X_(6);

		ekfUpdate(z_ext, H_ext, R_ext, h_ext);
		*/

	    // 8) Publish updated fused state
	    publishOdomAndPose(sensor_stamp);
            last_camera_x_ = msg->pose.pose.position.x;
            last_camera_y_ = msg->pose.pose.position.y;

	    logFusionSummary("Camera",
	        last_radar_x_, last_radar_y_,
	        last_camera_x_, last_camera_y_,
	        last_pressure_z_,
	        last_imu_yaw_,
	        last_step_count_);


	    //RCLCPP_INFO(get_logger(),
		//"[Camera] Update: X=%.2f, Y=%.2f => fused X=%.2f, Y=%.2f",
		//cam_x, cam_y, X_(0), X_(1));
	}



    /**
     * @brief Altitude (pressure) Callback: [z]
     */
    void altitudeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!use_altitude_) return;

        rclcpp::Time sensor_stamp(msg->header.stamp);
        double dt = (sensor_stamp - last_imu_time_).seconds() - altitude_delay_;
	if (dt < 0.0) {
	   // RCLCPP_WARN(get_logger(), "[Altitude] Negative dt=%.6f => forcing dt=0.0", dt);
	    dt = 0.0;
	} else {
	    last_imu_time_ = sensor_stamp;
	}
	ekfPredict(dt);


        VectorXd z(1);
        z << msg->pose.pose.position.z; // altitude

        MatrixXd H(1,8);
        H.setZero();
        H(0,2) = 1.0;  // picking out z

        MatrixXd R(1,1);
        R(0,0) = msg->pose.covariance[14];
        
        VectorXd h_of_x(1);
        h_of_x << X_(2);

        ekfUpdate(z, H, R, h_of_x);

        publishOdomAndPose(sensor_stamp);
        last_pressure_z_ = msg->pose.pose.position.z;

	logFusionSummary("Altitude",
	    last_radar_x_, last_radar_y_,
	    last_camera_x_, last_camera_y_,
	    last_pressure_z_,
	    last_imu_yaw_,
	    last_step_count_);


        //RCLCPP_INFO(get_logger(), "[Altitude] Update: Z=%.2f", X_(2));
    }

    /**
     * @brief Step Counter Callback
     * @details Measures horizontal displacement:
     *     z_step = sqrt((x - x_ref)^2 + (y - y_ref)^2)
     *  Nonlinear, so we pass in H + h_of_x.
     */
    void stepCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!use_step_counter_) return;

	double dt = (this->now() - last_imu_time_).seconds() - step_delay_;
	if (dt < 0.0) {
	    //RCLCPP_WARN(get_logger(), "[Step] Negative dt=%.6f => forcing dt=0.0", dt);
	    dt = 0.0;
	} else {
	    last_imu_time_ = this->now();
	}
	ekfPredict(dt);

        int current_steps = msg->data;
        int steps_taken = current_steps - reference_step_count_;
        if (steps_taken <= 0) return;

        double expected_displacement = step_length_ * steps_taken;

        double dx = X_(0) - reference_x_;
        double dy = X_(1) - reference_y_;
        double measured_displacement = std::sqrt(dx * dx + dy * dy);
        
       // RCLCPP_INFO(get_logger(),
            //        "[Step] dx=%.3f, dy=%.3f, measured=%.3f",
              //      dx, dy, measured_displacement);

        if (measured_displacement < 1e-3) {
            //RCLCPP_WARN(get_logger(), "[Step] Displacement too small, skipping update");
            return;
        }

        VectorXd z(1);
        z << expected_displacement;

        MatrixXd H(1,8);
        H.setZero();
        H(0,0) = dx / measured_displacement; // partial wrt x
        H(0,1) = dy / measured_displacement; // partial wrt y

        MatrixXd R(1,1);
        R(0,0) = step_variance_;

        VectorXd h_of_x(1);
        h_of_x << measured_displacement;

        ekfUpdate(z, H, R, h_of_x);

        // If we traveled at least ~70% of the step length, reset references:
        if (measured_displacement >= (step_length_ * 0.7)) {
            reference_x_ = X_(0);
            reference_y_ = X_(1);
            reference_step_count_ = current_steps;
        }

        // ------------------------------------------------------------------------

        publishOdomAndPose(this->now());
        last_step_count_ = current_steps;

	logFusionSummary("Step Counter",
	    last_radar_x_, last_radar_y_,
	    last_camera_x_, last_camera_y_,
	    last_pressure_z_,
	    last_imu_yaw_,
	    last_step_count_);


        //RCLCPP_INFO(get_logger(),
             //       "[Step] steps=%d, exp=%.2f, act=%.2f => X=%.2f, Y=%.2f",
                //    steps_taken, expected_displacement, measured_displacement, X_(0), X_(1));
    }


    // ------------------------------------------------------------------
    //                     Utility and Publishing
    // ------------------------------------------------------------------

    /**
     * @brief Extract yaw from geometry_msgs Quaternion
     */
    double extractYaw(const geometry_msgs::msg::Quaternion &q) const
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    /**
     * @brief Publish fused odometry and pose
     */
    void publishOdomAndPose(const rclcpp::Time &stamp)
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = world_frame_;
        odom_msg.child_frame_id = base_frame_;

        // Fill pose
        odom_msg.pose.pose.position.x = X_(0);
        odom_msg.pose.pose.position.y = X_(1);
        odom_msg.pose.pose.position.z = X_(2);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, X_(6));
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        // Fill twist
        odom_msg.twist.twist.linear.x  = X_(3);
        odom_msg.twist.twist.linear.y  = X_(4);
        odom_msg.twist.twist.linear.z  = X_(5);
        odom_msg.twist.twist.angular.z = X_(7);

        // Minimal covariance assignment:
        // We'll just store some diagonal entries for demonstration
        for (int i = 0; i < 36; i++)
            odom_msg.pose.covariance[i] = 0.0;

        odom_msg.pose.covariance[0]   = P_(0,0);   // var(x)
        odom_msg.pose.covariance[7]   = P_(1,1);   // var(y)
        odom_msg.pose.covariance[14]  = P_(2,2);   // var(z)
        odom_msg.pose.covariance[35]  = P_(6,6);   // var(yaw)

        for (int i = 0; i < 36; i++)
            odom_msg.twist.covariance[i] = 0.0;

        odom_msg.twist.covariance[0]  = P_(3,3);   // var(vx)
        odom_msg.twist.covariance[7]  = P_(4,4);   // var(vy)
        odom_msg.twist.covariance[14] = P_(5,5);   // var(vz)
        odom_msg.twist.covariance[35] = P_(7,7);   // var(yaw_rate)

        odom_pub_->publish(odom_msg);

        // Also publish PoseWithCovarianceStamped
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = odom_msg.header;
        pose_msg.pose   = odom_msg.pose;

        pose_pub_->publish(pose_msg);
    }
    
    void logFusionSummary(const std::string &source,
                          double radar_x, double radar_y,
                          double cam_x, double cam_y,
                          double pressure_z,
                          double imu_yaw,
                          int step_count)
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
            source.c_str(),
            radar_x, radar_y,
            cam_x, cam_y,
            pressure_z,
            imu_yaw,
            step_count,
            X_(0), X_(1), X_(2), X_(6),
            distance,
            P_(0,0), P_(1,1), P_(2,2)
        );
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