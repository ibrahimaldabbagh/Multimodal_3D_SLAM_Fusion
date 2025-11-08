// Enhanced Fusion Stack with CA Model — Complete Implementation
// Copyright (c) 2025
// License: MIT

/**
 * @file fusion_core_ca_node.cpp
 * @brief Complete multimodal fusion with ESKF-CA and advanced 3D mapping.
 *
 * Features:
 * - 24-state CA model (position, velocity, acceleration + angular counterparts)
 * - Multi-sensor fusion (IMU, Visual, Radar, Barometer, Odometry, Step Counter)
 * - Advanced voxel mapping with TSDF and occupancy
 * - Loop closure detection
 * - Temporal alignment and OOSM handling
 * - Outlier rejection with Mahalanobis gating
 * - Adaptive covariance tuning
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <Eigen/Dense>
#include <queue>
#include <memory>
#include <deque>
#include <chrono>

#include "fusion/eskf_ca.hpp"
#include "fusion/voxel_map_enhanced.hpp"

using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::NavSatFix;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;
using std_msgs::msg::Int32;
using std_msgs::msg::Float64;

namespace fusion {

enum class SensorSource {
  IMU,
  VISUAL_SLAM,
  RADAR_ICP,
  BAROMETER,
  WHEEL_ODO,
  STEP_COUNTER,
  GNSS,
  LIDAR_CLOUD,
  VELOCITY,
  MAGNETIC
};

struct SensorMeasurement {
  rclcpp::Time timestamp;
  SensorSource source;
  std::shared_ptr<void> data;
  double priority{1.0}; // Higher priority processed first for same timestamp
  
  bool operator<(const SensorMeasurement& o) const {
    if (std::abs((timestamp - o.timestamp).seconds()) < 1e-6) {
      return priority < o.priority; // Higher priority first (max-heap behavior inverted)
    }
    return timestamp > o.timestamp; // Earlier first (min-heap)
  }
};

struct PoseKeyframe {
  rclcpp::Time time;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  std::vector<Eigen::Vector3d> local_map;
};

class FusionCoreCANode : public rclcpp::Node {
public:
  FusionCoreCANode() : rclcpp::Node("fusion_core_ca") {
    initializeParameters();
    initializeFilter();
    initializeSubscribers();
    initializePublishers();
    
    // High-frequency processing timer
    timer_ = create_wall_timer(
      std::chrono::microseconds(2000), // 500 Hz
      std::bind(&FusionCoreCANode::processMeasurementBuffer, this));
    
    // Periodic tasks timer
    periodic_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), // 10 Hz
      std::bind(&FusionCoreCANode::periodicTasks, this));
    
    RCLCPP_INFO(get_logger(), "FusionCore CA Node initialized with 24-state model");
    RCLCPP_INFO(get_logger(), "State: [pos(3), vel(3), acc(3), att(3), bg(3), ba(3), w(3), alpha(3)]");
  }

private:
  // ========== INITIALIZATION ==========
  
  void initializeParameters() {
    // Frame IDs
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    
    // Topic names
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    visual_pose_topic_ = declare_parameter<std::string>("visual_pose_topic", "/mvslam/pose");
    radar_pose_topic_ = declare_parameter<std::string>("radar_pose_topic", "/radar/pose");
    baro_topic_ = declare_parameter<std::string>("baro_topic", "/baro/altitude");
    wheel_odo_topic_ = declare_parameter<std::string>("wheel_odo_topic", "/wheel/odom");
    step_topic_ = declare_parameter<std::string>("step_topic", "/step_count");
    gnss_topic_ = declare_parameter<std::string>("gnss_topic", "/gnss/fix");
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/lidar/points");
    velocity_topic_ = declare_parameter<std::string>("velocity_topic", "/velocity");
    
    // ESKF-CA parameters
    eskf_params_.sigma_g = declare_parameter<double>("sigma_gyro", 1.5e-3);
    eskf_params_.sigma_a = declare_parameter<double>("sigma_accel", 3.0e-2);
    eskf_params_.sigma_bg = declare_parameter<double>("sigma_gyro_bias", 1.0e-5);
    eskf_params_.sigma_ba = declare_parameter<double>("sigma_accel_bias", 5.0e-5);
    eskf_params_.sigma_v = declare_parameter<double>("sigma_velocity", 1.0e-3);
    eskf_params_.sigma_acc = declare_parameter<double>("sigma_acceleration", 5.0e-3);
    eskf_params_.sigma_w = declare_parameter<double>("sigma_angular_velocity", 1.0e-4);
    eskf_params_.sigma_alpha = declare_parameter<double>("sigma_angular_accel", 1.0e-3);
    
    // Measurement variances
    visual_pos_var_ = declare_parameter<double>("visual_pos_variance", 0.25);
    visual_yaw_var_ = declare_parameter<double>("visual_yaw_variance", 0.01);
    radar_pos_var_ = declare_parameter<double>("radar_pos_variance", 0.04);
    baro_var_ = declare_parameter<double>("baro_variance", 0.25);
    gnss_hor_var_ = declare_parameter<double>("gnss_horizontal_variance", 4.0);
    gnss_ver_var_ = declare_parameter<double>("gnss_vertical_variance", 9.0);
    wheel_odo_var_ = declare_parameter<double>("wheel_odo_variance", 0.01);
    step_var_ = declare_parameter<double>("step_variance", 0.25);
    velocity_var_ = declare_parameter<double>("velocity_variance", 0.01);
    
    // Outlier rejection
    mahalanobis_gate_ = declare_parameter<double>("mahalanobis_gate", 16.0);
    
    // Mapping parameters
    voxel_size_ = declare_parameter<double>("voxel_size", 0.2);
    max_mapping_range_ = declare_parameter<double>("max_mapping_range", 50.0);
    
    // Loop closure
    enable_loop_closure_ = declare_parameter<bool>("enable_loop_closure", true);
    loop_closure_distance_ = declare_parameter<double>("loop_closure_distance", 5.0);
    
    // Step counter
    step_length_ = declare_parameter<double>("step_length", 0.7);
    
    // GNSS
    use_gnss_ = declare_parameter<bool>("use_gnss", false);
    gnss_origin_set_ = false;
  }
  
  void initializeFilter() {
    eskf_ca_.setParams(eskf_params_);
    
    // Initialize with reasonable defaults
    eskf_ca_.setPosition(Eigen::Vector3d::Zero());
    eskf_ca_.setVelocity(Eigen::Vector3d::Zero());
    eskf_ca_.setOrientation(Eigen::Quaterniond::Identity());
    
    voxel_map_ = std::make_unique<VoxelMapEnhanced>(voxel_size_, max_mapping_range_);
  }
  
  void initializeSubscribers() {
    // IMU: highest priority
    sub_imu_ = create_subscription<Imu>(
      imu_topic_, 400,
      [this](Imu::SharedPtr msg) {
        SensorMeasurement m{msg->header.stamp, SensorSource::IMU, msg, 10.0};
        buffer_.push(m);
      });
    
    // Visual SLAM
    sub_visual_ = create_subscription<PoseWithCovarianceStamped>(
      visual_pose_topic_, 50,
      [this](PoseWithCovarianceStamped::SharedPtr msg) {
        SensorMeasurement m{msg->header.stamp, SensorSource::VISUAL_SLAM, msg, 5.0};
        buffer_.push(m);
      });
    
    // Radar ICP
    sub_radar_ = create_subscription<PoseWithCovarianceStamped>(
      radar_pose_topic_, 50,
      [this](PoseWithCovarianceStamped::SharedPtr msg) {
        SensorMeasurement m{msg->header.stamp, SensorSource::RADAR_ICP, msg, 5.0};
        buffer_.push(m);
      });
    
    // Barometer
    sub_baro_ = create_subscription<Float64>(
      baro_topic_, 20,
      [this](Float64::SharedPtr msg) {
        auto stamped = std::make_shared<PoseWithCovarianceStamped>();
        stamped->header.stamp = now();
        stamped->pose.pose.position.z = msg->data;
        SensorMeasurement m{stamped->header.stamp, SensorSource::BAROMETER, stamped, 3.0};
        buffer_.push(m);
      });
    
    // Wheel odometry
    sub_wheel_odo_ = create_subscription<Odometry>(
      wheel_odo_topic_, 50,
      [this](Odometry::SharedPtr msg) {
        SensorMeasurement m{msg->header.stamp, SensorSource::WHEEL_ODO, msg, 4.0};
        buffer_.push(m);
      });
    
    // Step counter
    sub_step_ = create_subscription<Int32>(
      step_topic_, 20,
      [this](Int32::SharedPtr msg) {
        SensorMeasurement m{last_process_time_, SensorSource::STEP_COUNTER, msg, 2.0};
        buffer_.push(m);
      });
    
    // GNSS
    if (use_gnss_) {
      sub_gnss_ = create_subscription<NavSatFix>(
        gnss_topic_, 10,
        [this](NavSatFix::SharedPtr msg) {
          SensorMeasurement m{msg->header.stamp, SensorSource::GNSS, msg, 3.0};
          buffer_.push(m);
        });
    }
    
    // LiDAR point cloud
    sub_lidar_ = create_subscription<PointCloud2>(
      lidar_topic_, 10,
      [this](PointCloud2::SharedPtr msg) {
        SensorMeasurement m{msg->header.stamp, SensorSource::LIDAR_CLOUD, msg, 1.0};
        buffer_.push(m);
      });
    
    // External velocity measurements
    sub_velocity_ = create_subscription<TwistWithCovarianceStamped>(
      velocity_topic_, 20,
      [this](TwistWithCovarianceStamped::SharedPtr msg) {
        SensorMeasurement m{msg->header.stamp, SensorSource::VELOCITY, msg, 4.0};
        buffer_.push(m);
      });
  }
  
  void initializePublishers() {
    pub_odom_ = create_publisher<Odometry>("fused/odom", 50);
    pub_pose_ = create_publisher<PoseWithCovarianceStamped>("fused/pose", 50);
    pub_path_ = create_publisher<Path>("fused/path", 10);
    pub_velocity_ = create_publisher<TwistWithCovarianceStamped>("fused/velocity", 30);
    pub_acceleration_ = create_publisher<TwistWithCovarianceStamped>("fused/acceleration", 30);
    pub_map_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("fused/map", 1);
    pub_diagnostics_ = create_publisher<std_msgs::msg::String>("fused/diagnostics", 10);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    
    path_.header.frame_id = world_frame_;
  }
  
  // ========== MEASUREMENT PROCESSING ==========
  
  void processMeasurementBuffer() {
    int processed_count = 0;
    const int max_per_cycle = 50; // Prevent blocking
    
    while (!buffer_.empty() && processed_count < max_per_cycle) {
      SensorMeasurement meas = buffer_.top();
      buffer_.pop();
      
      // OOSM handling: drop measurements significantly out of order
      if (filter_initialized_) {
        double dt = (meas.timestamp - last_process_time_).seconds();
        if (dt < -0.1) { // 100ms tolerance
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Dropping late measurement: %.3f seconds old", -dt);
          continue;
        }
      }
      
      last_process_time_ = meas.timestamp;
      
      // Dispatch to appropriate handler
      switch (meas.source) {
        case SensorSource::IMU:
          handleImuMeasurement(std::static_pointer_cast<Imu>(meas.data));
          break;
        case SensorSource::VISUAL_SLAM:
          handleVisualSlamMeasurement(std::static_pointer_cast<PoseWithCovarianceStamped>(meas.data));
          break;
        case SensorSource::RADAR_ICP:
          handleRadarIcpMeasurement(std::static_pointer_cast<PoseWithCovarianceStamped>(meas.data));
          break;
        case SensorSource::BAROMETER:
          handleBarometerMeasurement(std::static_pointer_cast<PoseWithCovarianceStamped>(meas.data));
          break;
        case SensorSource::WHEEL_ODO:
          handleWheelOdometry(std::static_pointer_cast<Odometry>(meas.data));
          break;
        case SensorSource::STEP_COUNTER:
          handleStepCounter(std::static_pointer_cast<Int32>(meas.data));
          break;
        case SensorSource::GNSS:
          handleGnssMeasurement(std::static_pointer_cast<NavSatFix>(meas.data));
          break;
        case SensorSource::LIDAR_CLOUD:
          handleLidarCloud(std::static_pointer_cast<PointCloud2>(meas.data));
          break;
        case SensorSource::VELOCITY:
          handleVelocityMeasurement(std::static_pointer_cast<TwistWithCovarianceStamped>(meas.data));
          break;
        default:
          break;
      }
      
      processed_count++;
    }
    
    if (filter_initialized_ && processed_count > 0) {
      publishState(last_process_time_);
    }
  }
  
  // ========== SENSOR HANDLERS ==========
  
  void handleImuMeasurement(const Imu::SharedPtr& msg) {
    double dt = 0.0;
    if (last_imu_time_.seconds() > 0) {
      dt = (msg->header.stamp - last_imu_time_).seconds();
      if (dt <= 0.0 || dt > 0.5) dt = 0.005; // Default 200Hz
    }
    last_imu_time_ = msg->header.stamp;
    
    Eigen::Vector3d gyro(msg->angular_velocity.x,
                         msg->angular_velocity.y,
                         msg->angular_velocity.z);
    Eigen::Vector3d accel(msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z);
    
    eskf_ca_.propagate(dt, gyro, accel);
    filter_initialized_ = true;
    
    imu_count_++;
  }
  
  void handleVisualSlamMeasurement(const PoseWithCovarianceStamped::SharedPtr& msg) {
    Eigen::Vector3d z_pos(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
    
    // Extract covariance
    Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity() * visual_pos_var_;
    if (msg->pose.covariance[0] > 1e-9) {
      R_pos(0,0) = msg->pose.covariance[0];
      R_pos(1,1) = msg->pose.covariance[7];
      R_pos(2,2) = msg->pose.covariance[14];
    }
    
    // Extract yaw
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Mahalanobis gating
    if (mahalanobisTest(z_pos, eskf_ca_.p(), R_pos, 3)) {
      eskf_ca_.correctPoseWithYaw(z_pos, yaw, R_pos, visual_yaw_var_);
      visual_count_++;
      
      // Store keyframe for loop closure
      storeKeyframe(msg->header.stamp, z_pos,
                   Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Visual SLAM measurement rejected (Mahalanobis gate)");
    }
  }
  
  void handleRadarIcpMeasurement(const PoseWithCovarianceStamped::SharedPtr& msg) {
    Eigen::Vector3d z_pos(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
    
    Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity() * radar_pos_var_;
    if (msg->pose.covariance[0] > 1e-9) {
      R_pos(0,0) = msg->pose.covariance[0];
      R_pos(1,1) = msg->pose.covariance[7];
      R_pos(2,2) = msg->pose.covariance[14];
    }
    
    if (mahalanobisTest(z_pos, eskf_ca_.p(), R_pos, 3)) {
      eskf_ca_.correctPosition(z_pos, R_pos);
      radar_count_++;
    }
  }
  
  void handleBarometerMeasurement(const PoseWithCovarianceStamped::SharedPtr& msg) {
    double z_alt = msg->pose.pose.position.z;
    eskf_ca_.correctAltitude(z_alt, baro_var_);
    baro_count_++;
  }
  
  void handleWheelOdometry(const Odometry::SharedPtr& msg) {
    // Extract velocity in body frame
    Eigen::Vector3d v_body(msg->twist.twist.linear.x,
                           msg->twist.twist.linear.y,
                           msg->twist.twist.linear.z);
    
    // Transform to world frame
    Eigen::Matrix3d R_wb = eskf_ca_.q().toRotationMatrix();
    Eigen::Vector3d v_world = R_wb * v_body;
    
    Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * wheel_odo_var_;
    if (msg->twist.covariance[0] > 1e-9) {
      R_vel(0,0) = msg->twist.covariance[0];
      R_vel(1,1) = msg->twist.covariance[7];
      R_vel(2,2) = msg->twist.covariance[14];
    }
    
    eskf_ca_.correctVelocity(v_world, R_vel);
    wheel_odo_count_++;
  }
  
  void handleStepCounter(const Int32::SharedPtr& msg) {
    int current_steps = msg->data;
    
    if (last_step_count_ >= 0) {
      int delta_steps = current_steps - last_step_count_;
      if (delta_steps > 0 && delta_steps < 10) { // Sanity check
        accumulated_step_distance_ += delta_steps * step_length_;
        eskf_ca_.correctAlongTrack(accumulated_step_distance_, step_var_);
        step_count_++;
      }
    }
    
    last_step_count_ = current_steps;
  }
  
  void handleGnssMeasurement(const NavSatFix::SharedPtr& msg) {
    if (msg->status.status < 0) return; // No fix
    
    // Set origin on first valid GNSS
    if (!gnss_origin_set_) {
      gnss_origin_lat_ = msg->latitude;
      gnss_origin_lon_ = msg->longitude;
      gnss_origin_alt_ = msg->altitude;
      gnss_origin_set_ = true;
      RCLCPP_INFO(get_logger(), "GNSS origin set: lat=%.6f, lon=%.6f, alt=%.2f",
                  gnss_origin_lat_, gnss_origin_lon_, gnss_origin_alt_);
      return;
    }
    
    // Convert to local ENU coordinates
    Eigen::Vector3d enu = llaToEnu(msg->latitude, msg->longitude, msg->altitude);
    
    Eigen::Matrix3d R_gnss = Eigen::Matrix3d::Identity();
    R_gnss(0,0) = gnss_hor_var_;
    R_gnss(1,1) = gnss_hor_var_;
    R_gnss(2,2) = gnss_ver_var_;
    
    if (mahalanobisTest(enu, eskf_ca_.p(), R_gnss, 3)) {
      eskf_ca_.correctPosition(enu, R_gnss);
      gnss_count_++;
    }
  }
  
  void handleLidarCloud(const PointCloud2::SharedPtr& msg) {
    auto points = parsePointCloud(msg);
    if (points.empty()) return;
    
    // Transform to world frame
    Eigen::Matrix3d R = eskf_ca_.q().toRotationMatrix();
    Eigen::Vector3d t = eskf_ca_.p();
    
    std::vector<Eigen::Vector3d> points_world;
    points_world.reserve(points.size());
    
    for (const auto& pt : points) {
      points_world.push_back(R * pt + t);
    }
    
    // Insert into map
    voxel_map_->insertPointCloud(points_world, t);
    
    // Periodic visualization
    if (++map_vis_counter_ % 10 == 0) {
      publishMapVisualization(msg->header.stamp);
    }
    
    lidar_count_++;
  }
  
  void handleVelocityMeasurement(const TwistWithCovarianceStamped::SharedPtr& msg) {
    Eigen::Vector3d v_meas(msg->twist.twist.linear.x,
                           msg->twist.twist.linear.y,
                           msg->twist.twist.linear.z);
    
    Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * velocity_var_;
    if (msg->twist.covariance[0] > 1e-9) {
      R_vel(0,0) = msg->twist.covariance[0];
      R_vel(1,1) = msg->twist.covariance[7];
      R_vel(2,2) = msg->twist.covariance[14];
    }
    
    eskf_ca_.correctVelocity(v_meas, R_vel);
  }
  
  // ========== PUBLISHING ==========
  
  void publishState(const rclcpp::Time& stamp) {
    const auto& P = eskf_ca_.P();
    
    // Odometry message
    Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = base_frame_;
    
    // Position
    odom.pose.pose.position.x = eskf_ca_.p().x();
    odom.pose.pose.position.y = eskf_ca_.p().y();
    odom.pose.pose.position.z = eskf_ca_.p().z();
    
    // Orientation
    odom.pose.pose.orientation = tf2::toMsg(eskf_ca_.q());
    
    // Position covariance
    odom.pose.covariance[0] = P(0,0);
    odom.pose.covariance[7] = P(1,1);
    odom.pose.covariance[14] = P(2,2);
    
    // Orientation covariance
    odom.pose.covariance[21] = P(9,9);
    odom.pose.covariance[28] = P(10,10);
    odom.pose.covariance[35] = P(11,11);
    
    // Velocity
    odom.twist.twist.linear.x = eskf_ca_.v().x();
    odom.twist.twist.linear.y = eskf_ca_.v().y();
    odom.twist.twist.linear.z = eskf_ca_.v().z();
    
    // Angular velocity
    odom.twist.twist.angular.x = eskf_ca_.w().x();
    odom.twist.twist.angular.y = eskf_ca_.w().y();
    odom.twist.twist.angular.z = eskf_ca_.w().z();
    
    // Velocity covariance
    odom.twist.covariance[0] = P(3,3);
    odom.twist.covariance[7] = P(4,4);
    odom.twist.covariance[14] = P(5,5);
    
    pub_odom_->publish(odom);
    
    // Pose message
    PoseWithCovarianceStamped pose_msg;
    pose_msg.header = odom.header;
    pose_msg.pose = odom.pose;
    pub_pose_->publish(pose_msg);
    
    // Velocity message with acceleration
    TwistWithCovarianceStamped vel_msg;
    vel_msg.header = odom.header;
    vel_msg.twist.twist = odom.twist.twist;
    vel_msg.twist.covariance = odom.twist.covariance;
    pub_velocity_->publish(vel_msg);
    
    // Acceleration message
    TwistWithCovarianceStamped acc_msg;
    acc_msg.header = odom.header;
    acc_msg.twist.twist.linear.x = eskf_ca_.a().x();
    acc_msg.twist.twist.linear.y = eskf_ca_.a().y();
    acc_msg.twist.twist.linear.z = eskf_ca_.a().z();
    acc_msg.twist.twist.angular.x = eskf_ca_.alpha().x();
    acc_msg.twist.twist.angular.y = eskf_ca_.alpha().y();
    acc_msg.twist.twist.angular.z = eskf_ca_.alpha().z();
    acc_msg.twist.covariance[0] = P(6,6);
    acc_msg.twist.covariance[7] = P(7,7);
    acc_msg.twist.covariance[14] = P(8,8);
    pub_acceleration_->publish(acc_msg);
    
    // Path
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odom.header;
    pose_stamped.pose = odom.pose.pose;
    path_.poses.push_back(pose_stamped);
    
    // Downsample path if too long
    if (path_.poses.size() > 10000) {
      std::vector<geometry_msgs::msg::PoseStamped> downsampled;
      for (size_t i = 0; i < path_.poses.size(); i += 2) {
        downsampled.push_back(path_.poses[i]);
      }
      path_.poses = downsampled;
    }
    
    path_.header = odom.header;
    pub_path_->publish(path_);
    
    // TF broadcast
    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.translation.z = odom.pose.pose.position.z;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }
  
  void publishMapVisualization(const rclcpp::Time& stamp) {
    auto occupied_voxels = voxel_map_->cellsAbove(0.5f);
    
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    
    marker.header.frame_id = world_frame_;
    marker.header.stamp = stamp;
    marker.ns = "occupancy_map";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = voxel_size_;
    marker.scale.y = voxel_size_;
    marker.scale.z = voxel_size_;
    marker.color.a = 0.7;
    marker.color.r = 0.2;
    marker.color.g = 0.7;
    marker.color.b = 0.9;
    
    marker.points.reserve(occupied_voxels.size());
    marker.colors.reserve(occupied_voxels.size());
    
    for (const auto& [key, data] : occupied_voxels) {
      geometry_msgs::msg::Point p;
      Eigen::Vector3d pos = voxel_map_->keyToPosition(key);
      p.x = pos.x() + voxel_size_ * 0.5;
      p.y = pos.y() + voxel_size_ * 0.5;
      p.z = pos.z() + voxel_size_ * 0.5;
      marker.points.push_back(p);
      
      std_msgs::msg::ColorRGBA color;
      color.r = data.color.x();
      color.g = data.color.y();
      color.b = data.color.z();
      color.a = 0.7;
      marker.colors.push_back(color);
    }
    
    marker_array.markers.push_back(marker);
    pub_map_markers_->publish(marker_array);
  }
  
  // ========== PERIODIC TASKS ==========
  
  void periodicTasks() {
    if (!filter_initialized_) return;
    
    // Publish diagnostics
    publishDiagnostics();
    
    // Loop closure detection
    if (enable_loop_closure_ && keyframes_.size() > 10) {
      detectLoopClosures();
    }
    
    // Prune old voxels (every 5 seconds)
    static int prune_counter = 0;
    if (++prune_counter >= 50) {
      voxel_map_->pruneOldVoxels(1000); // Keep last 1000 updates
      prune_counter = 0;
    }
  }
  
  void publishDiagnostics() {
    std_msgs::msg::String diag_msg;
    std::stringstream ss;
    
    ss << "=== Fusion Core CA Diagnostics ===\n";
    ss << "State: [" << eskf_ca_.p().transpose() << "]\n";
    ss << "Velocity: [" << eskf_ca_.v().transpose() << "]\n";
    ss << "Acceleration: [" << eskf_ca_.a().transpose() << "]\n";
    ss << "Angular Vel: [" << eskf_ca_.w().transpose() << "]\n";
    ss << "Angular Acc: [" << eskf_ca_.alpha().transpose() << "]\n";
    ss << "Gyro Bias: [" << eskf_ca_.bg().transpose() << "]\n";
    ss << "Accel Bias: [" << eskf_ca_.ba().transpose() << "]\n";
    ss << "\nMeasurement Counts:\n";
    ss << "  IMU: " << imu_count_ << "\n";
    ss << "  Visual: " << visual_count_ << "\n";
    ss << "  Radar: " << radar_count_ << "\n";
    ss << "  Baro: " << baro_count_ << "\n";
    ss << "  Wheel: " << wheel_odo_count_ << "\n";
    ss << "  Steps: " << step_count_ << "\n";
    ss << "  GNSS: " << gnss_count_ << "\n";
    ss << "  LiDAR: " << lidar_count_ << "\n";
    ss << "\nMap Size: " << voxel_map_->size() << " voxels\n";
    ss << "Keyframes: " << keyframes_.size() << "\n";
    ss << "Buffer Size: " << buffer_.size() << "\n";
    
    diag_msg.data = ss.str();
    pub_diagnostics_->publish(diag_msg);
  }
  
  // ========== LOOP CLOSURE ==========
  
  void storeKeyframe(const rclcpp::Time& time,
                    const Eigen::Vector3d& position,
                    const Eigen::Quaterniond& orientation) {
    // Store keyframe with local map
    PoseKeyframe kf;
    kf.time = time;
    kf.position = position;
    kf.orientation = orientation;
    
    // Extract local map around keyframe
    kf.local_map = extractLocalMapPoints(position, 10.0);
    
    keyframes_.push_back(kf);
    
    // Limit keyframe history
    if (keyframes_.size() > 1000) {
      keyframes_.pop_front();
    }
  }
  
  void detectLoopClosures() {
    if (keyframes_.size() < 20) return;
    
    const auto& current_kf = keyframes_.back();
    
    // Check against older keyframes
    for (size_t i = 0; i < keyframes_.size() - 20; ++i) {
      const auto& old_kf = keyframes_[i];
      
      double distance = (current_kf.position - old_kf.position).norm();
      
      if (distance < loop_closure_distance_) {
        // Potential loop closure detected
        double match_score = computeMapMatchScore(current_kf.local_map, old_kf.local_map);
        
        if (match_score > 0.6) { // Threshold for accepting loop closure
          RCLCPP_INFO(get_logger(), "Loop closure detected! Distance: %.2fm, Score: %.3f",
                     distance, match_score);
          
          // Apply loop closure correction
          applyLoopClosure(current_kf, old_kf);
          break; // Only one loop closure per cycle
        }
      }
    }
  }
  
  double computeMapMatchScore(const std::vector<Eigen::Vector3d>& map1,
                             const std::vector<Eigen::Vector3d>& map2) {
    if (map1.empty() || map2.empty()) return 0.0;
    
    // Simple ICP-like score based on nearest neighbor distances
    double total_distance = 0.0;
    int matches = 0;
    const double max_correspondence_dist = 1.0;
    
    for (const auto& pt1 : map1) {
      double min_dist = std::numeric_limits<double>::max();
      for (const auto& pt2 : map2) {
        double dist = (pt1 - pt2).norm();
        if (dist < min_dist) min_dist = dist;
      }
      
      if (min_dist < max_correspondence_dist) {
        total_distance += min_dist;
        matches++;
      }
    }
    
    if (matches == 0) return 0.0;
    
    double avg_distance = total_distance / matches;
    double overlap_ratio = static_cast<double>(matches) / map1.size();
    
    // Score: high overlap, low average distance
    return overlap_ratio * (1.0 - std::min(avg_distance / max_correspondence_dist, 1.0));
  }
  
  void applyLoopClosure(const PoseKeyframe& current, const PoseKeyframe& loop) {
    // Compute pose correction
    Eigen::Vector3d pos_error = loop.position - current.position;
    
    // Apply correction as a position measurement with high confidence
    Eigen::Matrix3d R_loop = Eigen::Matrix3d::Identity() * 0.01; // High confidence
    eskf_ca_.correctPosition(loop.position, R_loop);
    
    RCLCPP_INFO(get_logger(), "Applied loop closure correction: [%.2f, %.2f, %.2f]",
               pos_error.x(), pos_error.y(), pos_error.z());
  }
  
  std::vector<Eigen::Vector3d> extractLocalMapPoints(const Eigen::Vector3d& center, 
                                                     double radius) {
    auto local_voxels = voxel_map_->getLocalMap(center, radius);
    
    std::vector<Eigen::Vector3d> points;
    points.reserve(local_voxels.size());
    
    for (const auto& [key, data] : local_voxels) {
      if (data.isOccupied()) {
        points.push_back(voxel_map_->keyToPosition(key));
      }
    }
    
    return points;
  }
  
  // ========== UTILITY FUNCTIONS ==========
  
  bool mahalanobisTest(const Eigen::Vector3d& z, const Eigen::Vector3d& z_pred,
                      const Eigen::Matrix3d& R, int dof) {
    Eigen::Vector3d innov = z - z_pred;
    double mahal_dist = innov.transpose() * R.inverse() * innov;
    
    // Chi-squared test
    return mahal_dist < mahalanobis_gate_ * dof;
  }
  
  Eigen::Vector3d llaToEnu(double lat, double lon, double alt) {
    // Simple flat-earth approximation (valid for small areas)
    const double R_earth = 6378137.0; // meters
    
    double dlat = (lat - gnss_origin_lat_) * M_PI / 180.0;
    double dlon = (lon - gnss_origin_lon_) * M_PI / 180.0;
    double dalt = alt - gnss_origin_alt_;
    
    double x = R_earth * dlon * std::cos(gnss_origin_lat_ * M_PI / 180.0);
    double y = R_earth * dlat;
    double z = dalt;
    
    return Eigen::Vector3d(x, y, z);
  }
  
  std::vector<Eigen::Vector3d> parsePointCloud(const PointCloud2::SharedPtr& msg) {
    std::vector<Eigen::Vector3d> points;
    
    if (msg->fields.size() < 3) return points;
    
    // Find x, y, z field offsets
    int offset_x = -1, offset_y = -1, offset_z = -1;
    for (const auto& field : msg->fields) {
      if (field.name == "x") offset_x = field.offset;
      else if (field.name == "y") offset_y = field.offset;
      else if (field.name == "z") offset_z = field.offset;
    }
    
    if (offset_x < 0 || offset_y < 0 || offset_z < 0) return points;
    
    points.reserve(msg->width * msg->height);
    
    for (size_t i = 0; i < msg->width * msg->height; ++i) {
      size_t idx = i * msg->point_step;
      
      if (idx + offset_z + sizeof(float) > msg->data.size()) break;
      
      float x = *reinterpret_cast<const float*>(&msg->data[idx + offset_x]);
      float y = *reinterpret_cast<const float*>(&msg->data[idx + offset_y]);
      float z = *reinterpret_cast<const float*>(&msg->data[idx + offset_z]);
      
      // Filter invalid points
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        points.emplace_back(x, y, z);
      }
    }
    
    return points;
  }
  
  // ========== MEMBER VARIABLES ==========
  
  // Filter
  ESKF_CA eskf_ca_;
  EskfCAParams eskf_params_;
  std::unique_ptr<VoxelMapEnhanced> voxel_map_;
  
  // State
  bool filter_initialized_{false};
  rclcpp::Time last_process_time_;
  rclcpp::Time last_imu_time_;
  
  // Measurement buffer
  std::priority_queue<SensorMeasurement> buffer_;
  
  // Loop closure
  std::deque<PoseKeyframe> keyframes_;
  bool enable_loop_closure_;
  double loop_closure_distance_;
  
  // Step counter state
  int last_step_count_{-1};
  double accumulated_step_distance_{0.0};
  double step_length_;
  
  // GNSS state
  bool use_gnss_;
  bool gnss_origin_set_;
  double gnss_origin_lat_, gnss_origin_lon_, gnss_origin_alt_;
  
  // Statistics
  uint64_t imu_count_{0};
  uint64_t visual_count_{0};
  uint64_t radar_count_{0};
  uint64_t baro_count_{0};
  uint64_t wheel_odo_count_{0};
  uint64_t step_count_{0};
  uint64_t gnss_count_{0};
  uint64_t lidar_count_{0};
  int map_vis_counter_{0};
  
  // ROS interfaces
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_visual_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_radar_;
  rclcpp::Subscription<Float64>::SharedPtr sub_baro_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_wheel_odo_;
  rclcpp::Subscription<Int32>::SharedPtr sub_step_;
  rclcpp::Subscription<NavSatFix>::SharedPtr sub_gnss_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr sub_velocity_;
  
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<Path>::SharedPtr pub_path_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr pub_velocity_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr pub_acceleration_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_map_markers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_diagnostics_;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr periodic_timer_;
  
  // Parameters
  std::string world_frame_, base_frame_, imu_frame_;
  std::string imu_topic_, visual_pose_topic_, radar_pose_topic_;
  std::string baro_topic_, wheel_odo_topic_, step_topic_;
  std::string gnss_topic_, lidar_topic_, velocity_topic_;
  
  double visual_pos_var_, visual_yaw_var_, radar_pos_var_;
  double baro_var_, gnss_hor_var_, gnss_ver_var_;
  double wheel_odo_var_, step_var_, velocity_var_;
  double mahalanobis_gate_;
  double voxel_size_, max_mapping_range_;
  
  Path path_;
};

} // namespace fusion

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fusion::FusionCoreCANode>());
  rclcpp::shutdown();
  return 0;
}