
/**
 * @file quadmmwave_icp_core.cpp
 * @brief Core pipeline for 4‑Directional mmWave Radar SLAM using ICP with IMU-aided orientation,
 *        adaptive filtering, robust correspondence rejection, and automatic keyframe mapping.
 *
 * DESIGN (completely new, merged, and modernized):
 *  • Subscribes to FOUR independent radar clouds + one IMU topic.
 *  • Applies per-sensor extrinsics to bring every radar cloud into the robot base frame.
 *  • Fuses the four clouds, then applies adaptive VoxelGrid + StatisticalOutlierRemoval.
 *  • Runs Robust ICP (with reciprocal correspondences, RANSAC outlier rejection,
 *    and adaptive max correspondence distance). Falls back to NDT mapping if ICP quality drops.
 *  • Integrates translation in the odom frame using the IMU quaternion for attitude.
 *  • Maintains a local keyframe map: when the robot moves far enough, the current clean cloud
 *    becomes the next target to limit drift and preserve geometry.
 *
 *  PUBLISHES:
 *   - /quadmmwave/points/fused      : fused raw cloud (base frame)
 *   - /quadmmwave/points/clean      : filtered cloud (base frame)
 *   - /quadmmwave/points/aligned    : current source aligned into odom frame (for visualization)
 *   - /quadmmwave/pose              : accumulated pose (PoseWithCovarianceStamped, odom frame)
 *   - /quadmmwave/pose_delta        : step delta pose (PoseWithCovarianceStamped, odom frame)
 *
 *  PARAMETERS (see YAML for examples):
 *   input_topic_[0..3]     : radar input topics
 *   imu_topic              : IMU input topic
 *   base_frame / odom_frame: reference frames
 *   sensor{i}_extrinsics   : [x y z rx ry rz] meters/radians extrinsics per radar
 *   x_rot_deg              : optional vendor-to-robot rotation around X (deg)
 *   voxel_min / voxel_max  : bounds for adaptive voxel leaf size (m)
 *   sor_k / sor_std        : SOR parameters
 *   pass_radius_min/max    : range gating (m)
 *   pass_z_min/max         : height gating (m)
 *   pass_intensity_min     : intensity gating
 *   icp_*                  : ICP tuning (iterations, fitness epsilon, etc.)
 *   icp_adapt_corr_{min,max}: bounds for adaptive max correspondence distance (m)
 *   ransac_thresh          : RANSAC outlier rejection threshold (m)
 *   scale_correction       : optional scale fudge (1.0 = off)
 *   keyframe_{trans,rot}   : when exceeded, promote current cloud to new keyframe target
 *   use_ndt_fallback       : if true, attempt NDT registration when ICP is weak
 *
 *  BUILD: rclcpp, sensor_msgs, geometry_msgs, pcl_conversions, PCL(common,filters,registration), OPTIONAL PCL NDT
 */

#define PCL_NO_PRECOMPILE
#include "rclcpp/rclcpp.hpp"                                       // ROS 2 base
#include <sensor_msgs/msg/point_cloud2.hpp>                        // Cloud messages
#include <sensor_msgs/msg/imu.hpp>                                 // IMU messages
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>      // Pose messages
#include <pcl/point_types.h>                                       // PCL point types
#include <pcl/point_cloud.h>                                       // PCL cloud container
#include <pcl/filters/statistical_outlier_removal.h>               // SOR filter
#include <pcl/filters/voxel_grid.h>                                // VoxelGrid filter
#include <pcl/registration/transforms.h>                           // Transform utilities
#include <pcl/registration/icp.h>                                  // ICP
#include <pcl/registration/ndt.h>                                  // NDT (fallback)
#include <pcl_conversions/pcl_conversions.h>                       // ROS <-> PCL conversion
#include <Eigen/Geometry>                                          // Eigen math
#include <array>
#include <string>
#include <algorithm>
#include <cmath>

// ---------------------- Custom radar point ----------------------
// A dedicated point type with XYZ + intensity + radial velocity.
// This is common for mmWave radar fused point clouds.
struct RadarXYZIV {
  PCL_ADD_POINT4D;     // adds float x,y,z; padding; 16-byte alignment
  float intensity;     // mmWave reported SNR/Power proxy
  float velocity;      // radial velocity (m/s) if available; can be 0
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Register the point structure so PCL understands it
POINT_CLOUD_REGISTER_POINT_STRUCT(RadarXYZIV,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, velocity, velocity))

// Point aliases for convenience
using PointT = RadarXYZIV;
using CloudT = pcl::PointCloud<PointT>;

// ---------------------- Small math helpers ----------------------
// Build a transform from translation (x,y,z) and Euler RPY (rx,ry,rz)
static inline Eigen::Affine3f xyzrpy(float x, float y, float z, float rx, float ry, float rz){
  Eigen::Affine3f T = Eigen::Affine3f::Identity();
  T.translation() << x, y, z;
  T.rotate(Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX()));
  T.rotate(Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY()));
  T.rotate(Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()));
  return T;
}

// Compute Euclidean norm of a 3D vector
static inline float vec3_norm(const Eigen::Vector3f& v){ return std::sqrt(v.x()*v.x() + v.y()*v.y() + v.z()*v.z()); }

class QuadMmwaveIcpCore : public rclcpp::Node {
public:
  QuadMmwaveIcpCore() : rclcpp::Node("quadmmwave_icp_core") {
    // -------- Parameters: topics --------
    for (int i=0;i<4;++i){
      input_topics_[i] = declare_parameter<std::string>("input_topic_"+std::to_string(i), "/radar/scan_"+std::to_string(i)); // default topic per radar
    }
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");         // IMU topic for orientation

    // -------- Parameters: frames --------
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");       // robot base frame
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");            // odom/world frame

    // -------- Parameters: extrinsics --------
    for (int i=0;i<4;++i){
      auto v = declare_parameter<std::vector<double>>("sensor"+std::to_string(i)+"_extrinsics", {0,0,0,0,0,0}); // x y z rx ry rz
      if (v.size()!=6) throw std::runtime_error("sensor{i}_extrinsics requires 6 values [x y z rx ry rz]");
      extrinsics_[i] = xyzrpy(v[0],v[1],v[2],v[3],v[4],v[5]);                      // precompute transform
    }
    x_rot_deg_ = declare_parameter<double>("x_rot_deg", 90.0);                      // optional radar->robot rotation around X

    // -------- Parameters: filtering (adaptive voxel + SOR + pass) --------
    voxel_min_ = declare_parameter<double>("voxel_min", 0.06);                      // minimum voxel leaf size (meters)
    voxel_max_ = declare_parameter<double>("voxel_max", 0.16);                      // maximum voxel leaf size (meters)
    sor_k_     = declare_parameter<int>("sor_k", 50);                               // SOR mean K
    sor_std_   = declare_parameter<double>("sor_std", 1.3);                         // SOR stddev multiplier
    rmin_      = declare_parameter<double>("pass_radius_min", 0.25);                // min range gate (m)
    rmax_      = declare_parameter<double>("pass_radius_max", 14.0);                // max range gate (m)
    zmin_      = declare_parameter<double>("pass_z_min", -3.0);                     // min z (m)
    zmax_      = declare_parameter<double>("pass_z_max", 3.0);                      // max z (m)
    imin_      = declare_parameter<double>("pass_intensity_min", 6.0);              // min intensity threshold

    // -------- Parameters: ICP (robust) --------
    icp_iter_  = declare_parameter<int>("icp_max_iterations", 60);                  // fewer iters with better filtering
    fit_eps_   = declare_parameter<double>("icp_fitness_epsilon", 1e-5);            // convergence epsilon
    trans_eps_ = declare_parameter<double>("icp_transformation_epsilon", 1e-9);     // transformation epsilon
    corr_min_  = declare_parameter<double>("icp_adapt_corr_min", 0.15);             // min max-correspondence distance (m)
    corr_max_  = declare_parameter<double>("icp_adapt_corr_max", 0.60);             // max max-correspondence distance (m)
    ransac_th_ = declare_parameter<double>("ransac_thresh", 0.08);                  // RANSAC outlier rejection threshold (m)
    reciprocal_= declare_parameter<bool>("use_reciprocal_corr", true);              // use reciprocal correspondences

    // -------- Parameters: scale + keyframes + fallback --------
    scale_     = declare_parameter<double>("scale_correction", 1.0);                // scale fudge (1.0 = off)
    kf_trans_  = declare_parameter<double>("keyframe_trans_thresh", 0.80);          // meters to trigger new keyframe
    kf_rot_    = declare_parameter<double>("keyframe_rot_thresh_deg", 10.0);        // degrees to trigger new keyframe
    use_ndt_   = declare_parameter<bool>("use_ndt_fallback", true);                 // enable NDT fallback

    // -------- Publishers --------
    pub_fused_   = create_publisher<sensor_msgs::msg::PointCloud2>("/quadmmwave/points/fused",   3);   // fused cloud (debug)
    pub_clean_   = create_publisher<sensor_msgs::msg::PointCloud2>("/quadmmwave/points/clean",   3);   // filtered cloud
    pub_aligned_ = create_publisher<sensor_msgs::msg::PointCloud2>("/quadmmwave/points/aligned", 3);   // aligned cloud (odom)
    pub_pose_    = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/quadmmwave/pose", 3);
    pub_delta_   = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/quadmmwave/pose_delta", 3);

    // -------- Subscriptions --------
    // Radar cloud inputs: copy to buffers, defer compute to timer callback
    for (int i=0;i<4;++i){
      subs_[i] = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topics_[i], rclcpp::SensorDataQoS(),
        [this,i](const sensor_msgs::msg::PointCloud2::SharedPtr msg){
          pcl::fromROSMsg(*msg, sensors_[i]);             // convert ROS2 cloud to PCL cloud
          last_stamp_ = msg->header.stamp;                 // keep last timestamp for publishing
        });
    }
    // IMU input: keep normalized orientation quaternion
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 100,
      [this](const sensor_msgs::msg::Imu& m){
        imu_q_ = Eigen::Quaternionf(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z).normalized();
      });

    // -------- Processing loop timer --------
    timer_ = create_wall_timer(std::chrono::milliseconds(66), std::bind(&QuadMmwaveIcpCore::tick, this)); // ~15 Hz
  }

private:
  // ------------------- Node state members -------------------
  std::string input_topics_[4];        // radar topic names
  std::string imu_topic_;              // IMU topic name
  std::string base_frame_, odom_frame_;// frames
  Eigen::Affine3f extrinsics_[4];      // per-sensor transforms
  double x_rot_deg_;                   // extra X rotation

  // Filtering parameters
  double voxel_min_, voxel_max_;
  int sor_k_; double sor_std_;
  double rmin_, rmax_, zmin_, zmax_, imin_;

  // ICP parameters
  int icp_iter_; double fit_eps_, trans_eps_, corr_min_, corr_max_, ransac_th_; bool reciprocal_;

  // SLAM/general
  double scale_, kf_trans_, kf_rot_; bool use_ndt_;

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subs_[4];
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_fused_, pub_clean_, pub_aligned_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_, pub_delta_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Cloud buffers
  CloudT sensors_[4];     // raw sensor clouds (latest)
  CloudT fused_;          // fused cloud (base frame)
  CloudT cleaned_;        // filtered cloud (base frame)
  CloudT source_, target_, aligned_; // ICP buffers (source=current, target=keyframe)

  // Pose/IMU
  rclcpp::Time last_stamp_;
  Eigen::Quaternionf imu_q_{1,0,0,0}; // latest orientation from IMU
  Eigen::Vector3f pos_{0,0,0};        // accumulated translation in odom
  Eigen::Matrix4f last_guess_ = Eigen::Matrix4f::Zero(); // ICP initial guess
  Eigen::Quaternionf last_kf_q_{1,0,0,0};                // orientation at keyframe
  Eigen::Vector3f    last_kf_t_{0,0,0};                  // translation at keyframe

  // ------------------- Main processing tick -------------------
  void tick(){
    // Ensure we have fresh data from all sensors; otherwise skip this cycle.
    for (const auto& c : sensors_) if (c.empty()) return;

    // 1) Fuse incoming clouds into base frame.
    fuseClouds();

    // 2) Publish fused for debugging.
    publishCloud(fused_, pub_fused_, base_frame_);

    // 3) Adaptive filtering (VoxelGrid leaf size depends on motion magnitude).
    filterCloudAdaptive(fused_, cleaned_);

    // 4) Publish cleaned cloud.
    publishCloud(cleaned_, pub_clean_, base_frame_);

    // 5) Register against the keyframe target via robust ICP; fallback to NDT if needed.
    bool ok = registerAndIntegrate(cleaned_);

    // 6) If success, maybe promote to keyframe based on motion thresholds.
    if (ok && keyframeNeeded()) {
      target_ = cleaned_;
      last_kf_t_ = pos_;
      last_kf_q_ = imu_q_;
    }

    // 7) Clear sensor buffers to wait for fresh scans.
    for (auto& c : sensors_) c.clear();
  }

  // ------------------- Fusion step -------------------
  void fuseClouds(){
    // Merge all four radar clouds after applying per-sensor extrinsics.
    CloudT merged;
    for (int i=0;i<4;++i){
      CloudT tr;
      pcl::transformPointCloud(sensors_[i], tr, extrinsics_[i]);
      merged += tr;
    }
    // Optional vendor-to-robot alignment (rotate around X by x_rot_deg_).
    float rx = static_cast<float>(x_rot_deg_ * M_PI / 180.0);
    CloudT rotated; pcl::transformPointCloud(merged, rotated, xyzrpy(0,0,0, rx, 0, 0));
    rotated.header.frame_id = base_frame_;
    fused_.swap(rotated);
  }

  // ------------------- Adaptive filtering -------------------
  void filterCloudAdaptive(const CloudT& in, CloudT& out){
    // Estimate a motion magnitude proxy from the last ICP delta guess.
    float motion_proxy = 0.0f;
    if (!last_guess_.isZero()){
      Eigen::Vector3f d(last_guess_(0,3), last_guess_(1,3), last_guess_(2,3));
      motion_proxy = std::min(1.0f, vec3_norm(d)); // clamp to [0,1]
    }
    // Interpolate voxel leaf size between voxel_min_ and voxel_max_ based on motion speed.
    float leaf = static_cast<float>(voxel_min_ + (voxel_max_ - voxel_min_) * motion_proxy);

    // 1) Voxel downsampling (speed + robustness)
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(leaf, leaf, leaf);
    vg.setInputCloud(in.makeShared());
    CloudT ds; vg.filter(ds);

    // 2) Statistical outlier removal (denoise)
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setMeanK(sor_k_);
    sor.setStddevMulThresh(sor_std_);
    sor.setInputCloud(ds.makeShared());
    CloudT sor_out; sor.filter(sor_out);

    // 3) Pass filters (range, height, intensity)
    CloudT pass; pass.header = sor_out.header;
    pass.reserve(sor_out.size());
    for (const auto& p : sor_out.points){
      const double r = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
      if (r < rmin_ || r > rmax_) continue;
      if (p.z < zmin_ || p.z > zmax_) continue;
      if (p.intensity < imin_) continue;
      pass.push_back(p);
    }
    pass.header.frame_id = base_frame_;
    out.swap(pass);
  }

  // ------------------- Registration + integration -------------------
  bool registerAndIntegrate(const CloudT& current){
    // Prepare ICP object with robust settings.
    pcl::IterativeClosestPoint<PointT,PointT> icp;
    icp.setInputSource(current.makeShared());
    if (!target_.empty()) icp.setInputTarget(target_.makeShared());

    // Adaptive max correspondence distance based on previous translation magnitude.
    float d_last = 0.0f;
    if (!last_guess_.isZero()){
      Eigen::Vector3f d(last_guess_(0,3), last_guess_(1,3), last_guess_(2,3));
      d_last = vec3_norm(d);
    }
    double corr = std::clamp(static_cast<double>(d_last * 2.0), corr_min_, corr_max_);
    icp.setMaxCorrespondenceDistance(corr);

    // Standard tuning.
    icp.setMaximumIterations(icp_iter_);
    icp.setEuclideanFitnessEpsilon(fit_eps_);
    icp.setTransformationEpsilon(trans_eps_);
    icp.setUseReciprocalCorrespondences(reciprocal_);
    icp.setRANSACOutlierRejectionThreshold(ransac_th_);

    // Align with or without an initial guess.
    if (last_guess_.isZero()) icp.align(aligned_);
    else                      icp.align(aligned_, last_guess_);

    bool ok = icp.hasConverged();
    double fitness = icp.getFitnessScore();

    // Optional NDT fallback if ICP fails or fitness is poor.
    if ((!ok || fitness > 0.5) && use_ndt_){
      pcl::NormalDistributionsTransform<PointT,PointT> ndt;
      ndt.setResolution(1.0);                        // coarse resolution; adjust via params if needed
      ndt.setStepSize(0.1);                          // optimizer step
      ndt.setTransformationEpsilon(1e-4);            // termination
      ndt.setMaximumIterations(40);
      ndt.setInputSource(current.makeShared());
      if (!target_.empty()) ndt.setInputTarget(target_.makeShared());

      Eigen::Matrix4f guess = last_guess_.isZero() ? Eigen::Matrix4f::Identity() : last_guess_;
      aligned_.clear();
      ndt.align(aligned_, guess);
      ok = ndt.hasConverged();
      if (ok) last_guess_ = ndt.getFinalTransformation();
    } else if (ok){
      last_guess_ = icp.getFinalTransformation();
    }

    // If registration succeeded, integrate translation using IMU for orientation.
    if (ok){
      Eigen::Vector3f dT(last_guess_(0,3), last_guess_(1,3), last_guess_(2,3)); // translation in base frame
      Eigen::Vector3f delta = static_cast<float>(scale_) * (imu_q_ * dT);       // rotate into odom via IMU attitude
      pos_ += delta;                                                            // integrate position

      publishDelta(delta, last_stamp_);
      publishPose(last_stamp_);
      publishAligned(last_stamp_);
    }

    // If there is no target yet, or registration worked, set target to current for next iteration.
    if (ok || target_.empty()) target_ = current;
    return ok;
  }

  // ------------------- Keyframe policy -------------------
  bool keyframeNeeded() const {
    // Trigger new keyframe if translation or yaw since last KF exceeded thresholds.
    Eigen::Vector3f d = pos_ - last_kf_t_;
    double trans = vec3_norm(d);
    // Compute yaw delta between IMU orientation and last keyframe orientation
    double yaw_now = std::atan2(2.0*(imu_q_.w()*imu_q_.z() + imu_q_.x()*imu_q_.y()),
                                1.0 - 2.0*(imu_q_.y()*imu_q_.y() + imu_q_.z()*imu_q_.z()));
    double yaw_kf  = std::atan2(2.0*(last_kf_q_.w()*last_kf_q_.z() + last_kf_q_.x()*last_kf_q_.y()),
                                1.0 - 2.0*(last_kf_q_.y()*last_kf_q_.y() + last_kf_q_.z()*last_kf_q_.z()));
    double dyaw_deg = std::abs((yaw_now - yaw_kf) * 180.0 / M_PI);
    return (trans > kf_trans_) || (dyaw_deg > kf_rot_);
  }

  // ------------------- Publishing helpers -------------------
  void publishCloud(const CloudT& c, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub, const std::string& frame){
    if (!pub) return;
    CloudT tmp = c;                                   // copy to set frame safely
    tmp.header.frame_id = frame;                      // set target frame
    sensor_msgs::msg::PointCloud2 m;
    pcl::toROSMsg(tmp, m);                            // convert to ROS message
    m.header.stamp = last_stamp_;                     // stamp with last cloud time
    m.header.frame_id = frame;                        // set frame
    pub->publish(m);                                  // publish
  }

  void publishAligned(const rclcpp::Time& t){
    // Transform the aligned cloud into the odom frame using current pose/orientation.
    CloudT out;
    pcl::transformPointCloud(aligned_, out, Eigen::Vector3f(pos_.x(), pos_.y(), 0), imu_q_);
    out.header.frame_id = odom_frame_;                // visualization is in odom frame
    sensor_msgs::msg::PointCloud2 m;
    pcl::toROSMsg(out, m);
    m.header.stamp = t; m.header.frame_id = odom_frame_;
    pub_aligned_->publish(m);
  }

  void publishDelta(const Eigen::Vector3f& d, const rclcpp::Time& t){
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    m.header.stamp = t;                               // time stamp
    m.header.frame_id = odom_frame_;                  // odom frame
    m.pose.pose.position.x = d.x();                   // delta translation x
    m.pose.pose.position.y = d.y();                   // delta translation y
    m.pose.pose.position.z = d.z();                   // delta translation z
    m.pose.pose.orientation.x = imu_q_.x();          // IMU attitude x
    m.pose.pose.orientation.y = imu_q_.y();          // IMU attitude y
    m.pose.pose.orientation.z = imu_q_.z();          // IMU attitude z
    m.pose.pose.orientation.w = imu_q_.w();          // IMU attitude w
    for (double& c : m.pose.covariance) c = 0.0;      // reset covariance
    m.pose.covariance[0]=4.0; m.pose.covariance[7]=4.0; m.pose.covariance[14]=4.0; // translational variance
    m.pose.covariance[21]=0.02; m.pose.covariance[28]=0.02; m.pose.covariance[35]=0.02; // rotational variance
    pub_delta_->publish(m);                           // publish delta pose
  }

  void publishPose(const rclcpp::Time& t){
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    m.header.stamp = t;                               // time stamp
    m.header.frame_id = odom_frame_;                  // odom frame
    m.pose.pose.position.x = pos_.x();                // accumulated x
    m.pose.pose.position.y = pos_.y();                // accumulated y
    m.pose.pose.position.z = 0.0;                     // planar assumption for display
    m.pose.pose.orientation.x = imu_q_.x();          // IMU orientation x
    m.pose.pose.orientation.y = imu_q_.y();          // IMU orientation y
    m.pose.pose.orientation.z = imu_q_.z();          // IMU orientation z
    m.pose.pose.orientation.w = imu_q_.w();          // IMU orientation w
    for (double& c : m.pose.covariance) c = 0.0;      // reset covariance
    m.pose.covariance[0]=1.0; m.pose.covariance[7]=1.0; m.pose.covariance[14]=1.0; // positional covariance
    m.pose.covariance[21]=0.01; m.pose.covariance[28]=0.01; m.pose.covariance[35]=0.01; // rotational covariance
    pub_pose_->publish(m);                            // publish pose
  }
};

// ------------------- Main entry -------------------
int main(int argc, char** argv){
  rclcpp::init(argc, argv);                                           // initialize ROS 2
  rclcpp::spin(std::make_shared<QuadMmwaveIcpCore>());                // run node
  rclcpp::shutdown();                                                 // shutdown ROS 2
  return 0;                                                           // exit
}
