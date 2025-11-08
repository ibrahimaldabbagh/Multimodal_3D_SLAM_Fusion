
/**
 * @file quadmmwave_mapping.cpp
 * @brief Mapping + visualization node for 4‑Directional mmWave Radar SLAM.
 *
 *  • Subscribes to aligned cloud in odom frame (/quadmmwave/points/aligned) and pose.
 *  • Builds an accumulated visualization cloud with chunking based on travel distance.
 *  • Maintains and publishes a nav_msgs/Path and a nav_msgs/Odometry.
 *  • Generates a 2D occupancy grid using log‑odds update for probabilistic mapping
 *    (as opposed to binary marking), which is more state‑of‑the‑art for radar mapping.
 *  • Broadcasts map->odom and odom->base_link TFs for tools like RViz.
 */

#define PCL_NO_PRECOMPILE
#include "rclcpp/rclcpp.hpp"                                  // ROS 2 base
#include <sensor_msgs/msg/point_cloud2.hpp>                   // cloud msgs
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp> // pose msgs
#include <geometry_msgs/msg/transform_stamped.hpp>            // TF msg
#include <nav_msgs/msg/path.hpp>                              // path
#include <nav_msgs/msg/odometry.hpp>                          // odom
#include <nav_msgs/msg/occupancy_grid.hpp>                    // grid map
#include <tf2_ros/transform_broadcaster.h>                    // TF broadcaster
#include <tf2/LinearMath/Quaternion.h>                        // quaternion
#include <pcl/point_cloud.h>                                  // PCL cloud
#include <pcl/point_types.h>                                  // PCL types
#include <pcl_conversions/pcl_conversions.h>                  // ROS<->PCL
#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>

// Radar point definition (duplicated intentionally for compilation isolation).
struct RadarXYZIV { PCL_ADD_POINT4D; float intensity; float velocity; EIGEN_MAKE_ALIGNED_OPERATOR_NEW } EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RadarXYZIV, (float,x,x)(float,y,y)(float,z,z)(float,intensity,intensity)(float,velocity,velocity))
using PointT = RadarXYZIV; using CloudT = pcl::PointCloud<PointT>;

// Hash for 2D grid indexing
struct PairHash { size_t operator()(const std::pair<int,int>& p) const noexcept { return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second)<<1);} };

class QuadMmwaveMapping : public rclcpp::Node {
public:
  QuadMmwaveMapping() : rclcpp::Node("quadmmwave_mapping") {
    // ---- Topics & frames ----
    aligned_topic_ = declare_parameter<std::string>("aligned_topic", "/quadmmwave/points/aligned"); // input cloud (odom)
    pose_topic_    = declare_parameter<std::string>("pose_topic",    "/quadmmwave/pose");           // input pose (odom)
    path_topic_    = declare_parameter<std::string>("path_topic",    "/quadmmwave/path");           // output path
    odom_topic_    = declare_parameter<std::string>("odom_topic",    "/quadmmwave/odom");           // output odom
    accum_topic_   = declare_parameter<std::string>("accum_topic",   "/quadmmwave/points/accum");   // output viz cloud
    map_topic_     = declare_parameter<std::string>("map_topic",     "/quadmmwave/map");            // output grid map

    map_frame_  = declare_parameter<std::string>("map_frame",  "map");   // map frame
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");  // odom frame
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link"); // base frame

    // ---- Viz scaling & TF offset ----
    vis_scale_ = declare_parameter<double>("vis_scale", 1.0);      // optional scale for display
    yaw_deg_   = declare_parameter<double>("yaw_deg", 0.0);        // map->odom yaw offset (deg)
    x_off_     = declare_parameter<double>("x_offset", 0.0);       // map->odom translation x
    y_off_     = declare_parameter<double>("y_offset", 0.0);       // map->odom translation y
    z_off_     = declare_parameter<double>("z_offset", 0.0);       // map->odom translation z

    // ---- Chunking for accumulation ----
    chunk_dist_ = declare_parameter<double>("chunk_distance_m", 1.8); // meters; create new chunk after this travel

    // ---- Probabilistic grid params ----
    res_       = declare_parameter<double>("resolution", 0.08);       // meters per cell
    lim_       = declare_parameter<int>("limit_cells", 12000);        // cell bound (half-extent)
    lo_occ_    = declare_parameter<double>("logodds_occ", 2.0);       // log-odds increment for occupied
    lo_free_   = declare_parameter<double>("logodds_free", -0.7);     // log-odds decrement for free
    lo_min_    = declare_parameter<double>("logodds_min", -4.0);      // lower clamp
    lo_max_    = declare_parameter<double>("logodds_max", 4.0);       // upper clamp
    prob_th_   = declare_parameter<double>("prob_threshold", 0.5);    // probability threshold for occupancy

    // ---- IO ----
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sub_aligned_ = create_subscription<sensor_msgs::msg::PointCloud2>(aligned_topic_, 2, std::bind(&QuadMmwaveMapping::onAligned, this, std::placeholders::_1));
    sub_pose_    = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, 5, std::bind(&QuadMmwaveMapping::onPose, this, std::placeholders::_1));
    pub_path_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 1);
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 1);
    pub_accu_ = create_publisher<sensor_msgs::msg::PointCloud2>(accum_topic_, 1);
    pub_map_  = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 1);

    // ---- Timers ----
    timer_tf_  = create_wall_timer(std::chrono::milliseconds(20),  std::bind(&QuadMmwaveMapping::publishTf, this));
    timer_map_ = create_wall_timer(std::chrono::milliseconds(400), std::bind(&QuadMmwaveMapping::publishMap, this));
  }

private:
  // ---- Members ----
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_aligned_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_accu_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_tf_, timer_map_;

  // Params
  std::string aligned_topic_, pose_topic_, path_topic_, odom_topic_, accum_topic_, map_topic_;
  std::string map_frame_, odom_frame_, base_frame_;
  double vis_scale_, yaw_deg_, x_off_, y_off_, z_off_, chunk_dist_;
  double res_, lo_occ_, lo_free_, lo_min_, lo_max_, prob_th_;
  int lim_;

  // State
  geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
  nav_msgs::msg::Path path_;
  CloudT accum_;                              // merged accumulated cloud (odom)
  std::vector<CloudT> chunks_;                // chunked sub-clouds
  geometry_msgs::msg::Point last_chunk_pose_;
  rclcpp::Time last_stamp_;

  // Log-odds grid
  std::unordered_map<std::pair<int,int>, double, PairHash> log_odds_;
  int min_x_=0, max_x_=0, min_y_=0, max_y_=0;

  // ---- Callbacks ----
  void onPose(const geometry_msgs::msg::PoseWithCovarianceStamped& m){
    last_pose_ = m;                                  // store last pose
    last_stamp_ = m.header.stamp;                    // remember last time for publishing

    // Scale position for visualization if desired
    last_pose_.pose.pose.position.x *= vis_scale_;
    last_pose_.pose.pose.position.y *= vis_scale_;
    last_pose_.pose.pose.position.z *= vis_scale_;

    // Extend path
    geometry_msgs::msg::PoseStamped ps;
    ps.header = last_pose_.header; ps.pose = last_pose_.pose.pose;
    path_.poses.push_back(ps);                       // append new pose
    path_.header = last_pose_.header;                // update header
    pub_path_->publish(path_);                       // publish path

    // Mirror odometry
    nav_msgs::msg::Odometry od;
    od.child_frame_id = base_frame_;
    od.header = last_pose_.header; od.header.frame_id = odom_frame_;
    od.pose = last_pose_.pose;
    pub_odom_->publish(od);                          // publish odom
  }

  static CloudT filterForViz(const CloudT& in){
    CloudT out; out.header = in.header;
    out.reserve(in.size());
    for (const auto& p: in){
      // Keep near-ground returns and strong intensities to produce a crisp 2D-like map.
      if (p.z > -1.0f && p.z < 1.0f && p.intensity > 12.0f) out.push_back(p);
    }
    return out;
  }

  void onAligned(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    CloudT c; pcl::fromROSMsg(*msg, c);              // convert ROS -> PCL
    CloudT f = filterForViz(c);                      // prefilter for visualization

    // Chunking: create a new chunk when the robot moved far enough.
    const auto& cur = last_pose_.pose.pose.position;
    if (chunks_.empty() || std::hypot(cur.x - last_chunk_pose_.x, cur.y - last_chunk_pose_.y) > chunk_dist_){
      chunks_.push_back(f);
      last_chunk_pose_ = cur;
    } else {
      chunks_.back() += f;
    }

    // Merge all chunks to produce the current accumulated cloud.
    accum_.clear();
    for (const auto& ch : chunks_) accum_ += ch;

    // Apply visualization scaling.
    for (auto& p : accum_.points){ p.x *= vis_scale_; p.y *= vis_scale_; p.z *= vis_scale_; }
    accum_.header.frame_id = odom_frame_;

    // Publish accumulated cloud.
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(accum_, out);
    out.header = msg->header; out.header.frame_id = odom_frame_;
    pub_accu_->publish(out);

    // Update log-odds map with the latest accumulated cloud.
    updateLogOddsFromCloud(accum_);
  }

  // ---- TF publishing ----
  void publishTf(){
    // odom -> base_link from the last pose
    geometry_msgs::msg::TransformStamped odom_base;
    odom_base.header.stamp = now();
    odom_base.header.frame_id = odom_frame_;
    odom_base.child_frame_id  = base_frame_;
    odom_base.transform.translation.x = last_pose_.pose.pose.position.x;
    odom_base.transform.translation.y = last_pose_.pose.pose.position.y;
    odom_base.transform.translation.z = last_pose_.pose.pose.position.z;
    odom_base.transform.rotation = last_pose_.pose.pose.orientation;

    // map -> odom static offset with yaw
    geometry_msgs::msg::TransformStamped map_odom;
    map_odom.header = odom_base.header;
    map_odom.header.frame_id = map_frame_;
    map_odom.child_frame_id  = odom_frame_;
    double yaw = yaw_deg_ * M_PI / 180.0;
    tf2::Quaternion q; q.setRPY(0,0,yaw);
    map_odom.transform.rotation.x = q.x();
    map_odom.transform.rotation.y = q.y();
    map_odom.transform.rotation.z = q.z();
    map_odom.transform.rotation.w = q.w();
    map_odom.transform.translation.x = x_off_;
    map_odom.transform.translation.y = y_off_;
    map_odom.transform.translation.z = z_off_;

    tf_broadcaster_->sendTransform(odom_base);
    tf_broadcaster_->sendTransform(map_odom);
  }

  // ---- Probabilistic grid mapping (log‑odds) ----
  void updateLogOddsFromCloud(const CloudT& cloud){
    const float res = static_cast<float>(res_);
    // For each point, update an "occupied" cell; along the ray from origin, update "free" cells.
    for (const auto& p : cloud.points){
      int xi = static_cast<int>(std::round(p.x / res));
      int yi = static_cast<int>(std::round(p.y / res));
      if (std::abs(xi)>lim_ || std::abs(yi)>lim_) continue;
      min_x_ = std::min(min_x_, xi); max_x_ = std::max(max_x_, xi);
      min_y_ = std::min(min_y_, yi); max_y_ = std::max(max_y_, yi);

      // Bresenham ray from (0,0) to (xi,yi) to mark freespace
      int x0=0, y0=0, x1=xi, y1=yi;
      int dx = std::abs(x1-x0), dy = std::abs(y1-y0);
      int sx = (x0<x1) ? 1 : -1;
      int sy = (y0<y1) ? 1 : -1;
      int err = dx - dy;
      while (x0 != x1 || y0 != y1){
        auto k = std::make_pair(x0,y0);
        double lo = 0.0;
        auto it = log_odds_.find(k);
        if (it != log_odds_.end()) lo = it->second;
        lo = std::clamp(lo + lo_free_, lo_min_, lo_max_);
        log_odds_[k] = lo;
        int e2 = 2*err;
        if (e2 > -dy){ err -= dy; x0 += sx; }
        if (e2 <  dx){ err += dx; y0 += sy; }
      }
      // Endpoint occupied update
      auto kend = std::make_pair(xi,yi);
      double loe = 0.0;
      auto it2 = log_odds_.find(kend);
      if (it2 != log_odds_.end()) loe = it2->second;
      loe = std::clamp(loe + lo_occ_, lo_min_, lo_max_);
      log_odds_[kend] = loe;
    }
  }

  // ---- Publish occupancy grid ----
  void publishMap(){
    int width  = max_x_ - min_x_ + 1;
    int height = max_y_ - min_y_ + 1;
    if (width <= 0 || height <= 0) return;

    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = odom_frame_;
    map.header.stamp = now();
    map.info.resolution = res_;
    map.info.width  = width;
    map.info.height = height;
    map.info.origin.position.x = min_x_ * res_;
    map.info.origin.position.y = min_y_ * res_;
    map.info.origin.orientation.w = 1.0;

    // Convert log-odds to probability occupancy values [0..100], unknown = -1
    map.data.assign(width*height, -1);
    for (int y=min_y_; y<=max_y_; ++y){
      for (int x=min_x_; x<=max_x_; ++x){
        auto it = log_odds_.find({x,y});
        if (it == log_odds_.end()) continue;
        double lo = it->second;
        double p = 1.0 - 1.0 / (1.0 + std::exp(lo)); // logistic
        int8_t v = (p >= prob_th_) ? 100 : 0;        // occupied or free
        int xi = x - min_x_;
        int yi = y - min_y_;
        int idx = yi * width + xi;
        if (idx>=0 && idx<(int)map.data.size()) map.data[idx] = v;
      }
    }
    pub_map_->publish(map);
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadMmwaveMapping>());
  rclcpp::shutdown();
  return 0;
}
