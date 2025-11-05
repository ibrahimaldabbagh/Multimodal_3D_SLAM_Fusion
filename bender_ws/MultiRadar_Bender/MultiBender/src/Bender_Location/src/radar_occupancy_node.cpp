#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <unordered_map>

#define PCL_NO_PRECOMPILE

struct mmWaveCloudType
{
  PCL_ADD_POINT4D;
  float intensity;
  float velocity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(mmWaveCloudType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, velocity, velocity))

using PointType = mmWaveCloudType;
using PointCloud = pcl::PointCloud<PointType>;

struct PairHash
{
  std::size_t operator()(const std::pair<int, int> &p) const noexcept
  {
    return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
  }
};

class RadarOccupancyMapper : public rclcpp::Node
{
public:
  RadarOccupancyMapper() : Node("radar_occupancy_mapper")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/accu_pcl", rclcpp::SensorDataQoS(),
        std::bind(&RadarOccupancyMapper::pointCloudCallback, this, std::placeholders::_1));

    pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/radar_map", 1);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&RadarOccupancyMapper::publishMap, this));
  }

private:
  const float resolution_ = 0.05f;
  const int limit_ = 10000;

  std::unordered_map<std::pair<int, int>, int8_t, PairHash> grid_;
  int min_x_ = 0, max_x_ = 0, min_y_ = 0, max_y_ = 0;

  void raytrace(int x0, int y0, int x1, int y1)
  {
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (x0 != x1 || y0 != y1)
    {
      auto key = std::make_pair(x0, y0);
      if (grid_.find(key) == grid_.end())
        grid_[key] = 0;

      int e2 = 2 * err;
      if (e2 > -dy)
      {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dx)
      {
        err += dx;
        y0 += sy;
      }
    }
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PointCloud cloud;
    pcl::fromROSMsg(*msg, cloud);

    for (const auto &pt : cloud.points)
    {
      int x_idx = static_cast<int>(std::round(pt.x / resolution_));
      int y_idx = static_cast<int>(std::round(pt.y / resolution_));

      if (std::abs(x_idx) > limit_ || std::abs(y_idx) > limit_)
        continue;

      min_x_ = std::min(min_x_, x_idx);
      max_x_ = std::max(max_x_, x_idx);
      min_y_ = std::min(min_y_, y_idx);
      max_y_ = std::max(max_y_, y_idx);

      raytrace(0, 0, x_idx, y_idx);

      grid_[{x_idx, y_idx}] = 100;
    }
  }

  void publishMap()
  {
    int width = max_x_ - min_x_ + 1;
    int height = max_y_ - min_y_ + 1;

    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "odom";
    map.header.stamp = this->now();

    map.info.resolution = resolution_;
    map.info.width = width;
    map.info.height = height;
    map.info.origin.position.x = min_x_ * resolution_;
    map.info.origin.position.y = min_y_ * resolution_;
    map.info.origin.orientation.w = 1.0;

    map.data.assign(width * height, -1);

    for (const auto &[cell, value] : grid_)
    {
      int x = cell.first - min_x_;
      int y = cell.second - min_y_;
      int idx = y * width + x;
      if (idx >= 0 && idx < static_cast<int>(map.data.size()))
        map.data[idx] = value;
    }

    pub_map_->publish(map);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarOccupancyMapper>());
  rclcpp::shutdown();
  return 0;
}

