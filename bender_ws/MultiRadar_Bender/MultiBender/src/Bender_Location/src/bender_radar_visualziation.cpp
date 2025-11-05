#define PCL_NO_PRECOMPILE
#include "rclcpp/rclcpp.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/registration/icp.h>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>


struct mmWaveCloudType
{
    PCL_ADD_POINT4D;
    union
    {
       struct  {
	    float intensity;
	    float velocity;
	};
        float data_c[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(mmWaveCloudType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, velocity, velocity))

typedef mmWaveCloudType PointType;
typedef pcl::PointCloud<mmWaveCloudType> PointCloud;
using std::placeholders::_1;

geometry_msgs::msg::TransformStamped tf_odom_base_link, tf_map_odom, tf_map_camera_odom_frame;
std::shared_ptr<sensor_msgs::msg::PointCloud2> pc2_msg_;

rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pub_path_radar;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_odom;

bool first_t265_msg = true;

double path_scaling_multiplier = 1.0, path_yaw_rotation = 0.0, path_x_offset = 0.0, path_y_offset = -0.0, path_z_offset = 0;
double previous_path_scaling_multiplier = 1;

std::vector<PointCloud> cloud_chunks;
geometry_msgs::msg::Point last_chunk_pose;

geometry_msgs::msg::PoseWithCovarianceStamped pose_estimate_radar;

nav_msgs::msg::Path path_radar;
nav_msgs::msg::Odometry odom;

int path_mov_avg_window_size = 10;

PointCloud scalePointCloud(PointCloud pcl_in, double scaling_mulitplier)
{
    for (size_t i = 0; i < pcl_in.points.size(); i++)

    {
        pcl_in.points[i].x *= scaling_mulitplier;
        pcl_in.points[i].y *= scaling_mulitplier;
        pcl_in.points[i].z *= scaling_mulitplier;
    }
    return pcl_in;
}

void addToAccumulatedPointCloud(const PointCloud& current_filtered, const geometry_msgs::msg::Point& current_pos)
{
    if (cloud_chunks.empty() ||
        std::hypot(current_pos.x - last_chunk_pose.x, current_pos.y - last_chunk_pose.y) > 2.0)
    {
        cloud_chunks.push_back(current_filtered);
        last_chunk_pose = current_pos;
    }
    else
    {
        cloud_chunks.back() += current_filtered;
    }

    // Set frame for latest
    cloud_chunks.back().header.frame_id = "odom";
}

PointCloud filterACPC(const PointCloud& pc)
{
    PointCloud pc_filtered;
    pc_filtered.header = pc.header;

    auto filter_evaliation = [](PointType pt)
    {
        return pt.z < 1 && pt.z > -1;
    };

    std::copy_if(pc.begin(), pc.end(), std::back_inserter(pc_filtered), filter_evaliation);

    PointCloud pc_final;
    pc_final.header = pc.header;

    auto filter_intensity = [](PointType pt)
    {
        return pt.intensity > 15;
    };

    std::copy_if(pc_filtered.begin(), pc_filtered.end(), std::back_inserter(pc_final), filter_intensity);

    return pc_final;
}


void updatePathRadar()
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = pose_estimate_radar.header;
    pose_stamped.pose = pose_estimate_radar.pose.pose;

    path_radar.poses.push_back(pose_stamped);

    path_radar.header = pose_estimate_radar.header;
    _pub_path_radar->publish(path_radar);
}

void publishOdometry()
{
    odom.child_frame_id = "base_link";
    odom.header = pose_estimate_radar.header;
    odom.header.frame_id = "odom";

    odom.pose = pose_estimate_radar.pose;

    _pub_odom->publish(odom);
}

class BenderRadarVisu : public rclcpp::Node
{

public:
    BenderRadarVisu() : Node("bender_radar_visu")
    {
        tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        _sub_aligned_pcl = create_subscription<sensor_msgs::msg::PointCloud2>("/aligned_pcl", 1, std::bind(&BenderRadarVisu::aligned_pcl_callback, this, _1));
        _sub_imu = create_subscription<sensor_msgs::msg::Imu>("/imu/data_madg", 1, std::bind(&BenderRadarVisu::imu_callback, this, _1));
        _sub_pose_estimate = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_estimation", 5, std::bind(&BenderRadarVisu::pose_estimate_callback, this, _1));
        _pub_path_radar = create_publisher<nav_msgs::msg::Path>("path/radar", 1);
        pubTimeTransform = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&BenderRadarVisu::publishTransform, this));
        _pub_accu_pcl = create_publisher<sensor_msgs::msg::PointCloud2>("accu_pcl", 1);
        _pub_odom = create_publisher<nav_msgs::msg::Odometry>("radar_odm", 1);
    }

private:
    void publishTransform()
    {
        tf_odom_base_link.header.stamp = this->now();
        tf_map_odom.header.stamp = this->now();
        tf_map_odom.header.frame_id = "map";
        tf_map_odom.child_frame_id = "odom";
        tf_odom_base_link.header.frame_id = "odom";
        tf_odom_base_link.child_frame_id = "base_link";
        tf2_broadcaster_->sendTransform(tf_odom_base_link);
        tf2_broadcaster_->sendTransform(tf_map_odom);
    }

    void pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &pose_msg)
    {
        pose_estimate_radar = pose_msg;

        pose_estimate_radar.pose.pose.position.x *= path_scaling_multiplier;
        pose_estimate_radar.pose.pose.position.y *= path_scaling_multiplier;
        pose_estimate_radar.pose.pose.position.z *= path_scaling_multiplier;

        updatePathRadar();

        tf_odom_base_link.transform.translation.x = pose_estimate_radar.pose.pose.position.x;
        tf_odom_base_link.transform.translation.y = pose_estimate_radar.pose.pose.position.y;
        tf_odom_base_link.transform.translation.z = pose_estimate_radar.pose.pose.position.z;

        tf2::Quaternion rotation;
        rotation.setX(pose_estimate_radar.pose.pose.orientation.x);
        rotation.setY(pose_estimate_radar.pose.pose.orientation.y);
        rotation.setZ(pose_estimate_radar.pose.pose.orientation.z);
        rotation.setW(pose_estimate_radar.pose.pose.orientation.w);

        tf_odom_base_link.transform.rotation.x = rotation.x();
        tf_odom_base_link.transform.rotation.y = rotation.y();
        tf_odom_base_link.transform.rotation.z = rotation.z();
        tf_odom_base_link.transform.rotation.w = rotation.w();

        tf_map_odom.transform.translation.x = path_x_offset;
        tf_map_odom.transform.translation.y = path_y_offset;
        tf_map_odom.transform.translation.z = path_z_offset;

        path_yaw_rotation = path_yaw_rotation / 180 * 3.141596;
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, path_yaw_rotation);

        tf_map_odom.transform.rotation.x = orientation.x();
        tf_map_odom.transform.rotation.y = orientation.y();
        tf_map_odom.transform.rotation.z = orientation.z();
        tf_map_odom.transform.rotation.w = orientation.w();

        publishOdometry();
    }

	void aligned_pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
	{
	    PointCloud aligned_pcl;
	    pcl::fromROSMsg(*input, aligned_pcl);

	    // Apply filters
	    PointCloud filtered = filterACPC(aligned_pcl);

	    // Save pose
	    geometry_msgs::msg::Point cur_pos = pose_estimate_radar.pose.pose.position;

	    // Store filtered data
	    addToAccumulatedPointCloud(filtered, cur_pos);

	    // Merge chunks into one visualization
	    PointCloud merged;
	    for (const auto& chunk : cloud_chunks)
	    {
		merged += chunk;
	    }

	    // Scale for display
	    PointCloud scaled = scalePointCloud(merged, path_scaling_multiplier);

	    // Publish
	    pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
	    pcl::toROSMsg(scaled, *pc2_msg_);
	    pc2_msg_->header = input->header;
	    pc2_msg_->header.frame_id = "odom";
	    _pub_accu_pcl->publish(*pc2_msg_);
	}



    void imu_callback(const sensor_msgs::msg::Imu &imu_msg)
    {
        previousTimestamp = imu_msg.header.stamp;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_aligned_pcl;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _sub_pose_estimate;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_imu;
    rclcpp::Time previousTimestamp;
    rclcpp::TimerBase::SharedPtr pubTimeTransform;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_accu_pcl;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BenderRadarVisu>());
    rclcpp::shutdown();
    return 0;
}

