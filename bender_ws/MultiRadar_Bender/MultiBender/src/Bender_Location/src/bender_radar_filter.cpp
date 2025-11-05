#define PCL_NO_PRECOMPILE

#include "rclcpp/rclcpp.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/voxel_grid.h>

using std::placeholders::_1;

struct mmWaveCloudType
{
    PCL_ADD_POINT4D;
    union
    {
        struct
        {
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

double filter_radius_min_thresh = 0.25, filter_radius_max_thresh = 10;
double filter_height_max_thresh = 10, filter_height_min_thresh = -3;

double filter_intensity_min_thresh = 6;
std::shared_ptr<sensor_msgs::msg::PointCloud2> pc2_msg_;

pcl::PointCloud<PointType>::Ptr filtered_pcl_ptr(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType> &filtered_pcl = *filtered_pcl_ptr;

void filter_outliers(pcl::PointCloud<PointType>::Ptr pc)
{
    pcl::StatisticalOutlierRemoval<PointType> sort;
    sort.setInputCloud(pc);
    sort.setMeanK(50);
    sort.setStddevMulThresh(1.3);
    sort.filter(*pc);
}

void filter_radius_outlier(pcl::PointCloud<PointType>::Ptr pc)
{
    pcl::RadiusOutlierRemoval<PointType> rad;
    rad.setInputCloud(pc);
    rad.setRadiusSearch(0.5);
    rad.setMinNeighborsInRadius(7);
    rad.filter(*pc);
}

void filter_by_intensity(PointCloud &pc)
{
    PointCloud pc_filtered;

    pc_filtered.header = pc.header;

    auto filter = [](PointType pt)
    {
        if (pt.intensity > filter_intensity_min_thresh)
            return true;
        else
            return false;
    };
    std::copy_if(pc.begin(), pc.end(), std::back_inserter(pc_filtered), filter);
    pc = pc_filtered;
}

void filter_radius(PointCloud &pc)
{
    PointCloud pc_filtered;

    pc_filtered.header = pc.header;

    auto filter = [](PointType pt)
    {
        double radial_distance = sqrt(pow(pt.x, 2) + pow(pt.y, 2) + pow(pt.z, 2)); // over = positive, below = negative

        bool is_within_max_radius = radial_distance > filter_radius_min_thresh && radial_distance < filter_radius_max_thresh;

        return is_within_max_radius;
    };

    std::copy_if(pc.begin(), pc.end(), std::back_inserter(pc_filtered), filter);
    pc = pc_filtered;
}

void filter_elevation(PointCloud &pc)
{
    PointCloud pc_filtered;
    pc_filtered.header = pc.header;

    auto filter = [](PointType pt)
    {
        double elevation = pt.z;
        bool is_in_elevation_range = elevation < filter_height_max_thresh && elevation > filter_height_min_thresh;

        return is_in_elevation_range;
    };
    std::copy_if(pc.begin(), pc.end(), std::back_inserter(pc_filtered), filter);
    pc = pc_filtered;
}

class BenderRadarFilter : public rclcpp::Node
{
public:
    BenderRadarFilter() : Node("bender_radar_filter")
    {
        sub_pcl_combined = create_subscription<sensor_msgs::msg::PointCloud2>("combined_pcl", 5, std::bind(&BenderRadarFilter::combined_callback, this, _1));
        pub_filtered_pcl = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pcl", 5);
    }

private:
    void combined_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {

        pcl::fromROSMsg(*input, *filtered_pcl_ptr);
        filter_outliers(filtered_pcl_ptr);
        // filter_radius_outlier(filtered_pcl_ptr);
        filter_elevation(filtered_pcl);
        filter_radius(filtered_pcl);
        // filter_by_intensity(filtered_pcl);
        pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(filtered_pcl, *pc2_msg_);

        pub_filtered_pcl->publish(*pc2_msg_);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_combined;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_pcl;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BenderRadarFilter>());
    rclcpp::shutdown();
    return 0;
}
