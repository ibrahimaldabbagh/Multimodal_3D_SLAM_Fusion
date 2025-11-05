#include "rclcpp/rclcpp.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <boost/shared_ptr.hpp>
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

Eigen::Affine3f tf2_pcl_middle_0, tf2_pcl_left_2, tf2_pcl_1_right, tf2_pcl_3_up, tf2_zaxis;
PointCloud radar_pcl_0_middle, radar_pcl_1_left, radar_pcl_2_right, radar_pcl_3_up, pcl_combined_currently, pcl_rotate;
std::shared_ptr<sensor_msgs::msg::PointCloud2> pc2_msg_;

Eigen::Affine3f CoordTransformer(float x, float y, float z, float alpha, float beta, float theta)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    transform.translation() << x, y, z;
    transform.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    return transform;
}

PointCloud TransfromPointCloud(PointCloud input, Eigen::Affine3f transform)
{
    PointCloud transformedPointCloud;
    pcl::transformPointCloud<PointType>(input, transformedPointCloud, transform);
    return transformedPointCloud;
}

class BenderRadarProcessing : public rclcpp::Node
{
public:
    BenderRadarProcessing() : Node("bender_radar_processing")
    {

        sub_pcl_0_middle = create_subscription<sensor_msgs::msg::PointCloud2>("/ti_mmwave/radar_scan_pcl_1", 5, std::bind(&BenderRadarProcessing::pcl_mid_callback, this, std::placeholders::_1));
        sub_pcl_2_left = create_subscription<sensor_msgs::msg::PointCloud2>("/ti_mmwave/radar_scan_pcl_0", 5, std::bind(&BenderRadarProcessing::pcl_left_callback, this, std::placeholders::_1));
        sub_pcl_1_right = create_subscription<sensor_msgs::msg::PointCloud2>("/ti_mmwave/radar_scan_pcl_2", 5, std::bind(&BenderRadarProcessing::pcl_right_callback, this, std::placeholders::_1));
        sub_pcl_3_upper = create_subscription<sensor_msgs::msg::PointCloud2>("/ti_mmwave/radar_scan_pcl_3", 5, std::bind(&BenderRadarProcessing::pcl_upper_callback, this, std::placeholders::_1));
        combined_pcl_pub = create_publisher<sensor_msgs::msg::PointCloud2>("combined_pcl", 5);
        _timer = this->create_wall_timer(std::chrono::milliseconds(70), std::bind(&BenderRadarProcessing::timer_callback, this)); // Timer Callback wird alle 70ms aufgerufen
    }

private: // Rotierung der einzelnen Sensoren also 4 mal
    void pcl_mid_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        tf2_pcl_middle_0 = CoordTransformer(0, 0, 0, 0, 0, 0);
        boost::shared_ptr<pcl::PointCloud<mmWaveCloudType>> RScan(new pcl::PointCloud<mmWaveCloudType>);
        PointCloud TransPointCloud_0;

        pcl::fromROSMsg(*input, *RScan);

        radar_pcl_0_middle = TransfromPointCloud(*RScan, tf2_pcl_middle_0);
    }

    void pcl_left_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        tf2_pcl_left_2 = CoordTransformer(-0.029, 0, 0.072, 0, -0.785398, 0);
        PointCloud TransPointCloud_1;
        boost::shared_ptr<pcl::PointCloud<mmWaveCloudType>> RScan2(new pcl::PointCloud<mmWaveCloudType>);

        pcl::fromROSMsg(*input, *RScan2);

        radar_pcl_1_left = TransfromPointCloud(*RScan2, tf2_pcl_left_2);
    }

    void pcl_right_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        tf2_pcl_1_right = CoordTransformer(0.029, 0, -0.072, 0, 0.785398, 0);
        PointCloud TransPointCloud_2;
        boost::shared_ptr<pcl::PointCloud<mmWaveCloudType>> RScan3(new pcl::PointCloud<mmWaveCloudType>);

        pcl::fromROSMsg(*input, *RScan3);

        radar_pcl_2_right = TransfromPointCloud(*RScan3, tf2_pcl_1_right);
    }

    void pcl_upper_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        PointCloud TransPointCloud_3;
        boost::shared_ptr<pcl::PointCloud<mmWaveCloudType>> RScan4(new pcl::PointCloud<mmWaveCloudType>);

        tf2_pcl_3_up = CoordTransformer(-0.021, 0.07, 0, 0, 0, 0.785398);

        pcl::fromROSMsg(*input, *RScan4);
        radar_pcl_3_up = TransfromPointCloud(*RScan4, tf2_pcl_3_up);
    }
    void timer_callback() // Aufrauf Timer Callback schaut alle 70ms in rein und schaut dann ob alle 4 Sensoren einen Wert liefern wenn ja kombiniert er sie wenn nicht kombiniert er keinen Punktwolken
    {
        if (!radar_pcl_0_middle.empty() && !radar_pcl_1_left.empty() && !radar_pcl_2_right.empty() && !radar_pcl_3_up.empty())
        {
            tf2_zaxis = CoordTransformer(0, 0, 0, 1.5708, 0, 0);
            radar_pcl_0_middle += radar_pcl_2_right;
            radar_pcl_0_middle += radar_pcl_1_left;
            radar_pcl_0_middle += radar_pcl_3_up;
            pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
            pcl_combined_currently = radar_pcl_0_middle;
            pcl_combined_currently.header.frame_id = "base_link";
            pc2_msg_->header.stamp = this->now();
            pc2_msg_->header.frame_id = "base_link";
            pcl_rotate = TransfromPointCloud(pcl_combined_currently, tf2_zaxis);
            pcl::toROSMsg(pcl_rotate, *pc2_msg_);
            pc2_msg_->header.frame_id = "base_link";
            combined_pcl_pub->publish(*pc2_msg_);
            radar_pcl_0_middle.clear(), radar_pcl_1_left.clear(), radar_pcl_2_right.clear(), radar_pcl_3_up.clear(), pcl_combined_currently.clear();
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_0_middle;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_1_right;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_2_left;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_3_upper;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pcl_pub;
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BenderRadarProcessing>());
    rclcpp::shutdown();
    return 0;
}
