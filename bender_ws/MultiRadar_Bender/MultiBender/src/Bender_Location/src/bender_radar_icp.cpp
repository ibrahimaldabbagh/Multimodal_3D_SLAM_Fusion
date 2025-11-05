#define PCL_NO_PRECOMPILE
using namespace std;
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
#include <string>
#include <fstream>
#include <json/json.h>
#include <pcl/registration/gicp.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <geometry_msgs/msg/quaternion.hpp>

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
PointCloud aligned_pcl;
static int counter = 0;
std::string reasons, reason;
static int counter_f = 0;
std::shared_ptr<sensor_msgs::msg::PointCloud2>
    pc2_msg_;
pcl::IterativeClosestPoint<PointType, PointType> icp;
pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
pcl::PointCloud<PointType>::Ptr source_pcl_ptr(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType> &source_pcl = *source_pcl_ptr;
pcl::PointCloud<PointType>::Ptr target_pcl_ptr(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType> &target_pcl = *target_pcl_ptr;
pcl::PointCloud<PointType>::Ptr jsontarget_pcl(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType> &jsontarget = *jsontarget_pcl;
pcl::PointCloud<PointType>::Ptr jsonsource_pcl(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType> &jsonsource = *jsonsource_pcl;
Eigen::Vector3f position_estimate_vector,
    translation_estimate_vector = Eigen::Vector3f(0, 0, 0);
//geometry_msgs::msg::PoseStamped pose_estimate;
//geometry_msgs::msg::PoseStamped translation_estimate;
geometry_msgs::msg::PoseWithCovarianceStamped pose_estimate;
geometry_msgs::msg::PoseWithCovarianceStamped translation_estimate;
Eigen::Matrix4f icp_transformation, gicp_transformation;
sensor_msgs::msg::Imu last_imu_msg;
static int counter_c = 0;
inline double fitscore = 0.0;
static bool first = true;
tf2::Quaternion imu_quat;
tf2::Quaternion result_quat;
Eigen::Quaternionf imu_eigen_quat;
Eigen::Quaternionf imu_eigen_quat_last;
Eigen::Quaternionf imu_eigen_quat_rel;
//rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_estimate_pose;
//rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_translate_pose;
rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_estimate_pose;
rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_translate_pose;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aligned_pcl;

void ICPMatching() // Funktion zur Durchführung von ICP
{
    if (!target_pcl.empty())
    {

        icp.setInputSource(source_pcl_ptr);
        icp.setInputTarget(target_pcl_ptr);
        icp.setMaximumIterations(300);         // 300 /* Einstellbare Paramater können hier geändert werden Iteration TransformationEpsilon für die Genauigkeit usw
        icp.setMaxCorrespondenceDistance(0.2); // 0.2
        icp.setTransformationEpsilon(1e-10);
        icp.setEuclideanFitnessEpsilon(1e-5);
        icp.setUseReciprocalCorrespondences(true); // Leichte Glättung
        if (icp_transformation.isZero())
        {
            icp.align(aligned_pcl);
            std::cout << "Die Matrix ist leer." << std::endl; // Hier wird geschaut ob es die erste Transformation ist
        } // wenn ja dann wird nicht die alte Transformation mitgegeben
        else
        {
            icp.align(aligned_pcl, icp_transformation); // Danach wird immer die vorherige Transformation mitgeben als vorverarbeitung
        }

        switch (icp.getConvergeCriteria()->getConvergenceState())
        {
        case 1:
            reason = "Not Converged";
            counter_c++;
            std::cout << counter_c << endl;

            break;

        case 2:
            reason = "Max Num of Iterations";
            break;
        case 3:
            reason = "Epsilon Threshhold";
            break;
        case 4:
            reason = "EuclidianDistance";
            break;
        }
        if (reason != "Not Converged") // Neue Transformation wird nur genommen wennn der Algorithmus
        {                              // mit einem der oberen Kriterien Konvergiert wenn nicht dann wird die Zielpunktwolke behalten und nochmal genutzt
            icp_transformation = icp.getFinalTransformation();
        }
        // Wenn nicht konvergiert nicht nutzen sondern nächste wolke und dann nicht publishen
        std::cout << reason << endl;

        fitscore = icp.getFitnessScore();
        std::cout << fitscore << std::endl;
    }

    if (reason != "Not Converged" || target_pcl.empty())
    {

        std::cout << "New Target" << endl;
        target_pcl = source_pcl;
    }
}

void publishTranslationEstimation() // Die einzelne Pose wird gepublisht
{
    translation_estimate.header.frame_id = "odom";
    translation_estimate.header.stamp = pcl_conversions::fromPCL(aligned_pcl.header.stamp);

    translation_estimate.pose.pose.position.x = translation_estimate_vector.x();
    translation_estimate.pose.pose.position.y = translation_estimate_vector.y();
    translation_estimate.pose.pose.position.z = translation_estimate_vector.z();

    translation_estimate.pose.pose.orientation.x = double(imu_eigen_quat.x());
    translation_estimate.pose.pose.orientation.y = double(imu_eigen_quat.y());
    translation_estimate.pose.pose.orientation.z = double(imu_eigen_quat.z());
    translation_estimate.pose.pose.orientation.w = double(imu_eigen_quat.w());

    for (int i = 0; i < 36; ++i) translation_estimate.pose.covariance[i] = 0.0;
    translation_estimate.pose.covariance[0] = 9.0;
    translation_estimate.pose.covariance[7] = 9.0;
    translation_estimate.pose.covariance[14] = 9.0;
    translation_estimate.pose.covariance[21] = 0.03;
    translation_estimate.pose.covariance[28] = 0.03;
    translation_estimate.pose.covariance[35] = 0.03;

    pub_translate_pose->publish(translation_estimate);

}

void PublishPoseEstimation()
{
    pose_estimate.header.frame_id = "odom";
    pose_estimate.header.stamp = pcl_conversions::fromPCL(aligned_pcl.header.stamp);

    pose_estimate.pose.pose.position.x = position_estimate_vector.x();
    pose_estimate.pose.pose.position.y = position_estimate_vector.y();
    pose_estimate.pose.pose.position.z = 0.0;

    pose_estimate.pose.pose.orientation.x = double(imu_eigen_quat.x());
    pose_estimate.pose.pose.orientation.y = double(imu_eigen_quat.y());
    pose_estimate.pose.pose.orientation.z = double(imu_eigen_quat.z());
    pose_estimate.pose.pose.orientation.w = double(imu_eigen_quat.w());

    for (int i = 0; i < 36; ++i) {
        pose_estimate.pose.covariance[i] = 0.0;
    }

    // Check ICP convergence and fitness score
    bool icp_good = (!source_pcl.empty() && icp.hasConverged());
    double fit = icp_good ? icp.getFitnessScore() : 1e6;
    double scale = (icp_good && fit < 0.5) ? 1.0 : 2.0; // Increase covariance if ICP bad

    pose_estimate.pose.covariance[0] = 1.0 * scale;
    pose_estimate.pose.covariance[7] = 1.0 * scale;
    pose_estimate.pose.covariance[14] = 1.0 * scale;
    pose_estimate.pose.covariance[21] = 0.03 * scale;
    pose_estimate.pose.covariance[28] = 0.03 * scale;
    pose_estimate.pose.covariance[35] = 0.03 * scale;

    pub_estimate_pose->publish(pose_estimate);
}


void publish_alignded_pcl() // Registrierte Punktwolke wird Rotiert und dann veröffentlicht in ROS2 als Topic
{

    pcl::transformPointCloud(aligned_pcl, aligned_pcl, Eigen::Vector3f(position_estimate_vector.x(), position_estimate_vector.y(), 0), imu_eigen_quat);
    aligned_pcl.header.frame_id = "odom";
    pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(aligned_pcl, *pc2_msg_);
    pub_aligned_pcl->publish(*pc2_msg_);
}

void reset()
{
    position_estimate_vector = Eigen::Vector3f(0, 0, 0);
    target_pcl.clear();
}

class Bender_Radar_ICP : public rclcpp::Node
{

public:
    Bender_Radar_ICP() : Node("bender_radar_icp")
    {
        sub_filter_pcl = create_subscription<sensor_msgs::msg::PointCloud2>("/filtered_pcl", 5, std::bind(&Bender_Radar_ICP::filtered_pcl_callback, this, _1));
        sub_imu_data = create_subscription<sensor_msgs::msg::Imu>("/imu/data", 5, std::bind(&Bender_Radar_ICP::imu_callback, this, _1));
        //pub_estimate_pose = create_publisher<geometry_msgs::msg::PoseStamped>("pose_estimation", 5);
        //pub_translate_pose = create_publisher<geometry_msgs::msg::PoseStamped>("translation_estimate", 5);
        pub_estimate_pose = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_estimation", 5);
        pub_translate_pose = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("translation_estimate", 5);
        pub_aligned_pcl = create_publisher<sensor_msgs::msg::PointCloud2>("aligned_pcl", 5);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu imu_msg)
    {

        if ((imu_msg.header.stamp.sec - last_imu_msg.header.stamp.sec) < 0)
        {
        }
        imu_eigen_quat = Eigen::Quaternionf(imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z);
        imu_eigen_quat.normalized();
        last_imu_msg = imu_msg;
    }

    void filtered_pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        pcl::fromROSMsg(*input, source_pcl);
        // if ((counter++ % 2 == 0) || target_pcl.empty())
        //{
        ICPMatching();

        if (reasons != "Not Converged") // NAch Transformation wird die translation nur genutzt wenn es Konvergiert ist sonst wird die Pose nicht veröffentlicht
        {

          //  translation_estimate_vector = imu_eigen_quat * Eigen::Vector3f(icp_transformation(0, 3), icp_transformation(1, 3), icp_transformation(2, 3)); // Globale Transformierung durch IMU
	    float scale_correction = 1.16;  // ~30% boost
	    translation_estimate_vector = scale_correction * (imu_eigen_quat * Eigen::Vector3f(
    	    icp_transformation(0, 3),
    	    icp_transformation(1, 3),
    	    icp_transformation(2, 3)));
            position_estimate_vector += translation_estimate_vector;

            publish_alignded_pcl();
            PublishPoseEstimation();
            publishTranslationEstimation();
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_filter_pcl;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_data;
};

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bender_Radar_ICP>());
    rclcpp::shutdown();
    return 0;
}
