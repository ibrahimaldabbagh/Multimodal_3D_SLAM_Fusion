#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "mv_msg/msg/motion_vector_combined.hpp"
#include "mv_msg/msg/motion_vector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>



class MotionVectorViewer : public rclcpp::Node
{
public:
    MotionVectorViewer()
    : Node("mv_viewer")
    {
        mv_1_subscriber_ = this->create_subscription<mv_msg::msg::MotionVectorCombined>(
            "mvs/dev/video2/motion_vector", 1, std::bind(&MotionVectorViewer::mv_1_callback, this, std::placeholders::_1));

        mv_2_subscriber_ = this->create_subscription<mv_msg::msg::MotionVectorCombined>(
            "mvs/dev/video6/motion_vector", 1, std::bind(&MotionVectorViewer::mv_2_callback, this, std::placeholders::_1));
            

        cam_1_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "mvs/dev/video2/image", 1, std::bind(&MotionVectorViewer::cam_1_callback, this, std::placeholders::_1));

        cam_2_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "mvs/dev/video6/image", 1, std::bind(&MotionVectorViewer::cam_2_callback, this, std::placeholders::_1));

        cam_1_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("mvs/dev/video2/image_mv_viewer", 1);

        cam_2_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("mvs/dev/video6/image_mv_viewer", 1);
        
        std::cout << "Motion Vector Viewer is running."<< std::endl;
    }

private:

    rclcpp::Subscription<mv_msg::msg::MotionVectorCombined>::SharedPtr mv_1_subscriber_;
    rclcpp::Subscription<mv_msg::msg::MotionVectorCombined>::SharedPtr mv_2_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_1_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_2_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_1_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_2_publisher_;
    cv::Mat frame_1;
    cv::Mat frame_2;

    void cam_1_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        frame_1 = cv_bridge::toCvCopy(msg, "bgr8")->image;
        //std::cout << "frame_1 set" <<std::endl;
    }

    void cam_2_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        frame_2 = cv_bridge::toCvCopy(msg, "bgr8")->image;
        //std::cout << "frame_2 set" <<std::endl;
    }

    void mv_1_callback(const mv_msg::msg::MotionVectorCombined::SharedPtr msg)
    {
        draw_motion_vectors(msg->mv, &frame_1);
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_1).toImageMsg();
        cam_1_publisher_->publish(*image_msg);
    }

    void mv_2_callback(const mv_msg::msg::MotionVectorCombined::SharedPtr msg)
    {
        draw_motion_vectors(msg->mv, &frame_2);
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_2).toImageMsg();
        cam_2_publisher_->publish(*image_msg);
    }


    void draw_motion_vectors(const std::vector<mv_msg::msg::MotionVector>& mv, cv::Mat* frame)
    {
        int n = mv.size();
        for (int i = 0; i < n; i++) {
            cv::arrowedLine(*frame, cv::Point2i(mv[i].src_x, mv[i].src_y), cv::Point2i(mv[i].dst_x, mv[i].dst_y), cv::Scalar(0, 0, 255), 1, cv::LINE_AA, 0, 0.1);
        }
    }


};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionVectorViewer>());
    rclcpp::shutdown();
    return 0;
}
