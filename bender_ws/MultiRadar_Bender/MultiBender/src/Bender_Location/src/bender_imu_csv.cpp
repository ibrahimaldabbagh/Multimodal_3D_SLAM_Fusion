#include "rclcpp/rclcpp.hpp"
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fstream>

using std::placeholders::_1;

std::string combineTimestamp(int32_t sec, std::string nanosec)
{
    return std::to_string(sec) + "." + (nanosec);
}

class BenderCSVIMU : public rclcpp::Node
{

public:
    BenderCSVIMU() : Node("rclcpp")
    {
        _sub_imu_mdg = create_subscription<sensor_msgs::msg::Imu>("/imu/data_madg", 5, std::bind(&BenderCSVIMU::madgwick_callback, this, _1));

        _sub_imu_data = create_subscription<sensor_msgs::msg::Imu>("/imu/datad", 5, std::bind(&BenderCSVIMU::data_callback, this, _1));
        _sub_imu_raw = create_subscription<sensor_msgs::msg::Imu>("/imu/data_raw", 5, std::bind(&BenderCSVIMU::raw_callback, this, _1));
        _sub_imu_comp = create_subscription<sensor_msgs::msg::Imu>("/imu/data", 5, std::bind(&BenderCSVIMU::comp_callback, this, _1));

        csv_file.open("/home/jansenol/csv/imu_madg.csv");
        csv_file << "Zeitstempel,Orientation_X,Orientation_Y,Orientation_Z,Orientation_W,Lineare_Beschleunigung_X,Lineare_Beschleunigung_Y,Lineare_Beschleunigung_Z,Winkelgeschwinigkeit_X,Winkelgeschwinigkeit_Y,Winkelgeschwinigkeit_z\n ";

        csv_file_comp.open("/home/jansenol/csv/imu_comp.csv");
        csv_file_comp << "Zeitstempel,Orientation_X,Orientation_Y,Orientation_Z,Orientation_W\n";
        csv_file_data.open("/home/jansenol/csv/imu_data.csv");
        csv_file_data << "Zeitstempel,Lineare_Beschleunigung_X,Lineare_Beschleunigung_Y,Lineare_Beschleunigung_Z,Winkelgeschwinigkeit_X,Winkelgeschwinigkeit_Y,Winkelgeschwinigkeit_z\n ";
        csv_file_raw.open("/home/jansenol/csv/imu_raw.csv");
        csv_file_raw << "Zeitstempel,Lineare_Beschleunigung_X,Lineare_Beschleunigung_Y,Lineare_Beschleunigung_Z,Winkelgeschwinigkeit_X,Winkelgeschwinigkeit_Y,Winkelgeschwinigkeit_z\n ";
    }

private:
    void comp_callback(const sensor_msgs::msg::Imu msg)
    {
        int32_t timestamp_sec = msg.header.stamp.sec;
        uint32_t timestamp_nanosec = msg.header.stamp.nanosec;
        std::ostringstream oss;
        oss << std::setw(9) << std::setfill('0') << timestamp_nanosec;
        std::string result = oss.str();

        std::string timestamp = combineTimestamp(timestamp_sec, result);
        float ori_x = msg.orientation.x;
        float ori_y = msg.orientation.y;
        float ori_z = msg.orientation.z;
        float ori_w = msg.orientation.w;

        csv_file_comp << timestamp << "," << ori_x << "," << ori_y << "," << ori_z << "," << ori_w << /*"," << acc_x << "," <<  acc_y<< "," <<  acc_z<< "," << ang_x<< "," << ang_y<< "," << ang_z << */ "\n";
    }

    void madgwick_callback(const sensor_msgs::msg::Imu msg)
    {
        int32_t timestamp_sec = msg.header.stamp.sec;
        uint32_t timestamp_nanosec = msg.header.stamp.nanosec;
        std::ostringstream oss;
        oss << std::setw(9) << std::setfill('0') << timestamp_nanosec;
        std::string result = oss.str();

        std::string timestamp = combineTimestamp(timestamp_sec, result);
        float ori_x = msg.orientation.x;
        float ori_y = msg.orientation.y;
        float ori_z = msg.orientation.z;
        float ori_w = msg.orientation.w;
        float acc_x = msg.linear_acceleration.x;
        float acc_y = msg.linear_acceleration.y;
        float acc_z = msg.linear_acceleration.z;
        float ang_x = msg.angular_velocity.x;
        float ang_y = msg.angular_velocity.y;
        float ang_z = msg.angular_velocity.z;

        csv_file << timestamp << "," << ori_x << "," << ori_y << "," << ori_z << "," << ori_w << "," << acc_x << "," << acc_y << "," << acc_z << "," << ang_x << "," << ang_y << "," << ang_z << "\n";
    }

    void data_callback(const sensor_msgs::msg::Imu msg_data)
    {
        int32_t timestamp_d_sec = msg_data.header.stamp.sec;
        uint32_t timestamp_d_nanosec = msg_data.header.stamp.nanosec;
        std::ostringstream oss;
        oss << std::setw(9) << std::setfill('0') << timestamp_d_nanosec;
        std::string result_d = oss.str();

        std::string timestamp_d = combineTimestamp(timestamp_d_sec, result_d);
        float acc_x_d = msg_data.linear_acceleration.x;
        float acc_y_d = msg_data.linear_acceleration.y;
        float acc_z_d = msg_data.linear_acceleration.z;
        float ang_x_d = msg_data.angular_velocity.x;
        float ang_y_d = msg_data.angular_velocity.y;
        float ang_z_d = msg_data.angular_velocity.z;

        csv_file_data << timestamp_d << "," << acc_x_d << "," << acc_y_d << "," << acc_z_d << "," << ang_x_d << "," << ang_y_d << "," << ang_z_d << "\n";
    }

    void raw_callback(const sensor_msgs::msg::Imu msg_raw)
    {
        int32_t timestamp_r_sec = msg_raw.header.stamp.sec;
        uint32_t timestamp_r_nanosec = msg_raw.header.stamp.nanosec;
        std::ostringstream oss;
        oss << std::setw(9) << std::setfill('0') << timestamp_r_nanosec;
        std::string result_r = oss.str();

        std::string timestamp_r = combineTimestamp(timestamp_r_sec, result_r);
        float acc_x_r = msg_raw.linear_acceleration.x;
        float acc_y_r = msg_raw.linear_acceleration.y;
        float acc_z_r = msg_raw.linear_acceleration.z;
        float ang_x_r = msg_raw.angular_velocity.x;
        float ang_y_r = msg_raw.angular_velocity.y;
        float ang_z_r = msg_raw.angular_velocity.z;

        csv_file_raw << timestamp_r << "," << acc_x_r << "," << acc_y_r << "," << acc_z_r << "," << ang_x_r << "," << ang_y_r << "," << ang_z_r << "\n";
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_imu_mdg;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_imu_data;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_imu_raw;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_imu_comp;
    std::ofstream csv_file;
    std::ofstream csv_file_data;
    std::ofstream csv_file_raw;
    std::ofstream csv_file_comp;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BenderCSVIMU>());
    rclcpp::shutdown();
    return 0;
}
