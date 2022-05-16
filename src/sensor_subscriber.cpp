#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std;
using std::placeholders::_1;

class SensorsSubscriber : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg) {
        rclcpp::Time image_ts = image_msg->header.stamp;
        //RCLCPP_INFO(this->get_logger(), "image received timestamp %d s %d ns", image_msg->header.stamp.sec, image_msg->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "image received timestamp %lf s (%lu ns)", image_ts.seconds(), image_ts.nanoseconds());
    }
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg) {
        rclcpp::Time lidar_ts = lidar_msg->header.stamp;
        //RCLCPP_INFO(this->get_logger(), "lidar received timestamp %d s %d ns", lidar_msg->header.stamp.sec, lidar_msg->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "lidar received timestamp %lf s (%lu ns)", lidar_ts.seconds(), lidar_ts.nanoseconds());
    }
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        rclcpp::Time imu_ts = imu_msg->header.stamp;
        //RCLCPP_INFO(this->get_logger(), "IMU received timestamp %d s %d ns", imu_msg->header.stamp.sec, imu_msg->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "IMU received timestamp %lf s (%lu ns)", imu_ts.seconds(), imu_ts.nanoseconds());
    }
public:
    SensorsSubscriber() : Node("sensors_subscriber") {
        _image_sub = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw", 10, std::bind(&SensorsSubscriber::image_callback, this, _1));
        _lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("velodyne_points", 10, std::bind(&SensorsSubscriber::lidar_callback, this, _1));
        _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 10, std::bind(&SensorsSubscriber::imu_callback, this, _1));
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    cout << "Start to subscribe lidar and camera topic" << endl;
    rclcpp::spin(std::make_shared<SensorsSubscriber>());
    rclcpp::shutdown();
    return 0;
}
