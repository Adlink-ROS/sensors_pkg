#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

using namespace std;
using std::placeholders::_1;

class SensorsSubscriber : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
    rclcpp::Subscription<velodyne_msgs::msg::VelodyneScan>::SharedPtr _lidar_sub;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "image received timestamp %d %d", msg->header.stamp.sec, msg->header.stamp.nanosec);
    }
    void lidar_callback(const velodyne_msgs::msg::VelodyneScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "lidar received timestamp %d %d", msg->header.stamp.sec, msg->header.stamp.nanosec);
    }
public:
    SensorsSubscriber() : Node("sensors_subscriber") {
        _image_sub = this->create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&SensorsSubscriber::image_callback, this, _1));
        _lidar_sub = this->create_subscription<velodyne_msgs::msg::VelodyneScan>("velodyne_packets", 10, std::bind(&SensorsSubscriber::lidar_callback, this, _1));
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    cout << "Start to subscribe lidar and camera topic" << endl;
    rclcpp::spin(std::make_shared<SensorsSubscriber>());
    rclcpp::shutdown();
    return 0;
}
