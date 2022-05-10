#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

using namespace std;
using std::placeholders::_1;

class SensorsSubscriber : public rclcpp::Node
{
private:
    message_filters::Subscriber<sensor_msgs::msg::Image> _image_sub;
    message_filters::Subscriber<velodyne_msgs::msg::VelodyneScan> _lidar_sub;
    void callback(const sensor_msgs::msg::Image::SharedPtr image_msg, const velodyne_msgs::msg::VelodyneScan::SharedPtr lidar_msg) {
        rclcpp::Time image_ts = image_msg->header.stamp;
        rclcpp::Time lidar_ts = lidar_msg->header.stamp;
        //RCLCPP_INFO(this->get_logger(), "image received timestamp %d s %d ns", image_msg->header.stamp.sec, image_msg->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "image received timestamp %lf s (%d ns)", image_ts.seconds(), image_ts.nanoseconds());
        //RCLCPP_INFO(this->get_logger(), "lidar received timestamp %d s %d ns", lidar_msg->header.stamp.sec, lidar_msg->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "lidar received timestamp %lf s (%d ns)", lidar_ts.seconds(), lidar_ts.nanoseconds());
        rclcpp::Duration diff_ts = image_ts-lidar_ts;
        RCLCPP_INFO(this->get_logger(), "Time diff (image - lidar) %lf s (%d ns)", diff_ts.seconds(), diff_ts.nanoseconds());
    }
public:
    SensorsSubscriber() : Node("sensors_subscriber") {
        _image_sub.subscribe(this, "camera/image_raw");
        _lidar_sub.subscribe(this, "velodyne_packets");
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, velodyne_msgs::msg::VelodyneScan> approximate_policy;
        message_filters::Synchronizer<approximate_policy> syncApproximate(approximate_policy(10), _image_sub, _lidar_sub);

        syncApproximate.registerCallback(&SensorsSubscriber::callback, this);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    cout << "Start to subscribe lidar and camera topic" << endl;
    rclcpp::spin(std::make_shared<SensorsSubscriber>());
    rclcpp::shutdown();
    return 0;
}
