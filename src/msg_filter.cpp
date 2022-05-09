#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;

class SensorsSubscriber : public rclcpp::Node
{
private:
    void callback() {}
public:
#if 0
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> disparity_sub_;

    MultiSubscriber(const std::string& name)
    : Node(name), 
      image_sub_(message_filters::Subscriber<sensor_msgs::msg::Image>(this, "image_color_topic")),
      disparity_sub_(message_filters::Subscriber<sensor_msgs::msg::Image>(this, "image_disparity_topic"))
    {

      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_policy;
      message_filters::Synchronizer<approximate_policy>syncApproximate(approximate_policy(10), image_sub_, disparity_sub_);

      syncApproximate.registerCallback(&MultSubscriber::disparityCb,this);


    }
    private:
    void disparityCb(const sensor_msgs::msg::Image::SharedPtr disparity_msg, const sensor_msgs::msg::Image::SharedPtr color_msg){
   //dosomething
                }
#endif
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  cout << "Subscribe to lidar and camera topic" << endl;
  return 0;
}
