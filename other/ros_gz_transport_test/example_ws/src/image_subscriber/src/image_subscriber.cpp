#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("image_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/images_raw", 10, [this](sensor_msgs::msg::Image::UniquePtr msg) -> void { 
      RCLCPP_INFO(this->get_logger(), "I received an image!");
    });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}