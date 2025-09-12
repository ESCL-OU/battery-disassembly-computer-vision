#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(): Node("twist_publisher"), count_(0) {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_ros", 10);
      
    timer_ = this->create_wall_timer(500ms, [this]() -> void {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 0.5;
      message.linear.y = 0.0;
      message.linear.z = 0.0;

      message.angular.x = 0.0;
      message.angular.y = 0.0;
      message.angular.z = this->count_ % 2;
      this->publisher_->publish(message);
      
      this->count_++;
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  uint8_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}