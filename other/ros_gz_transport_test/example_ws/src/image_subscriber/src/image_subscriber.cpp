#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>


class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("image_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/images_raw", 10, 
      [this](const sensor_msgs::msg::Image::UniquePtr msg) -> void {
        const cv::Mat img = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;
        RCLCPP_INFO(this->get_logger(), "I received an image!");
        cv::imshow("Display Window", img);
        cv::waitKey(1);
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