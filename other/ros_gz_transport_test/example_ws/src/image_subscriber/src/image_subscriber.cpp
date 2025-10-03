#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/surface_matching/ppf_match_3d.hpp>


class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("image_subscriber")
  {
    rgb_image_subscription = this->create_subscription<sensor_msgs::msg::Image>("/images_raw", 10, 
      [this](const sensor_msgs::msg::Image::UniquePtr msg) -> void {
        const cv::Mat img = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;
        RCLCPP_INFO(this->get_logger(), "I received an image!");
        cv::imshow("Display Window", img);
        cv::waitKey(1);
    });

    point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("/point_cloud_raw", 10, 
    [this](const sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void {
      cv::ppf_match_3d::PPF3DDetector detector(0.03, 0.05);
      RCLCPP_INFO(this->get_logger(), "I received point cloud data!");
    });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_subscription;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}