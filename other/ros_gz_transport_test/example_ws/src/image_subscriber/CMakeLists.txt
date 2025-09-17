cmake_minimum_required(VERSION 3.8)
project(image_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_subscriber src/image_subscriber.cpp)

ament_target_dependencies(image_subscriber rclcpp sensor_msgs OpenCV cv_bridge)

install(TARGETS
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
