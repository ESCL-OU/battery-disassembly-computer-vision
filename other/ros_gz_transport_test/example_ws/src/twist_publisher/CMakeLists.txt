cmake_minimum_required(VERSION 3.8)
project(twist_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(twist_publisher src/twist_node.cpp)

ament_target_dependencies(twist_publisher rclcpp std_msgs geometry_msgs)

install(TARGETS
  twist_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
