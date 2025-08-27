cmake_minimum_required(VERSION 3.3)

project(opencv_verification)

find_package(OpenCV REQUIRED)

add_executable(opencv_verification opencvVerify.cpp)

target_link_libraries(opencv_verification PRIVATE ${OpenCV_LIBS})