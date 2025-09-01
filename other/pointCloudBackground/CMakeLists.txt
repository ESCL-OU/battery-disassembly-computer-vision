cmake_minimum_required(VERSION 3.5)

project(pointCloudBackground)

find_package(OpenCV REQUIRED)

add_executable(vizEx vizEx.cpp)

target_link_libraries(vizEx PUBLIC ${OpenCV_LIBS})