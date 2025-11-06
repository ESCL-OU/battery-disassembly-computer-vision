cmake_minimum_required(VERSION 3.5)
project(ppf_test)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED COMPONENTS)
find_package(PCL REQUIRED COMPONENTS common io filters)
add_definitions(${PCL_DEFINITIONS})

add_executable(ppf_test src/ppf_test.cpp)
target_include_directories(ppf_test PRIVATE ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(ppf_test PRIVATE ${OpenCV_LIBS} ${PCL_LIBRARIES})

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Libraries: ${OpenCV_LIBS}")