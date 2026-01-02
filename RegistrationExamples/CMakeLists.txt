cmake_minimum_required(VERSION 3.24.0)
project(TestingZividApi)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Zivid libraries

find_package(Zivid COMPONENTS Core REQUIRED)

if (Zivid_FOUND) 
    message(STATUS "Found package: ${Zivid_VERSION} at ${Zivid_DIR}")
else()
    message("Couldn't find Zivid")
endif()

# Executable
add_executable(escl_test_capture escl_test_capture.cpp)

add_executable(hw hw.cpp)

add_executable(owl_composite owl_composite.cpp)

add_executable(visualizer_snippet visualizer_snippet.cpp)

add_executable(owl_pose_estimation owl_pose_estimation.cpp)

add_executable(driver_pose_estimation driver_pose_estimation.cpp)

add_executable(can_pose_estimation can_pose_estimation.cpp)

# Link libraries
target_link_libraries(escl_test_capture 
    Zivid::Core
    Zivid::Visualization
    )

target_link_libraries(escl_test_capture 
    Zivid::Core
    Zivid::Visualization
    )

target_link_libraries(owl_composite 
    Zivid::Core
    Zivid::Visualization
    )

target_link_libraries(visualizer_snippet 
    Zivid::Core
    Zivid::Visualization
    )

target_link_libraries(owl_pose_estimation 
    Zivid::Core
    Zivid::Visualization
    )

target_link_libraries(driver_pose_estimation
    Zivid::Core
    Zivid::Visualization
    )

target_link_libraries(can_pose_estimation
    Zivid::Core
    Zivid::Visualization
    )

#include directories
target_include_directories(owl_composite PRIVATE
    "C:/Users/ESCL/Documents/eigen-5.0.0/eigen-5.0.0"
)

target_include_directories(owl_pose_estimation PRIVATE
    "C:/Users/ESCL/Documents/eigen-5.0.0/eigen-5.0.0"
)

target_include_directories(driver_pose_estimation PRIVATE
    "C:/Users/ESCL/Documents/eigen-5.0.0/eigen-5.0.0"
)

target_include_directories(can_pose_estimation PRIVATE
    "C:/Users/ESCL/Documents/eigen-5.0.0/eigen-5.0.0"
)