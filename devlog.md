# Battery Auto Disassembly Computer Vision Devlog

### Monday August 25th, 2025

Using Ubuntu-24.04 on WSL2 in Windows 11

Installed ROS2 jazzy from https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Installed Gazebo from https://gazebosim.org/docs/latest/install_ubuntu/

Installed opencv C++ DLL for linux from https://opencv.org/get-started/

Followed CMake DLL linking tutorial from https://github.com/opencv/opencv/blob/4.x/samples/cpp/example_cmake/CMakeLists.txt

Point cloud background research

https://www.youtube.com/watch?v=DoZJaqBzSso&ab_channel=NicolaiNielsen

https://docs.opencv.org/3.4/d9/d25/group__surface__matching.html

https://www.youtube.com/watch?v=uFnqLFznuZU&ab_channel=TolgaBirdal


### September 1st, 2025

Setup workspace on a new laptop

https://gazebosim.org/docs/latest/fuel_insert/


### September 3rd, 2025

https://gazebosim.org/docs/latest/sdf_worlds/

https://gazebosim.org/docs/latest/sensors/

### September 8th, 2025

https://gazebosim.org/docs/latest/building_robot/

https://gazebosim.org/docs/latest/moving_robot/

### September 11th, 2025

https://gazebosim.org/docs/latest/ros2_integration/

important revision, we must use gazebo harmonic because we are going to use ros2 jazzy (ubuntu 24.04)

https://gazebosim.org/docs/harmonic/install_ubuntu/

https://index.ros.org/p/ros_gz_sim/

https://github.com/gazebosim/ros_gz/tree/jazzy

```
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
```

```
sudo apt install ros-jazzy-ros-gz
```

Getting some ros2 environment stuff setup for an example project. Still working on getting the ROS2 <-> Gazebo bridge setup and running.

### September 12th, 2025

https://stackoverflow.com/questions/65514366/visual-studio-code-includepath-issue-with-ros-headers

https://github.com/ros2/rviz/issues/1111 -> not really necessary, fixes warning when launching gazebo from cli

https://docs.nav2.org/setup_guides/sensors/setup_sensors_gz.html

### September 15th, 2025

Notes on ros gazebo bridge

Get the gazebo topic name by running `gz topic -l`

Get the gazebo topic message type by running `gz topic -i -t <topic name>`

Match the gazebo type with a ros type by looking at https://docs.ros.org/en/rolling/p/ros_gz_bridge/
