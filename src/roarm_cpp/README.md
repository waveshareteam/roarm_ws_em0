# RoArm

This folder contains the code for the RoArm-M2-S robotic manipulator. It includes implementations for Inverse Kinematics (IK), Forward Kinematics (FK) analytically without using extra packages like MoveIt2 to significantly save the computational resources, and a script to get the servo feedback which can be useful in various aspects like implementing different control strategies.

## Project Overview

RoArm-M2-S is a robotic manipulator designed for various tasks. This repository includes:
- `pub_ik.cpp`: A C++ file to compute the Inverse Kinematics (IK) of the manipulator.
- `pub_fk.cpp`: A C++ file to compute the Forward Kinematics (FK) of the manipulator.
- `servo_feedback.py`: A Python file to get the servo feedback from the manipulator.

## Installation

### Prerequisites

Ensure you have the following software and libraries installed:
- C++ compiler (e.g., `g++`)
- Python 3.x
- ROS2 Humble (Robot Operating System)
- All the required packages to use RoArm-M2-S

## Usage 

- Clone the cpp and python packages seperately in the respective ROS2 workspaces. 
- Update your CMakeLists.txt and package.xml files in the cpp workspace with the following.

#### Example for CMakeLists.txt:
```cmake
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
```
```cmake
add_executable(pub_ik src/pub_ik.cpp)
ament_target_dependencies(pub_ik rclcpp std_msgs geometry_msgs sensor_msgs)

add_executable(pub_fk src/pub_fk.cpp)
ament_target_dependencies(pub_fk rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS
  pub_fk
  pub_ik
  DESTINATION lib/${PROJECT_NAME})
```
#### Example packages.xml:
```xml
<build_type>ament_cmake</build_type>
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
```

- Update your package.xml in the python workspace with the following. 
#### Example package.xml:
```xml 
<build_depend>rclpy</build_depend>
<build_depend>sensor_msgs</build_depend>
<build_depend>geometry_msgs</build_depend>
<exec_depend>rclpy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
```
- Build the workspace and run the nodes using ROS2 commands. 

