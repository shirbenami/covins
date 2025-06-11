# Technical Context

## System Overview

COVINS (Collaborative Visual-Inertial SLAM) is a SLAM system designed for multi-agent scenarios. It originally uses:
- **ROS 1** for camera/IMU integration and visual output.
- **TCP sockets** for map and pose data sharing among agents.

This project reimagines that architecture to fit the **ROS 2** paradigm.

## Migration Plan

| Component                | From                | To                 |
|--------------------------|---------------------|--------------------|
| Sensor Input (Camera, IMU) | ROS 1 topics        | ROS 2 topics       |
| Visualization (Rviz)    | ROS 1 (Rviz)        | ROS 2 (Rviz2)      |
| Agent Communication     | TCP Sockets         | ROS 2 Topics/Services |
| Node Management          | ROS 1 launch files  | ROS 2 launch files |

## Docker Setup

The codebase is containerized with Docker to ensure reproducibility and portability. There are two stages:

- **Base Image (`covins_base_abs_layer`)**: Installs ROS 2, dependencies, and shared libraries.
- **Development Image (`covins_dev_abs_layer`)**: Adds the COVINS build and any mounted source code for active development.

## ROS 2 Stack

- **ROS 2 Humble** is the target distribution.
- Native support for:
    - `rclcpp`-based nodes
    - `sensor_msgs`, `nav_msgs`, and custom message types
    - Launch system (`launch.py`)
