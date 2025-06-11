# Project Brief

## Objective

This project aims to develop a **ROS 2 communication wrapper for COVINS**, a collaborative visual-inertial SLAM framework. The main goal is to replace the legacy communication infrastructure used by COVINS — namely:

- **ROS 1 I/O interfaces** (camera, IMU, map visualization, etc.)
- **TCP socket-based inter-agent communication**

with a fully **ROS 2-based system**. This transition will enable better modularity, improved performance in multi-agent systems, and long-term maintainability of the system within modern ROS 2 ecosystems.

## Scope

- Port all existing ROS 1 node interfaces to ROS 2.
- Replace internal TCP-based communication mechanisms with ROS 2 topics/services.
- Ensure compatibility with RViz2 and ROS 2 tooling.
- Maintain runtime configurability and launch convenience within the ROS 2 environment.

This work lays the groundwork for future deployments of collaborative SLAM in ROS 2-native robotic systems.
