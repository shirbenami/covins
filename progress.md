# Project Progress

## ✅ Completed



## 🛠️ In Progress
- [ ] Dockerized build setup for ROS 2 + COVINS
- [ ] Container images (base/dev) with clean separation
- [ ] Initial porting strategy and directory structure setup

- [ ] ROS 2 wrapper for camera and IMU inputs
- [ ] Inter-agent communication (map points and keyframes) via ROS 2 topics
- [ ] Port RViz visualization to ROS 2

## 🔜 Upcoming

- [ ] Implement ROS 2 service for map save/load
- [ ] Benchmark communication and map fusion performance
- [ ] Automated launch system using ROS 2 launch

## Challenges

- Replacing point-to-point TCP messaging with ROS 2 pub/sub requires careful synchronization between agents.
- Ensuring compatibility with ORB-SLAM3 backend requires maintaining timestamp and extrinsics integrity.
