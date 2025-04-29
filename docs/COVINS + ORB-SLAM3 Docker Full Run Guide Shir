# COVINS + ORB-SLAM3 Docker Full Run Guide

This guide explains step-by-step how to set up and run COVINS and ORB-SLAM3 inside Docker, including RViz support (optional).

---

## 1. Prerequisites

### 1.1 Install Docker
```bash
sudo apt update
sudo apt install docker.io
sudo systemctl enable docker
sudo systemctl start docker
```
Check Docker is installed:
```bash
docker --version
```

### 1.2 Allow GUI access for Docker (for RViz later)
```bash
xhost +local:docker
```
(Note: you'll need to rerun this after each system reboot.)

---

## 2. Build the Docker Image for COVINS

Navigate to the docker folder:
```bash
cd ~/ws/covins_ws/src/covins/docker
make build NR_JOBS=8
```
(If it fails, lower NR_JOBS to 4.)

---

## 3. Running the System (in separate terminals)

Each component will run in a separate terminal window:

### 3.1 Start `roscore`
```bash
cd ~/ws/covins_ws/src/covins/docker
./run.sh -c
```

### 3.2 Start COVINS Server (Back-End)
```bash
cd ~/ws/covins_ws/src/covins/docker
./run.sh -s ../covins_comm/config/config_comm.yaml ../covins_backend/config/config_backend.yaml
```

### 3.3 Start ORB-SLAM3 Front-End

**Edit launch file first:**
```bash
nano ../orb_slam3/Examples/ROS/ORB_SLAM3/launch/launch_docker_ros_euroc.launch
```
Add inside the `<node>`:
```xml
<remap from="/camera/image_raw" to="/cam0/image_raw"/>
<remap from="/imu" to="/imu0"/>
```
Save and exit.

**Then launch:**
```bash
./run.sh -r ../covins_comm/config/config_comm.yaml ../orb_slam3/Examples/ROS/ORB_SLAM3/launch/launch_docker_ros_euroc.launch
```

### 3.4 Open a Terminal Inside Docker
```bash
cd ~/ws/covins_ws/src/covins/docker
./run.sh -t
```

---

## 4. Playing the ROS Bag (Dataset Input)

Inside the Docker terminal:

Make sure `MH_01_easy.bag` exists under `/root/covins_ws/`. If not, copy it from host:
```bash
docker cp ~/datasets/EuRoC_bag/MH_01_easy.bag <your_container_name>:/root/covins_ws/
```

Play the bag:
```bash
cd /root/covins_ws
rosbag play MH_01_easy.bag --clock
```

---

## 5. (Optional) Running RViz

### 5.1 Set ROS Environment on Host
On your normal Ubuntu (not Docker):
```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
```

### 5.2 Launch RViz
```bash
rviz
```

### 5.3 In RViz Add Topics
- `/covins_markers_be` (MarkerArray)
- `/covins_cloud_be` (PointCloud)
- `/tf` (TF Tree)

---

## Key Points to Remember

| Point | Note |
|:------|:-----|
| Separate terminal per component | `roscore`, `covins_server`, `orb_slam3`, `rosbag` |
| Remapping topics is crucial | `/cam0/image_raw` -> `/camera/image_raw`, `/imu0` -> `/imu` |
| Allow GUI access | `xhost +local:docker` |
| RViz runs only on host | Not from Docker without advanced X11 setup |

---

# ✅ You're Ready to Run COVINS + ORB-SLAM3 + RViz!

Feel free to contribute or suggest improvements!

