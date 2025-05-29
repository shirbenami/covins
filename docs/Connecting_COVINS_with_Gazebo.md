# 🛰️ Connecting ORB-SLAM3 + COVINS with Gazebo (ROS1 Melodic)

This guide documents the key steps and configuration changes required to successfully integrate a TurtleBot3 simulation in **Gazebo** with **ORB-SLAM3** and the **COVINS** collaborative SLAM backend.

---

## ✅ What Works

- Robot is launched in Gazebo and can be manually controlled via keyboard.
- Real-time camera and IMU data is published over ROS topics.
- ORB-SLAM3 correctly subscribes to the camera and IMU topics and begins monocular-inertial tracking.
- COVINS-Back-End accepts ORB-SLAM3 keyframes and constructs collaborative map.
- RViz shows visualized point cloud and image data from Gazebo.

---

## 🧾 Key Files and Modifications

### 1. `real_camera.yaml`

This YAML defines the camera intrinsics and IMU parameters used by ORB-SLAM3.

```yaml
%YAML:1.0

Camera.type: "PinHole"
Camera.fx: 554.3827128226441
Camera.fy: 554.3827128226441
Camera.cx: 320.5
Camera.cy: 240.5

Camera.k1: -0.28340811
Camera.k2: 0.07395907
Camera.p1: 0.00019359
Camera.p2: 1.76187114e-05

Camera.width: 640
Camera.height: 480
Camera.fps: 30.0
Camera.RGB: 1

# Identity transformation (camera and IMU assumed co-located for simulation)
Tbc: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1]

IMU.NoiseGyro: 1.7e-4
IMU.NoiseAcc: 2.0e-3
IMU.GyroWalk: 1.9393e-05
IMU.AccWalk: 3.0e-03
IMU.Frequency: 200
```
### 2. launch_docker_ros_euroc_gazebo.launch

This is the launch file used for running ORB-SLAM3 inside the container.

```
<launch>
  <param name="use_sim_time" value="true"/>

  <arg name="ag_n" default="0" />
  <arg name="voc" default="/root/covins_ws/src/covins/orb_slam3/Vocabulary/ORBvoc.txt" />
  <arg name="cam" default="/root/covins_ws/src/covins/orb_slam3/Examples/real_camera.yaml" />

  <node pkg="ORB_SLAM3" type="Mono_Inertial" name="ORB_SLAM3_monoi$(arg ag_n)" args="$(arg voc) $(arg cam)" output="screen">
    <remap from="/camera/image_raw" to="/camera/rgb/image_raw"/>
    <remap from="/imu" to="/imu"/>
  </node>
</launch>
```

### 3. run.sh – ROS Client Section (ORB-SLAM3)


This is the run.sh used to launch the ORB-SLAM3 container with the correct configurations and volume mounts.
```
elif [ $ROS_CLIENT -eq 1 ]; then
    CONFIG_FILE_COMM=$(absPath ${*: -2:1})
    LAUNCH_FILE=$(absPath ${*: -1})
    docker run \
    -it \
    --net=host \
    --volume "${CONFIG_FILE_COMM}:${CATKIN_WS}/src/covins/covins_comm/config/config_comm.yaml" \
    --volume "${LAUNCH_FILE}:${CATKIN_WS}/src/covins/orb_slam3/Examples/ROS/ORB_SLAM3/launch/$(basename ${LAUNCH_FILE})" \
    --volume "/home/user/ws/covins_ws/src/covins/orb_slam3/Examples/real_camera.yaml:${CATKIN_WS}/src/covins/orb_slam3/Examples/real_camera.yaml" \
    covins \
    /bin/bash
```

🔁 Additional Notes

* The environment variable use_sim_time=true is essential to synchronize timestamps from Gazebo’s simulated clock (/clock).

* The transformation Tbc was set to identity because in the simulation, the IMU and camera are assumed to be co-located. In real-world setups, the correct extrinsic calibration must be used.

* --net=host is used for Docker containers to share the same network namespace. This is crucial for ROS topic sharing across containers.

* ROS topics such as /camera/rgb/image_raw and /imu must be actively published and subscribed with matching message types and timestamps.
