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

## 3. Set and Verify Server IP Address

IMPORTANT: Make sure that the sys.server_ip in config_comm.yaml matches the IP address of the machine running the COVINS server (back-end).

### 3.1 Check the local IP address

In the machine running the server:
```bash
hostname -I
```
Copy the IP (e.g., 192.168.0.102) and update this line in:
```bash
nano ~/ws/covins_ws/src/covins/covins_comm/config/config_comm.yaml
```
Update the line:
```bash
sys.server_ip: 'YOUR_SERVER_IP_HERE'
```
Save and exit.

## 4. Running the System (in separate terminals)

Each component will run in a separate terminal window:

### 4.1 Start `roscore`
```bash
cd ~/ws/covins_ws/src/covins/docker
./run.sh -c
```

### 4.2 Start COVINS Server (Back-End)
```bash
cd ~/ws/covins_ws/src/covins/docker
./run.sh -s ../covins_comm/config/config_comm.yaml ../covins_backend/config/config_backend.yaml
```

### 4.3 Start ORB-SLAM3 Front-End (Agent 0) 

**Edit launch file first:**
```bash
nano ../orb_slam3/Examples/ROS/ORB_SLAM3/launch/launch_docker_ros_euroc.launch
```
Add inside the `<node>`:
```xml
<remap from="/camera/image_raw" to="/cam0/image_raw"/>
<remap from="/imu" to="/imu0"/>
```
Set a unique node name:
```bash
<arg name="ag_n" default="0" />
```
Save and exit.

**Then launch:**
```bash
./run.sh -r ../covins_comm/config/config_comm.yaml ../orb_slam3/Examples/ROS/ORB_SLAM3/launch/launch_docker_ros_euroc.launch
```
### 4.4 Start ORB-SLAM3 Front-End (Agent 1)

Duplicate the launch file if needed:
```bash
cp ../orb_slam3/Examples/ROS/ORB_SLAM3/launch/launch_docker_ros_euroc.launch \
   ../orb_slam3/Examples/ROS/ORB_SLAM3/launch/launch_docker_ros_euroc_agent1.launch
```
Edit the duplicated file:

```bash
nano ../orb_slam3/Examples/ROS/ORB_SLAM3/launch/launch_docker_ros_euroc_agent1.launch
```
Add inside the <node>:

```bash
<remap from="/camera/image_raw" to="/cam0/image_raw1"/>
<remap from="/imu" to="/imu1"/>
```
Set a unique node name:

```bash
<arg name="ag_n" default="1" />
```
Then launch:
```bash
./run.sh -r ../covins_comm/config/config_comm.yaml ../orb_slam3/Examples/ROS/ORB_SLAM3/launch/launch_docker_ros_euroc_agent1.launch
```
---

### 4.5 Open a Terminal Inside Docker
```bash
cd ~/ws/covins_ws/src/covins/docker
./run.sh -t
```



## 5. Playing the ROS Bag (Dataset Input)

Inside the Docker terminal:

Make sure `MH_01_easy.bag` exists under `/root/covins_ws/`. If not, copy it from host:
```bash
docker cp ~/datasets/EuRoC_bag/MH_01_easy.bag <your_container_name>:/root/covins_ws/
```

Play the bag:
Agent 0:

```bash
cd /root/covins_ws
rosbag play MH_01_easy.bag /cam0/image_raw:=/cam0/image_raw0 /imu0:=/imu0 --clock --start 45
```

Agent 1:
```bash
cd /root/covins_ws
rosbag play MH_02_easy.bag /cam0/image_raw:=/cam0/image_raw1 /imu0:=/imu1 --clock --start 35
```
OR 
```bash
rosbag play MH_01_easy.bag 
```
Now both agents should be publishing to the COVINS back-end and sharing map data collaboratively.
---

## 6. RViz Visualization (Optional)

To visualize the COVINS backend output (e.g., trajectories, keyframes, landmarks) on your local machine, follow the steps below. This allows you to run RViz even if you are using Docker with sudo privileges.

### 6.1 Allow X11 Access

On your host machine (not inside Docker), allow the root user access to the X11 display:

```bash
xhost +SI:localuser:root
```
This allows GUI applications like RViz running as root (in Docker) to display on your screen.

### 6.2 Run the Docker Container with GUI Support

Instead of using ./run.sh -t, run the Docker container manually with all required permissions for GUI:

```bash
sudo docker run -it \
  --net=host \
  --env="DISPLAY=:1" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="XAUTHORITY=/run/user/1017/gdm/Xauthority" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/run/user/1017/gdm/Xauthority:/run/user/1017/gdm/Xauthority" \
  --volume="$HOME/ws:/root/ws" \
  covins_terminal \
  bash
```
Replace covins_terminal with the name of your actual Docker image.

Make sure the value of DISPLAY matches your host system (use echo $DISPLAY to check).

### 6.3 Run RViz Inside the Container

Once inside the Docker terminal, run:

```bash
rosrun rviz rviz -d /root/covins_ws/src/covins/covins_backend/config/covins.rviz
```

You should now see the RViz GUI displaying the COVINS backend data in real time.
Ensure that the covins.rviz file has the correct frame set (e.g., odom instead of world). If you encounter an error such as Fixed Frame [world] does not exist, open RViz, set the Fixed Frame to odom, and then save the configuration using:

File → Save Config As → covins_updated.rviz

You can then load this updated configuration using:

```bash
rosrun rviz rviz -d /root/covins_ws/src/covins/covins_backend/config/covins_updated.rviz
```

You should now see the RViz GUI displaying the COVINS backend data in real time, including keyframes, trajectories, and point clouds.

* for add the images- change image topic- /cam0/image_raw, /cam0/image_raw1, /cam0/image_raw2 .

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

