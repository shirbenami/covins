## 📌 Guide: How to Generate Ground Truth (GT) from Gazebo and Evaluate with EVO

This guide explains how to record the **Ground Truth** trajectory from Gazebo, convert it to **TUM format**, and compare it with your estimated SLAM trajectory using **evo\_ape**.

---

### ✅ Step 1 — Record the Ground Truth as a ROS bag

When your simulation is running, open a terminal and run:

```bash
rosbag record /uav0/ground_truth_to_tf/pose
```

For multiple UAVs:

```bash
rosbag record /uav0/ground_truth_to_tf/pose /uav1/ground_truth_to_tf/pose
```

This will create a `.bag` file with all the GT poses.

---

### ✅ Step 2 — Extract the GT pose data to CSV

After recording, convert the bag file to a CSV:

```bash
rostopic echo -b <your_bag.bag> -p /uav0/ground_truth_to_tf/pose > uav0_gt.csv
```

Example for multiple UAVs:

```bash
rostopic echo -b <your_bag.bag> -p /uav1/ground_truth_to_tf/pose > uav1_gt.csv
```

---

### ✅ Step 3 — Copy the file from Docker (if needed)

If you recorded inside a Docker container, copy it to your local machine:

```bash
docker cp <container_id>:/root/catkin_ws/uav0_gt.csv /home/<user>/Desktop/
```

---

### ✅ Step 4 — Convert the GT CSV to TUM format

The raw CSV will look like this:

```
%time,field.header.seq,field.header.stamp,field.pose.position.x,field.pose.position.y,field.pose.position.z,field.pose.orientation.x,field.pose.orientation.y,field.pose.orientation.z,field.pose.orientation.w
```

**EVO needs:**

```
timestamp tx ty tz qx qy qz qw
```

Use this Python snippet to convert:

```python
import pandas as pd

# Load the CSV
df = pd.read_csv("uav0_gt.csv")

# Convert nanoseconds to seconds
df['timestamp'] = df['%time'] / 1e9

# Select columns in TUM order
df_tum = df[['timestamp',
             'field.pose.position.x',
             'field.pose.position.y',
             'field.pose.position.z',
             'field.pose.orientation.x',
             'field.pose.orientation.y',
             'field.pose.orientation.z',
             'field.pose.orientation.w']]

# Save as TUM format
df_tum.to_csv("uav0_gt_tum.txt", sep=' ', index=False, header=False)

print("✅ TUM file created: uav0_gt_tum.txt")
```

---

### ✅ Step 5 — Run EVO APE evaluation

With your estimated trajectory in TUM format (`uav0_est_tum.txt`):

```bash
evo_ape tum uav0_gt_tum.txt uav0_est_tum.txt -va --align --correct_scale --plot --save_results results/gt_vs_est.zip
```

---

### 🔑 Tips

✅ Use `--align` and `--correct_scale` to align trajectories correctly
✅ If timestamps are shifted, use `--t_offset` to adjust
✅ Make sure units are in meters and timestamps are in seconds

---

**✨ Done! Now you have a Ground Truth trajectory ready for EVO!**
