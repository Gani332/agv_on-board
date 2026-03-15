# AGV On-Board Stack

On-vehicle ROS stack for the AGV robots used in the distributed multi-robot SLAM dataset project. This is the robot-side sensing, control, and dataset collection layer.

---

## Workspace Layout

```
agv_on-board/
├── myagv_ros/                  # Vendor robot workspace (base, LiDAR, URDF)
│   └── src/
│       ├── myagv_odometry/     # Serial interface to AGV base + odom publisher
│       ├── ydlidar_ros_driver/ # YDLiDAR X2 driver + laser_frame TF
│       ├── myagv_teleop/       # Keyboard teleoperation
│       ├── myagv_navigation/   # gmapping / AMCL / move_base launch files
│       ├── myagv_urdf/         # Robot URDF (not used in dataset bringup)
│       └── myagv_ps2/          # PS2 controller (unused in dataset)
│
├── agv_ws/                     # Dataset bringup workspace
│   └── src/
│       ├── agv_bringup/        # Main dataset launch + calibration files
│       │   ├── launch/
│       │   │   ├── bringup.launch   # ★ Start everything: odom + LiDAR + camera
│       │   │   ├── logging.launch   # rosbag recording
│       │   │   └── apriltag.launch  # AprilTag detection (future use)
│       │   └── calibration/         # ★ All calibration output files
│       │       ├── extrinsics.yaml
│       │       ├── camera_intrinsics.yaml
│       │       ├── imu_intrinsics.yaml
│       │       └── mocap_to_base.yaml
│       └── realsense-ros/      # RealSense SDK ROS wrapper
│
├── scripts/
│   └── calibration/            # ★ Calibration scripts (run on robot)
│       ├── extract_realsense_calib.py  # Reads factory intrinsics from live ROS topics
│       └── imu_static_test.py          # 60s static IMU noise characterisation
│
├── docs/
│   ├── PUBLISHABILITY_CHECKLIST.md
│   └── STAGE_1_CALIBRATION_SOP.md
│
├── drivers/                    # Vendored third-party SDKs
│   ├── YDLidar-SDK/
│   └── robot_pose_ekf/
│
├── configs/
│   └── yyy.rviz
│
└── workspaces/
    └── myagv_gmapping_ws/      # Legacy snapshot — do not use
```

---

## Quick Start

**On the robot** (SSH to `ubuntu@192.168.0.185`):

```bash
# 1. Pull latest
cd ~/slam_project && git pull origin main

# 2. Launch everything (odom + LiDAR + RealSense D455 + IMU + all TFs)
source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
roslaunch agv_bringup bringup.launch

# 3. Start recording (separate terminal)
roslaunch agv_bringup logging.launch
```

**Teleop** (separate terminal, robot side):
```bash
rosrun myagv_teleop myagv_teleop.py
```

---

## What Each Workspace Does

### `myagv_ros` — Robot runtime

The vendor workspace. Build and source this first as `agv_ws` depends on it.

| Package | What it does |
|---|---|
| `myagv_odometry` | Serial comms to AGV base controller; publishes `/odom` and `odom → base_footprint` TF |
| `ydlidar_ros_driver` | Drives YDLiDAR X2; publishes `/scan` and `base_footprint → laser_frame` TF |
| `myagv_teleop` | Keyboard teleoperation via `/cmd_vel` |
| `myagv_navigation` | gmapping / AMCL / move_base launch files (development use) |

### `agv_ws` — Dataset bringup

The dataset collection workspace. Source after `myagv_ros`.

| Package | What it does |
|---|---|
| `agv_bringup` | Single launch file starts all sensors; holds all calibration files |
| `realsense-ros` | RealSense SDK ROS wrapper for D455 colour, depth, and IMU |

### `scripts/calibration` — Calibration tools

Run on the robot to generate/update the calibration YAML files in `agv_bringup/calibration/`.

| Script | What it does |
|---|---|
| `extract_realsense_calib.py` | Reads factory intrinsics from `/camera/color/camera_info` and `/camera/depth/camera_info`; writes `camera_intrinsics.yaml` |
| `imu_static_test.py` | 60 s static test on `/camera/imu`; reports noise, drift, gravity alignment; writes `imu_intrinsics.yaml` |

---

## Transform Tree

```
world (PhaseSpace mocap)
  └── odom  [dynamic, published by myagv_odometry_node]
        └── base_footprint
              ├── laser_frame         [TF in X2.launch:     z=0.100m measured]
              └── camera_link         [TF in bringup.launch: x=-0.193m L-bracket]
                    ├── camera_color_optical_frame
                    ├── camera_depth_optical_frame
                    └── camera_imu_optical_frame
```

Static TFs are published at 40 Hz by `static_transform_publisher` nodes. Optical sub-frames are published by `realsense2_camera`.

---

## Key Topics

| Topic | Type | Source |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Teleop / planner |
| `/odom` | `nav_msgs/Odometry` | `myagv_odometry_node` |
| `/scan` | `sensor_msgs/LaserScan` | `ydlidar_ros_driver` |
| `/camera/color/image_raw` | `sensor_msgs/Image` | `realsense2_camera` |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | `realsense2_camera` |
| `/camera/imu` | `sensor_msgs/Imu` | `realsense2_camera` (200 Hz, accel+gyro combined) |

---

## Calibration Status (Stage 1 — Complete)

All calibration outputs live in `agv_ws/src/agv_bringup/calibration/`.

| Parameter | Status | Value |
|---|---|---|
| `base_footprint → laser_frame` z | ✅ Measured | 0.100 m |
| `base_footprint → laser_frame` x,y | ⏳ Pending | (0, 0) placeholder |
| `base_footprint → camera_link` | ✅ Measured | (−0.193, 0.003, 0.118) m |
| Camera colour intrinsics | ✅ Factory | fx=386.8, fy=386.4 px |
| Camera depth intrinsics | ✅ Factory | fx=fy=391.3 px |
| IMU noise density (accel) | ✅ Factory spec | 0.0028 m/s²/√Hz |
| IMU noise density (gyro) | ✅ Factory spec | 2.44×10⁻⁴ rad/s/√Hz |
| IMU gyro drift (measured) | ✅ Static test | 0.153 deg/s |
| Wheel odometry scale | ⏳ Pending | Stage 2 (needs mocap) |
| PhaseSpace extrinsic | ⏳ Pending | Stage 2 |

See `docs/STAGE_1_CALIBRATION_SOP.md` for the full calibration procedure.

---

## Hardware Connections

| Device | Port | Baud |
|---|---|---|
| AGV base controller | `/dev/ttyACM0` | — (firmware protocol) |
| YDLiDAR X2 | `/dev/ttyAMA0` | 115200 |
| RealSense D455 | USB 3.0 | — (USB) |

---

## Repository Sync

The robot clones this repo at `~/slam_project/`. The Mac side is the source of truth.

```bash
# Mac → GitHub
git push origin main

# Robot ← GitHub
ssh ubuntu@192.168.0.185 "cd ~/slam_project && git pull origin main"
```

---

## What Is Not Included Here

- PhaseSpace ROS driver and mocap integration (Stage 2)
- Multi-robot namespace management
- Dataset recording session management and metadata export
- Time synchronisation across robots
