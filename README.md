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
│       │   │   ├── bringup.launch   # ★ Start everything: odom + LiDAR + camera + TFs
│       │   │   ├── logging.launch   # rosbag recording (used by start_session.sh)
│       │   │   └── apriltag.launch  # AprilTag detection
│       │   └── calibration/         # ★ All calibration output files
│       │       ├── extrinsics.yaml          # Sensor poses relative to base_footprint
│       │       ├── camera_intrinsics.yaml   # D455 colour + depth intrinsics (factory)
│       │       ├── imu_intrinsics.yaml      # BMI085 noise + drift (static test)
│       │       └── mocap_to_base.yaml       # PhaseSpace extrinsic (Stage 2, pending)
│       └── realsense-ros/      # RealSense SDK ROS wrapper (vendored)
│
├── scripts/
│   ├── calibration/            # Calibration scripts (run on robot)
│   │   ├── extract_realsense_calib.py  # Reads factory intrinsics from live ROS topics
│   │   └── imu_static_test.py          # 60s static IMU noise characterisation
│   └── logging/                # ★ Dataset recording tools (run on Mac or robot)
│       ├── start_session.sh    # Wrapper: pre-flight checks + rosbag + session manifest
│       └── validate_bag.py     # Post-run bag quality validator (publishability check)
│
├── docs/
│   ├── PUBLISHABILITY_CHECKLIST.md
│   └── STAGE_1_CALIBRATION_SOP.md
│
├── drivers/                    # Vendored third-party SDKs
│   ├── YDLidar-SDK/
│   └── robot_pose_ekf/
│
└── configs/
    └── yyy.rviz
```

---

## Quick Start

**On the robot** (SSH to `ubuntu@192.168.0.185`):

```bash
# 1. Pull latest
cd ~/slam_project && git pull origin main

# 2. Source both workspaces (order matters — myagv_ros first)
source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash

# 3. Launch everything (odom + LiDAR + RealSense D455 colour/depth + all TFs)
roslaunch agv_bringup bringup.launch

# 4. Start a recording session (separate terminal, run from Mac or robot)
#    Sets env vars, runs pre-flight checks, records bag, writes session manifest
bash scripts/logging/start_session.sh
```

**Teleop** (separate terminal, robot side):
```bash
rosrun myagv_teleop myagv_teleop.py
```

**Validate a recorded bag** (Mac or robot, requires ROS sourced):
```bash
# Run on robot where ROS is available
source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python3 scripts/logging/validate_bag.py ~/agv_data/<bag_name>.bag
# Exit 0 = PASS, 1 = FAIL, 2 = WARN
```

---

## What Each Workspace Does

### `myagv_ros` — Robot runtime

The vendor workspace. Build and source this first — `agv_ws` depends on it.

| Package | What it does |
|---|---|
| `myagv_odometry` | Serial comms to AGV base controller (`/dev/ttyACM0`); publishes `/odom` and `odom → base_footprint` TF |
| `ydlidar_ros_driver` | Drives YDLiDAR X2 (`/dev/ttyAMA0`); publishes `/scan` and `base_footprint → laser_frame` TF |
| `myagv_teleop` | Keyboard teleoperation via `/cmd_vel` |
| `myagv_navigation` | gmapping / AMCL / move_base launch files (development use only) |

### `agv_ws` — Dataset bringup

The dataset collection workspace. Source after `myagv_ros`.

| Package | What it does |
|---|---|
| `agv_bringup` | Single launch file starts all sensors; holds all calibration files |
| `realsense-ros` | RealSense SDK ROS wrapper for D455 colour and depth (vendored) |

### `scripts/logging` — Dataset recording tools

| Script | Where to run | What it does |
|---|---|---|
| `start_session.sh` | Robot or Mac | Pre-flight topic checks, auto-generates `session_manifest.yaml`, wraps `logging.launch`, finalises manifest with duration + bag size on exit |
| `validate_bag.py` | Robot (needs ROS) | Checks topic presence, rates vs EuRoC/TUM thresholds, frame drops, colour/depth sync, bag integrity. Exit 0/1/2 = PASS/FAIL/WARN |

### `scripts/calibration` — Calibration tools

Run on the robot to generate/update the calibration YAML files in `agv_bringup/calibration/`.

| Script | What it does |
|---|---|
| `extract_realsense_calib.py` | Reads factory intrinsics from `/camera/color/camera_info` and `/camera/depth/camera_info`; writes `camera_intrinsics.yaml` |
| `imu_static_test.py` | 60 s static test on `/camera/imu`; reports noise, drift, gravity alignment; writes `imu_intrinsics.yaml` |

---

## Transform Tree

```
world (PhaseSpace mocap — Stage 2, pending)
  └── odom  [dynamic, published by myagv_odometry_node]
        └── base_footprint
              ├── laser_frame     [X2.launch:     z=0.100m measured, x/y pending]
              └── camera_link     [bringup.launch: CAD values from assembly.urdf]
                    ├── camera_color_optical_frame   ]
                    ├── camera_depth_optical_frame   } published by realsense2_camera
                    └── camera_imu_optical_frame     ]
```

Static TFs are published at 40 Hz by `static_transform_publisher` nodes.

**Camera extrinsic** (`base_footprint → camera_link`):
```
x=-0.132025  y=0.000153  z=0.187925  rpy=(π/2, −0.0079, −π/2)
```
Source: `assembly.urdf` CAD. An L-bracket mount was trialled 2026-03-15 but caused significant LiDAR forward-sector blockage and was reverted.

---

## Key Topics

| Topic | Target Hz | Measured Hz | Type | Source |
|---|---|---|---|---|
| `/cmd_vel` | on demand | — | `geometry_msgs/Twist` | Teleop / planner |
| `/odom` | 20 Hz | ~13 Hz | `nav_msgs/Odometry` | `myagv_odometry_node` |
| `/scan` | 18 Hz | ~18 Hz | `sensor_msgs/LaserScan` | `ydlidar_ros_driver` |
| `/camera/color/image_raw` | 15 Hz | ~15 Hz | `sensor_msgs/Image` | `realsense2_camera` |
| `/camera/color/camera_info` | 15 Hz | ~15 Hz | `sensor_msgs/CameraInfo` | `realsense2_camera` |
| `/camera/aligned_depth_to_color/image_raw` | 15 Hz | ~15 Hz | `sensor_msgs/Image` | `realsense2_camera` |
| `/camera/imu` | 200 Hz | — (disabled) | `sensor_msgs/Imu` | **USB 2 only — see note below** |
| `/tag_detections` | on demand | — | `apriltag_ros/AprilTagDetectionArray` | `apriltag_ros` |

Measured Hz from 2026-03-15 test bags. `/odom` lower than target — base controller serial output rate; further investigation pending.

### USB 2 limitation

The D455 is connected via USB 2.0. At USB 2 bandwidth (~480 Mbps theoretical, ~200 Mbps practical for bulk transfers):

- Colour 640×480@15Hz ≈ 110 Mbps
- Depth 640×480@15Hz ≈ 74 Mbps
- **Total ≈ 184 Mbps — within USB 2 budget**
- IMU (accel + gyro) ≈ 77 Mbps — pushes total over limit

`bringup.launch` therefore disables `enable_accel`, `enable_gyro`, and `enable_sync`. Colour and depth stream reliably. IMU is unavailable until a USB 3 port/hub is available. This is noted as a known limitation in the dataset paper.

**Validated measured rates** (2026-03-15, 119s static test bag):
`/scan` 17.2 Hz · `/odom` 12.6 Hz · colour 15.0 Hz · depth 14.9 Hz · colour/depth sync mean 0.3 ms · max sync 66.9 ms · frame drops ~0%

**Motion validation:** pending — requires a 2-minute drive session recorded via `logging.launch` (not manual `rosbag record`).

---

## Calibration Status (Stage 1 — Complete)

All outputs live in `agv_ws/src/agv_bringup/calibration/`.

| Parameter | Status | Value |
|---|---|---|
| `base_footprint → laser_frame` z | ✅ Measured | 0.100 m |
| `base_footprint → laser_frame` x, y | ⏳ Pending | (0, 0) placeholder |
| `base_footprint → camera_link` | ✅ CAD | (−0.132, 0.000, 0.188) m |
| Camera colour intrinsics (fx, fy) | ✅ Factory | 386.8, 386.4 px |
| Camera depth intrinsics (fx, fy) | ✅ Factory | 391.3, 391.3 px |
| IMU noise density — accel | ✅ Factory | 0.0028 m/s²/√Hz |
| IMU noise density — gyro | ✅ Factory | 2.44×10⁻⁴ rad/s/√Hz |
| IMU gyro drift | ✅ Static test | 0.153 deg/s |
| Wheel odometry scale | ⏳ Pending | Stage 2 (needs mocap) |
| PhaseSpace extrinsic | ⏳ Pending | Stage 2 |

See `docs/STAGE_1_CALIBRATION_SOP.md` for the full calibration procedure.

---

## Hardware Connections

| Device | Port | Notes |
|---|---|---|
| AGV base controller | `/dev/ttyACM0` | STM32 firmware protocol |
| YDLiDAR X2 | `/dev/ttyAMA0` | 115200 baud |
| RealSense D455 | USB 2.0 (current) | USB 3 needed for IMU; see USB 2 note above |

---

## Repository Sync

The robot clones this repo at `~/slam_project/`. The Mac is the source of truth.

```bash
# Mac → GitHub
git push origin main

# Robot ← GitHub
ssh ubuntu@192.168.0.185 "cd ~/slam_project && git pull origin main"
```

---

## Troubleshooting

### LiDAR fails to start ("Operation timed out")

This is a transient motor spin-up timeout in the X2 driver. Kill and retry:
```bash
rosnode kill /ydlidar_ros_driver_node
# or kill the whole bringup and re-run roslaunch agv_bringup bringup.launch
```

### Camera nodelet dies immediately on startup

Cause: stale nodelet registration in the ROS master from a previous session.
Fix: kill all ROS processes cleanly before relaunching.
```bash
# On robot
killall -9 rosmaster roscore roslaunch nodelet python2 2>/dev/null
sleep 2
# Then relaunch normally
roslaunch agv_bringup bringup.launch
```

### Camera `control_transfer returned error, index: 768`

These are benign USB 2 bandwidth warnings from the RealSense SDK polling hardware registers. Image streaming is unaffected. They appear in the log but do not cause node failure.

### `roslaunch agv_bringup bringup.launch` finds multiple files

A stale copy of `bringup.launch` was left in the `calibration/` folder on the robot.
```bash
rm ~/slam_project/agv_ws/src/agv_bringup/calibration/bringup.launch
```

---

## What Is Not Included Here

- PhaseSpace ROS driver and mocap integration (Stage 2)
- Multi-robot namespace management and ROS_MASTER_URI configuration
- Time synchronisation across robots (PTP / `chrony`)
- Wheel odometry physical calibration (Stage 2, needs mocap ground truth)
