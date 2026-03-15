# AGV On-Board Stack

## Purpose

This folder contains the on-vehicle ROS stack for the AGV robots used in the distributed SLAM dataset project.

The local layout has now been refactored to mirror the robot-side structure that actually runs on the AGVs:

- `myagv_ros/` for the main robot workspace
- `agv_ws/` for bring-up, logging, and RealSense camera integration
- `workspaces/myagv_gmapping_ws/` kept as a legacy snapshot from the earlier local copy

This is the robot-side sensing, control, and logging layer. It is the foundation for dataset collection, but publishability still depends on calibration, synchronization, metadata, QA, and benchmark design.

## Canonical Layout

Use this as the main mental model going forward:

```text
agv_on-board/
├── agv_ws/
│   └── src/
│       ├── agv_bringup/
│       └── realsense-ros/
├── myagv_ros/
│   └── src/
│       ├── charging/
│       ├── myagv_navigation/
│       ├── myagv_odometry/
│       ├── myagv_ps2/
│       ├── myagv_teleop/
│       ├── myagv_urdf/
│       └── ydlidar_ros_driver/
├── docs/
├── drivers/
├── scripts/
└── workspaces/
    └── myagv_gmapping_ws/   # legacy snapshot
```

## Which Workspace To Use

### `myagv_ros`

This is the main robot runtime workspace and should be treated as the primary source tree for:

- base control
- teleop
- navigation
- LiDAR
- URDF
- auxiliary robot packages

### `agv_ws`

This is the dataset bring-up and camera-side workspace and should be treated as the primary source tree for:

- unified bring-up
- rosbag logging
- RealSense integration
- dataset-facing launch composition

### `workspaces/myagv_gmapping_ws`

This is a preserved older local snapshot. Keep it for reference, but do not treat it as the canonical robot layout anymore.

## Publishability Checklist

The formal dataset quality checklist for this project lives here:

- [PUBLISHABILITY_CHECKLIST.md](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/docs/PUBLISHABILITY_CHECKLIST.md)

That document defines the actual bar for a publishable distributed SLAM dataset:

- sensor coverage
- organization
- calibration
- synchronization
- quality thresholds
- scenario design
- run duration
- QA
- benchmark utility

## What Each Area Does

### `myagv_ros`

This is now the main ROS 1 catkin workspace that matches the robot-side deployment layout.

#### `myagv_odometry`

This package interfaces with the AGV base controller over serial.

Its responsibilities are:

- subscribe to `cmd_vel`
- send scaled velocity commands to the base
- read back velocity and IMU-like feedback
- integrate odometry
- publish `nav_msgs/Odometry`
- broadcast `odom -> base_footprint`

Main files:

- [myAGV.cpp](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/myagv_odometry/src/myAGV.cpp)
- [myAGVSub.cpp](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/myagv_odometry/src/myAGVSub.cpp)
- [myagv_active.launch](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/myagv_odometry/launch/myagv_active.launch)

#### `ydlidar_ros_driver`

This package publishes LiDAR data into ROS.

Its responsibilities are:

- connect to the YDLidar device
- publish `sensor_msgs/LaserScan` on `scan`
- publish a point cloud on `point_cloud`
- define the static transform from the robot base to the laser frame

Main files:

- [ydlidar_ros_driver.cpp](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/ydlidar_ros_driver/src/ydlidar_ros_driver.cpp)
- [X2.launch](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/ydlidar_ros_driver/launch/X2.launch)

#### `myagv_teleop`

This package provides keyboard teleoperation.

Its responsibilities are:

- read keyboard input
- convert keys into `geometry_msgs/Twist`
- publish commands to `/cmd_vel`

Main file:

- [myagv_teleop.py](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/myagv_teleop/scripts/myagv_teleop.py)

#### `myagv_navigation`

This package contains launch files and RViz configurations for mapping and navigation.

Its responsibilities are:

- launch gmapping for online 2D SLAM
- launch AMCL for localization against a saved map
- compose full runtime launch files for SLAM and navigation

Main files:

- [gmapping.launch](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/myagv_navigation/launch/gmapping.launch)
- [myagv_slam_laser.launch](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/myagv_navigation/launch/myagv_slam_laser.launch)
- [navigation_active.launch](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/myagv_navigation/launch/navigation_active.launch)
- [amcl.launch](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/myagv_navigation/launch/amcl.launch)

### `agv_ws`

This workspace matches the AGV-side dataset bring-up path.

#### `agv_bringup`

This package is the main place for:

- unified driver bring-up
- logging launch files
- scenario-facing collection workflows

Main files:

- [bringup.launch](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/agv_ws/src/agv_bringup/launch/bringup.launch)
- [logging.launch](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/agv_ws/src/agv_bringup/launch/logging.launch)

#### `realsense-ros`

This is the camera-side workspace content for RealSense D435i integration.

### `scripts`

This contains a standalone Python serial test script:

- [myAGVBase.py](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/scripts/myAGVBase.py)

It appears to be a low-level test utility for directly sending and receiving base packets outside the ROS node.

### `drivers`

This folder contains vendored third-party components and supporting code.

#### `drivers/YDLidar-SDK`

Vendor SDK used by the LiDAR ROS driver.

#### `drivers/robot_pose_ekf`

Vendored ROS package for pose fusion. It exists in this repository, but it does not appear to be part of the main onboard launch path currently in use.

#### Other driver folders

There are also directories such as `librealsense` and `teleop_twist_keyboard`, but the main active onboard stack identified here is centered around the catkin workspace packages above.

### `configs`

This contains RViz-related configuration assets such as:

- [yyy.rviz](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/configs/yyy.rviz)

## Runtime Architecture

The main runtime can be understood as the following chain:

```text
Keyboard / planner
        |
        v
     /cmd_vel
        |
        v
myagv_odometry_node
        |
        +--> serial commands to AGV base
        +--> serial feedback from AGV base
        |
        +--> /odom
        +--> TF: odom -> base_footprint

YDLidar device
        |
        v
ydlidar_ros_driver
        |
        +--> /scan
        +--> /point_cloud
        +--> TF: base_footprint -> laser_frame

/odom + /scan
        |
        v
gmapping / AMCL / move_base
```

## ROS Topics and Frames

### Core topics

The stack primarily revolves around these ROS topics:

- `cmd_vel`: commanded base motion
- `odom`: robot odometry estimate
- `scan`: 2D LiDAR scans
- `point_cloud`: LiDAR points published by the LiDAR driver

### Core frames

The main TF frames are:

- `odom`
- `base_footprint`
- `base_link`
- `laser_frame`

The transform chain is intended to be:

```text
odom -> base_footprint -> base_link
                      -> laser_frame
```

## Main Launch Flow

### Base + LiDAR bringup

The launch file:

- [myagv_active.launch](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/myagv_odometry/launch/myagv_active.launch)

starts:

- `myagv_odometry_node`
- a static transform between `base_footprint` and `base_link`
- the YDLidar launch file

### Online SLAM

The launch file:

- [myagv_slam_laser.launch](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/myagv_navigation/launch/myagv_slam_laser.launch)

is intended to combine:

- gmapping
- base and LiDAR bringup
- RViz

### Navigation on a saved map

The launch file:

- [navigation_active.launch](/Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/myagv_ros/src/myagv_navigation/launch/navigation_active.launch)

is intended to combine:

- `map_server`
- AMCL
- `move_base`
- RViz

## What This Stack Does Well

This codebase already gives you a workable robot runtime for:

- moving the AGV from ROS
- reading onboard motion feedback
- publishing LiDAR scans
- running local 2D mapping
- running localization and navigation against a saved map

That makes it a useful base for dataset collection on the robot itself.

## What This Stack Does Not Yet Provide

For a distributed SLAM dataset collection project, some important capabilities are not present here as first-class components.

Notably absent are:

- rosbag session management
- dataset recording launch files
- timestamp synchronization across robots
- robot ID and namespace management for multi-robot operation
- explicit inter-robot communication or map exchange
- dataset metadata export
- sensor calibration management
- collection orchestration scripts

So this folder should be seen as the onboard sensing and motion layer, not the full distributed collection pipeline.

## Important Implementation Notes

There are a few code-level details worth knowing before relying on this stack heavily.

- The AGV base serial port is hardcoded to `/dev/ttyACM0` in the odometry code.
- The LiDAR port is set separately in the launch file, currently `/dev/ttyAMA0`.
- The odometry code integrates pose locally from returned velocity data rather than using a more formal fused estimator in the active launch path.
- `robot_pose_ekf` is present in the repository but is not clearly wired into the main runtime.
- The stack appears targeted at ROS 1 and catkin.

## Known Issues In The Current Code

These are useful to know if you plan to run or extend the stack:

- `myagv_slam_laser.launch` contains a stray quote and is malformed.
- `myagv_teleop.py` has an invalid exception handler using an undefined `e`.
- `myAGV.cpp` declares `execute()` as returning `bool` but does not return a value.
- serial ports are hardcoded rather than parameterized
- parts of the odometry packet parsing are fragile and should be reviewed before depending on them for high-quality datasets

## Recommended Mental Model

If you want one simple way to think about this folder, use this:

1. `myagv_odometry` talks to the robot base.
2. `ydlidar_ros_driver` talks to the laser.
3. `myagv_teleop` produces manual commands.
4. `myagv_navigation` launches mapping and navigation on top of those lower-level streams.
5. everything else is supporting or vendored code.

## If You Extend This For Dataset Collection

The next logical additions would be:

- a dedicated data collection launch file
- a standard topic list for recording
- rosbag naming and session metadata conventions
- parameterized serial device configuration
- multi-robot namespacing
- time synchronization and calibration documentation
- health checks before starting a collection run

## Summary

This folder is the AGV-side ROS runtime stack. Its job is to make the robot move, publish its state, publish LiDAR data, and support local SLAM/navigation.

It is an important foundation for distributed SLAM dataset collection, but it is not yet the full collection system.
