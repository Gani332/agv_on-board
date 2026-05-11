# Offline Swarm-SLAM

This folder is the standalone offline Swarm-SLAM workspace for the AGV bags. It
can be moved out of `agv_on-board` as one directory.

Swarm-SLAM itself is used through Docker from the upstream repository. The local
`overlay/agv_swarmslam_tools` package only provides AGV bag playback,
visualization, and optional AprilTag-derived loop-closure injection.

## Folder Layout

```text
offline_swarmslam/
├── scripts/                  Docker build, ROS1->ROS2 conversion, run scripts
├── overlay/agv_swarmslam_tools/
│   ├── launch/               AGV offline launch files
│   ├── config/               Swarm-SLAM RGB-D config for AGV bags
│   ├── rviz/                 RViz layout, preconfigured for robot 0-4
│   └── agv_swarmslam_tools/  Visualization and optional tag constraint nodes
├── downloaded_bags/          Local ROS 1 bags, ignored by git
├── offline_slam_data/        Converted ROS 2 bags, ignored by git
├── models/                   CosPlace model, ignored by git
└── results/                  Swarm-SLAM/tag outputs, ignored by git
```

## Current Two-Robot Pass1 Run

Run from this folder:

```bash
cd /Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/agv_on-board/offline_swarmslam
FORCE=true bash scripts/convert_bags_to_ros2.sh
python3 scripts/analyze_apriltags.py --tag-size 0.10 --ids 1 --duplicate-id-other-robot --out-dir results/apriltags_pass1 --bag robotA=downloaded_bags/scenario1_square_20260506/robotA/robotA_scenario1_square_pass1.bag --bag robotB=downloaded_bags/scenario1_square_20260506/robotB/robotB_scenario1_square_pass1.bag
python3 scripts/estimate_tag_alignment.py
bash scripts/download_cosplace_model.sh
bash scripts/build_swarmslam_image.sh
RVIZ=true DASHBOARD=false PLAYBACK_RATE=1 bash scripts/run_swarmslam_rgbd.sh
open vnc://localhost:5900
```

The VNC password is:

```text
agv
```

`APRILTAG_GRAPH_CONSTRAINTS=auto` is the default. It enables the current
two-robot AprilTag constraint file only when running two robots and
`results/apriltags_pass1/robotA_to_robotB_alignment.json` exists.

## Five-Robot Bag Run

The runner can now launch any number of converted robot bags. Convert with
stable labels, then run with the same label order:

```bash
cd /path/to/offline_swarmslam
FORCE=true BAGS='robot0=/path/to/robot0.bag,robot1=/path/to/robot1.bag,robot2=/path/to/robot2.bag,robot3=/path/to/robot3.bag,robot4=/path/to/robot4.bag' bash scripts/convert_bags_to_ros2.sh
bash scripts/download_cosplace_model.sh
bash scripts/build_swarmslam_image.sh
ROBOT_LABELS=robot0,robot1,robot2,robot3,robot4 RVIZ=true DASHBOARD=false PLAYBACK_RATE=1 bash scripts/run_swarmslam_rgbd.sh
open vnc://localhost:5900
```

RViz is preconfigured for robot 0-4 paths and sparse RGB-D keyframe maps. Extra
robots can still publish; add their topics manually in RViz if needed.

## Readiness Notes

Swarm-SLAM multi-robot playback is ready for properly named/converted bags. For
tomorrow's five-robot concentric-circle run, the necessary recording-side setup
is:

```text
unique robot names in bag filenames
unique AprilTag IDs on the robots
chrony-synced clocks
stable RGB-D, aligned depth, odom, tf, tf_static, scan, and /imu streams
recorded or measured tag mount notes for each robot
```

The current AprilTag graph-constraint helper is pairwise and was built for the
two-robot duplicate-ID pass1 dataset. For five robots with proper unique IDs,
the raw Swarm-SLAM run is ready, but automatic N-robot AprilTag graph injection
still needs a pairwise tag-mount configuration step before it should be treated
as plug-and-play.

## Bag Topic Mapping

Each converted bag is remapped by robot index:

| ROS 1 bag topic | Robot i ROS 2 topic |
| --- | --- |
| `/camera/color/image_raw` | `/ri/color/image_raw` |
| `/camera/color/camera_info` | `/ri/color/camera_info` |
| `/camera/aligned_depth_to_color/image_raw` | `/ri/aligned_depth_to_color/image_raw` |
| `/odom` | `/ri/odom` |

Swarm-SLAM expects robot IDs starting from `0`; `ROBOT_LABELS` only controls the
host folder names and launch order.

## Operational Notes

Keep `PLAYBACK_RATE=1` for real analysis. Higher playback rates are useful only
for launch smoke tests and can starve the RGB-D queues on a laptop.

The camera is inverted in the current pass1 dataset, so RViz and the optional
dashboard rotate RGB display images by 180 degrees for viewing only. Swarm-SLAM
receives the raw RGB-D streams.

`Could not compute transformation` means a candidate visual inter-robot loop
closure did not have enough usable features. That is expected in some frames and
is separate from bag/logging failure.

The Docker image defaults to ROS 2 Humble and builds as `linux/amd64`. On Apple
Silicon this runs through Docker Desktop emulation.
