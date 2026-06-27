# AGV On-Board Stack

Robot-side ROS 1 stack for AGV data collection in the multi-robot SLAM dataset project.

The goal of this repo is repeatable deployment: clone or pull it on a robot, run one setup script, then collect bags with a single session command.

## 🚀 Quick Start

On a new robot:

### 0. Wi-Fi

For fresh SD cards and hotspot setup, use the netplan/cloud-init template in
`ros2_wifi_setup/wifi_setup.txt`. Replace the Wi-Fi SSID/password placeholders
there before flashing or applying network config.

Do not repeatedly start `wpa_supplicant -B` and `dhclient` by hand; that can
leave duplicate Wi-Fi/DHCP processes and make SSH, apt, git, and ROS traffic
unstable.

### 1. Installation
On a fresh or updated robot, use one of the following methods to retrieve the stack.

**Option A: Standard Clone (Try this first)**
```bash
git clone --depth 1 https://github.com/Gani332/agv_on-board.git ~/slam_project
cd ~/slam_project
bash scripts/setup_robot.sh
```

**Option B: Download Zip**
```bash
# Download the repository as a zip file
wget https://github.com/Gani332/agv_on-board/archive/refs/heads/main.zip

# Unzip the file
unzip main.zip

# Rename the resulting folder to slam_project
mv agv_on-board-main slam_project

# Remove the zip file to save space
rm main.zip

bash scripts/setup_robot.sh
```


On an updated robot:

```bash
cd ~/slam_project
git pull
bash scripts/setup_robot.sh
```

`setup_robot.sh` installs expected system dependencies by default, including
`chrony`, `apriltag_ros`, ROS message packages, rosbag, TF, and build tools. Use
`bash scripts/setup_robot.sh --skip-system` only when the robot is already
provisioned or has no internet access.

### RealSense Dependency

The legacy ROS 1 stack in `agv_ws` was validated with `librealsense2` v2.57.6.
Avoid mixing that workspace with older ROS Melodic apt SDK headers
(`ros-melodic-librealsense2`, commonly v2.50.0). On a correctly provisioned
legacy robot, `roslaunch agv_bringup bringup.launch` prints:

```text
Built with LibRealSense v2.57.6
Running with LibRealSense v2.57.6
```

For the ROS 2 dataset robots, the authoritative camera standard is enforced by
`configs/robot_doctor_dataset_gate.json`:

```text
D455 firmware:                  5.17.0.10
standalone librealsense tools:  2.58.1
RealSense ROS driver:           realsense2_camera 4.57.7
RealSense ROS node runtime:     LibRealSense 2.57.7
RGB-D stream gate:              640x480 at 15 Hz
USB gate:                       USB 3.x / 5000 Mb/s
```

The ROS 2 setup path also installs D455-specific udev rules for USB autosuspend
and `uvcvideo` binding. This covers the failure where `lsusb` sees the D455 at
USB3 but RealSense tools report no device because the Video interfaces are
unbound.

Check the installed SDK headers with:

```bash
pkg-config --modversion realsense2
```

Start a data collection session:

```bash
cd ~/slam_project
export REQUIRE_GT=false
export REQUIRE_IMU=true
bash scripts/logging/start_session.sh agv1 square_manual
```

Before collecting publishable data, run the unified diagnostic gate:

```bash
cd ~/slam_project
bash scripts/diagnostics/robot_doctor.sh agv1 --profile preflight
```

For strict pre-run collection readiness, use the dataset gate and keep the
generated `~/agv_data/diagnostics/<robot>_<timestamp>/summary.json` with the
run notes. This proves the robot side before motion, but it is not a post-run
bag audit:

```bash
bash scripts/diagnostics/robot_doctor.sh <robot_name> \
  --config configs/robot_doctor_dataset_gate.json \
  --mocap-topic /optitrack/rigid_bodies/<rigid_body_name> \
  --cmd-topic /<robot_name>/cmd_vel \
  --strict-ops \
  --confirm-mechanical \
  --confirm-mocap \
  --confirm-anchors
```

After recording, run the post-run dataset audit with the bag path:

```bash
bash scripts/diagnostics/robot_doctor.sh <robot_name> \
  --config configs/robot_doctor_dataset_gate.json \
  --require-bag \
  --bag ~/agv_data/<bag_dir_or_bag_file> \
  --mocap-topic /optitrack/rigid_bodies/<rigid_body_name> \
  --cmd-topic /<robot_name>/cmd_vel \
  --strict-ops \
  --confirm-mechanical \
  --confirm-mocap \
  --confirm-anchors
```

The diagnostic report maps every failure to the failure tree in
`robot_failure_modes_v3.png`: robot platform, robot data stack, or experiment
dataset. Full instructions are in
[`docs/ROBOT_DIAGNOSTIC_PIPELINE.md`](docs/ROBOT_DIAGNOSTIC_PIPELINE.md).
If the report raises `1.2 d455_physical_swap_evidence`, complete the generated
`operator_d455_swap_checklist.md` before deciding whether the fault follows the
camera, cable, robot USB3 port, or power path.

After running diagnostics across several robots, use the strict fleet audit
before treating a collection as publishable:

```bash
python3 scripts/diagnostics/fleet_doctor_summary.py \
  --strict-fleet \
  diagnostic_reports/agv*/*/summary.json
```

For a completed dataset run, audit copied reports, bags, and manifests together:

```bash
python3 scripts/diagnostics/dataset_run_audit.py \
  --report 'diagnostic_reports/agv*/agv*/summary.json' \
  --bag '/path/to/copied/bags/*' \
  --manifest '/path/to/copied/manifests/*_manifest.yaml' \
  --mocap-topic /optitrack/rigid_bodies/<rigid_body_name> \
  --cmd-topic /<robot_name>/cmd_vel \
  --require-gt \
  --require-imu \
  --strict
```

This is the final publishability gate: it validates report evidence, bag
rates/gaps, manifest completion, and that the copied report/bag/manifest
artifacts belong to the same robot, scenario, and session identity.

For a lab host list, the equivalent remote wrapper form is:

```bash
SSH_PASS=ubuntu bash scripts/diagnostics/run_fleet_doctor_remote.sh hosts.txt --strict-fleet -- \
  --config configs/robot_doctor_dataset_gate.json \
  --profile preflight
```

Drive manually in another terminal:

```bash
ssh ubuntu@<robot-ip>
source /opt/ros/noetic/setup.bash
source ~/slam_project/myagv_ros/devel/setup.bash
rosrun myagv_teleop myagv_teleop.py
```

Or run the validated OptiTrack mocap-feedback 1x1 square:

```bash
ssh ubuntu@<robot-ip>
cd ~/slam_project
source /opt/ros/noetic/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python3 scripts/logging/drive_square.py --yes
```

The older odom-feedback square helper is still available for non-mocap tests:

```bash
python scripts/logging/drive_square_odom.py --side 0.75 --linear 0.22 --angular 0.28 --cycles 1
```

Or run a concentric-circle S1 motion test:

```bash
ssh ubuntu@<robot-ip>
cd ~/slam_project
source /opt/ros/noetic/setup.bash
source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python scripts/logging/drive_circle.py --radius 1.00 --linear 0.16 --duration 60 --no-prompt --verbose
```

Stop recording with `Ctrl+C`. Bags and manifests are written to `~/agv_data`.

## Next Lab Visit Commands

Use separate terminals on the robot. Keep the robot on the floor with clear space before running motion scripts.

Terminal 1, record a straight-line bag:

```bash
ssh ubuntu@<robot-ip>
cd ~/slam_project
export REQUIRE_GT=false
export REQUIRE_IMU=true
bash scripts/logging/start_session.sh agv1 straight_slow
```

Terminal 2, drive the straight line:

```bash
ssh ubuntu@<robot-ip>
cd ~/slam_project
source /opt/ros/noetic/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python3 scripts/logging/drive_mocap_straight.py \
  --pose-topic /gt/agv1/pose \
  --distance 1.0 \
  --linear 0.12 \
  --timeout 12 \
  --line-yaw-offset-deg 90 \
  --max-lateral-error 0.15 \
  --yes \
  --verbose
```

Stop Terminal 1 with `Ctrl+C`, then validate:

```bash
python3 scripts/logging/validate_bag.py $(ls -t ~/agv_data/*.bag | head -1)
python scripts/logging/audit_bag_fast.py $(ls -t ~/agv_data/*.bag | head -1)
```

Then record a square bag:

```bash
ssh ubuntu@<robot-ip>
cd ~/slam_project
export REQUIRE_GT=false
export REQUIRE_IMU=true
bash scripts/logging/start_session.sh agv1 square_slow
```

In another terminal:

```bash
ssh ubuntu@<robot-ip>
cd ~/slam_project
source /opt/ros/noetic/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python3 scripts/logging/drive_square.py --yes
```

Stop recording and validate again:

```bash
python3 scripts/logging/validate_bag.py $(ls -t ~/agv_data/*.bag | head -1)
python scripts/logging/audit_bag_fast.py $(ls -t ~/agv_data/*.bag | head -1)
```

## OptiTrack Mocap Driving

The current AGV 1 mocap setup uses the Motive rigid body `orkar_agv1`.
The ROS 1 control topic is `/gt/agv1/pose`, with a convenience relay on
`/optitrack/rigid_bodies/orkar_agv1`. Both are `geometry_msgs/PoseStamped`.
The calibrated forward direction is rigid-body yaw plus 90 degrees; those
defaults are stored in `agv_ws/src/agv_bringup/calibration/optitrack_agv1.yaml`.

Before moving, verify that the mocap source is live:

```bash
source /opt/ros/noetic/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_IP ROS_HOSTNAME

rostopic echo -n 1 /gt/agv1/pose
rostopic echo -n 1 /optitrack/rigid_bodies/orkar_agv1
```

Drive 1 m forward using mocap feedback:

```bash
python3 scripts/logging/drive_mocap_straight.py \
  --pose-topic /gt/agv1/pose \
  --distance 1.0 \
  --linear 0.12 \
  --timeout 12 \
  --line-yaw-offset-deg 90 \
  --max-lateral-error 0.15 \
  --yes \
  --verbose
```

Drive 6 m forward quickly, when the arena is clear:

```bash
python3 scripts/logging/drive_mocap_straight.py \
  --pose-topic /gt/agv1/pose \
  --distance 6.0 \
  --linear 0.20 \
  --timeout 45 \
  --line-yaw-offset-deg 90 \
  --max-lateral-error 0.18 \
  --yes \
  --verbose
```

Drive the validated 1x1 m square profile:

```bash
python3 scripts/logging/drive_square.py --yes
```

On a laptop with Pixi installed, the ROS 2 and NatNet inspection helpers are:

```bash
pixi run ros2-topic-list-types
pixi run ros2-echo-agv1
pixi run mocap-watch-agv1
```

If `/optitrack/rigid_bodies/orkar_agv1` exists but `rostopic echo` prints
nothing, the relay is probably still registered while `vrpn_client_node` is no
longer forwarding from Motive. Restart the mocap source and relays:

```bash
source /opt/ros/noetic/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_IP ROS_HOSTNAME

rosnode cleanup
pkill -f 'vrpn_client_ros sample.launch server:=192.168.50.200' || true
pkill -f 'vrpn_client_node' || true
pkill -f 'topic_tools.*relay.*/vrpn_client_node/orkar_agv1/pose' || true

nohup bash -lc 'source /opt/ros/noetic/setup.bash; source ~/slam_project/agv_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; unset ROS_IP ROS_HOSTNAME; roslaunch vrpn_client_ros sample.launch server:=192.168.50.200' \
  > ~/.ros/vrpn_mocap_drive.log 2>&1 &
sleep 4
nohup bash -lc 'source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://localhost:11311; unset ROS_IP ROS_HOSTNAME; rosrun topic_tools relay /vrpn_client_node/orkar_agv1/pose /gt/agv1/pose' \
  > ~/.ros/relay_gt_agv1_pose.log 2>&1 &
nohup bash -lc 'source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://localhost:11311; unset ROS_IP ROS_HOSTNAME; rosrun topic_tools relay /vrpn_client_node/orkar_agv1/pose /optitrack/rigid_bodies/orkar_agv1' \
  > ~/.ros/relay_optitrack_orkar_agv1.log 2>&1 &
```

## What Is Production

Use these paths for normal robot operation:

```text
scripts/setup_robot.sh                     Build/check workspaces after clone or pull
scripts/diagnostics/apply_robot_doctor_fix.sh Targeted, dry-run-first remediation for known findings
scripts/diagnostics/dataset_run_audit.py     Final post-run audit across reports, bags, and manifests
scripts/diagnostics/robot_doctor.sh        Unified robot readiness/failure-classification gate
scripts/diagnostics/fleet_doctor_summary.py Compare robot_doctor summaries across a fleet
scripts/diagnostics/robot_doctor_selftest.py No-hardware regression tests for diagnostics
scripts/diagnostics/run_fleet_doctor_remote.sh Deploy/run diagnostics across a host list
scripts/diagnostics/run_robot_doctor_remote.sh Deploy/run diagnostics on a robot over SSH
scripts/diagnostics/validate_robot_doctor_report.py Validate summary.json consistency
docs/ROBOT_DEBUG_PIPELINE_COVERAGE_AUDIT.md Answered coverage audit for D455/ROS2/Chrony gaps
scripts/setup_robot_ros2.sh                ROS 2 robot provisioning with RealSense/tooling gate
scripts/logging/start_session.sh           One-command bringup + rosbag + manifest
scripts/logging/validate_bag.py            Full post-run publishability check
scripts/logging/validate_ros2_bag.py       ROS 2 rosbag2 .db3/.mcap publishability check
scripts/logging/audit_bag_fast.py          Fast topic/rate/gap/sync audit
scripts/logging/drive_straight.py          Odom-bounded straight-line dataset helper
scripts/logging/drive_mocap_straight.py    OptiTrack mocap-feedback straight-line helper
scripts/logging/drive_mocap_square.py      OptiTrack mocap-feedback square helper
scripts/logging/drive_square.py            Validated OptiTrack 1x1 square wrapper
scripts/logging/drive_square_odom.py       Archived odom-feedback square motion helper
scripts/logging/drive_circle.py            Odom-feedback circular motion helper for S1
scripts/logging/drive_forward_back.py      Odom-bounded smoke-test motion helper
scripts/mocap/                             Direct NatNet discovery/publisher fallback tools
agv_ws/src/agv_bringup/launch/bringup.launch
agv_ws/src/agv_bringup/launch/logging.launch
agv_ws/src/agv_bringup/launch/aruco.launch
agv_ws/src/agv_bringup/launch/aruco_bringup.launch
agv_ws/src/agv_bringup/launch/aruco_test.launch
agv_ws/src/agv_bringup/launch/apriltag.launch
agv_ws/src/agv_bringup/calibration/
```

Diagnostic and hardware-investigation scripts live under:

```text
scripts/diagnostics/
```

## Repository Layout

```text
agv_on-board/
├── myagv_ros/                  Vendor AGV base, odometry, teleop, LiDAR ROS packages
├── agv_ws/
│   └── src/
│       ├── agv_bringup/        Dataset launch files, TFs, calibration, tag config
│       └── realsense-ros/      Vendored RealSense ROS wrapper
├── scripts/
│   ├── setup_robot.sh          Build/check robot after clone or pull
│   ├── benchmarking/           Trajectory evaluation, alignment, report assets
│   ├── calibration/            Calibration extraction and static tests
│   ├── diagnostics/            Hardware debug scripts
│   ├── logging/                Recording, validation, motion helpers
│   ├── scenarios/              Multi-robot scenario launch helpers
│   ├── orbslam3/               ORB-SLAM3 benchmark wrappers
│   ├── rtabmap/                RTAB-Map benchmark wrappers
│   ├── slam_toolbox/           SLAM Toolbox benchmark wrappers
│   ├── cartographer/           Cartographer benchmark wrappers
│   ├── swarmslam/              Swarm-SLAM benchmark wrappers
│   └── covins/                 COVINS-G benchmark wrappers
├── Report/                     Report source, references, curated figures
├── docs/                       SOPs and dataset checklists
├── drivers/                    Vendored third-party SDK/reference code
└── configs/                    RViz configs
```

External algorithm checkouts, raw bags, public datasets, generated benchmark
outputs, and benchmarking wrappers are local workspace artefacts and are
intentionally kept out of this robot deployment repository. Benchmarking code
lives in the separate `ORKAR_benchmarking` repository.

## Robot Runtime

Source order matters:

```bash
source /opt/ros/noetic/setup.bash
source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
```

Manual bringup without recording:

```bash
roslaunch agv_bringup bringup.launch enable_imu:=false
```

Current ArUco marker test, using the 15 cm `DICT_ARUCO_ORIGINAL` marker id `503`:

```bash
roslaunch agv_bringup aruco_bringup.launch target_id:=503 marker_size:=0.15 publish_image:=false
```

This runs normal robot bringup plus the ArUco detector. It prints detections and publishes the target marker pose on `/aruco/target_pose` as `geometry_msgs/PoseStamped`. The pose frame is the RealSense optical frame, where `x` is right, `y` is down, and `z` is forward.

Camera-only ArUco smoke test:

```bash
roslaunch agv_bringup aruco_test.launch target_id:=503 marker_size:=0.15 publish_image:=false
```

AprilTag detector only, for the project `tag36h11` markers with 100 mm code size. For robust live multi-robot work, give each robot a unique tag ID; the current pass1 offline bags have duplicate ID 1 tags and handle ownership with an offline two-robot rule:

```bash
roslaunch agv_bringup apriltag.launch
```

Override the tag file when testing a different marker layout:

```bash
roslaunch agv_bringup apriltag.launch tags_file:=$(rospack find agv_bringup)/config/tags.yaml
```

Production recording:

```bash
bash scripts/logging/start_session.sh <robot_name> <scenario_name>
```

S1 concentric-circle command pattern:

```bash
python scripts/logging/drive_circle.py --radius <0.50|0.75|1.00|1.25|1.50> --linear 0.16 --duration 600 --start-delay <0|30|60|90|120> --no-prompt --verbose
```

Place each robot on its ring and point it tangentially before starting. The
default direction is clockwise.

Normal scenario recording keeps live marker detection off so the robot
prioritises stable RGB-D, LiDAR, odom, TF, and base `/imu` logging. If a run
explicitly needs the live ArUco detector, set `ENABLE_ARUCO=true` before
starting the session; otherwise detect markers offline from the recorded images
or run `aruco_bringup.launch` as a separate smoke test.

`start_session.sh` writes:

```text
~/agv_data/<robot>_<scenario>_<timestamp>.bag
~/agv_data/<robot>_<scenario>_<timestamp>_manifest.yaml
~/agv_data/<robot>_<scenario>_<timestamp>_chrony.txt
```

It records with `rosbag --buffsize=2048 --lz4`, which was validated on the live robot for RGB-D + LiDAR recording without buffer overflow.

## Recorded Topics

Default robot bag topics:

```text
/scan
/odom
/cmd_vel
/tf
/tf_static
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/camera_info
/camera/aligned_depth_to_color/image_raw
/camera/aligned_depth_to_color/camera_info
/camera/extrinsics/depth_to_color
/imu
/diagnostics
```

Optional topics are included when available:

```text
/aruco/target_pose
/tag_detections
/camera/imu
/camera/accel/sample
/camera/gyro/sample
/camera/accel/imu_info
/camera/gyro/imu_info
/gt/agv1/pose
/optitrack/rigid_bodies/orkar_agv1
/mocap
```

Use:

```bash
export REQUIRE_GT=true
export MOCAP_TOPIC=/gt/agv1/pose
```

when ground truth must be present in the same ROS graph. If ground truth is
recorded separately, keep `REQUIRE_GT=false` and save chrony status on both
machines.

## Current Validated Baseline

Live robot bag checked on 2026-05-05, Robot 2, normal RGB-D logging with base
`/imu` and no detector:

```text
duration: 45.0 s
/scan: 17.67 Hz
/odom: 12.63 Hz
/imu: 12.65 Hz
/camera/color/image_raw: 15.01 Hz
/camera/aligned_depth_to_color/image_raw: 15.01 Hz
/tf: 111.56 Hz
camera color/depth sync: 0.00 ms median, 0.00 ms max
diagnostics: 0 warnings, 0 errors
validator: FAIL 0
overall audit: PASS
```

This is good enough for robot-only Week 1 SLAM smoke validation.

Known limitations:

```text
Dataset /imu is published from the AGV base MCU through myagv_odometry_node. The RealSense D455 camera IMU remains disabled by default because the current D455/wrapper/firmware stack publishes camera IMU in IMU-only mode, but not reliably while RGB-D video is active.
Live ArUco detection is disabled by default during dataset recording to protect RGB-D throughput. Enable it only with ENABLE_ARUCO=true. The current test marker is DICT_ARUCO_ORIGINAL id 503 with 0.15 m side length.
Ground truth is optional by default because PhaseSpace may be recorded separately on a chrony-synced machine.
```

## Transform Tree

```text
odom
└── base_footprint
    ├── base_link          static alias, colocated
    ├── laser_frame        z=0.100 m measured
    ├── imu_link           base MCU IMU, colocated
    └── camera_link        CAD extrinsic from original mount
        ├── camera_color_frame
        ├── camera_depth_frame
        └── camera_aligned_depth_to_color_frame
```

Important static transforms:

```text
base_footprint -> base_link:
  xyz=(0, 0, 0), rpy=(0, 0, 0)

base_footprint -> laser_frame:
  xyz=(0, 0, 0.100), rpy=(0, 0, 0)

base_footprint -> imu_link:
  xyz=(0, 0, 0), rpy=(0, pi, pi)

base_footprint -> camera_link:
  xyz=(-0.132025, 0.000153, 0.187925)
  rpy=(pi/2, -0.007906, -pi/2)
```

## Validation

Fast audit:

```bash
cd ~/slam_project
source /opt/ros/noetic/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python scripts/logging/audit_bag_fast.py ~/agv_data/<bag>.bag
```

Full validator:

```bash
python3 scripts/logging/validate_bag.py ~/agv_data/<bag>.bag
```

Exit codes:

```text
0 = pass
1 = fail
2 = warning
```

Expected warnings for the current robot-only setup:

```text
ground truth missing, unless REQUIRE_GT=true
camera IMU topics missing are expected; base /imu should be present by default
target marker topics may be empty when no marker is visible
```

## Copy Bags To Laptop

From the laptop:

```bash
mkdir -p /Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/dataset/week1
scp ubuntu@<robot-ip>:/home/ubuntu/agv_data/*.bag \
  /Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/dataset/week1/
```

## Clean Robot Run Data

On the robot:

```bash
rm -f ~/agv_data/*.bag ~/agv_data/*.bag.active ~/agv_data/*_manifest.yaml ~/agv_data/*_chrony.txt
```

## Hardware

```text
AGV base controller: /dev/ttyACM0
YDLiDAR X2:          /dev/ttyAMA0
RealSense D455:      USB 3.x, RGB-D 640x480 at 15 Hz
ROS2 camera gate:    firmware 5.17.0.10, tools 2.58.1, driver 4.57.7, node SDK 2.57.7
Base IMU topic:      /imu at about 12.6 Hz, frame_id=imu_link
```

## Scaling To More Robots

For each robot:

1. Clone/pull this repo to `~/slam_project`.
2. Assign a stable robot name, e.g. `agv100`, `agv101`, `agv102`.
3. On ROS 2 robots, run `SUDO_PASSWORD=ubuntu bash scripts/setup_robot_ros2.sh <robot_name>`.
4. Run `robot_doctor` with `configs/robot_doctor_dataset_gate.json`.
5. Fix any `FAIL`; resolve or explicitly document every `WARN`.
6. If D455 physical-path failures persist after USB reset, complete the camera/cable/host-port A/B swap checklist.
7. Record with `bash scripts/logging/start_session.sh <robot_name> <scenario>`.
8. Keep robot bags, manifests, and `robot_doctor` summaries with the same robot/scenario/timestamp convention; `dataset_run_audit.py` fails mismatched artifact sets.
9. Before each run, confirm chrony on robot and mocap machines if ground truth is recorded separately.

Offline Swarm-SLAM analysis wrappers live in `scripts/swarmslam/`. The upstream
Swarm-SLAM checkout and generated results should remain outside git in the
ignored local workspace directories.
