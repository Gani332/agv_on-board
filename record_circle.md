# Scenario 1 - Concentric Circles

Use the two-terminal workflow for data collection. This keeps recording/bringup
separate from motion control, which is safer and easier to stop/debug.

Live AprilTag detection is off by default. Detect tags offline from the recorded
RGB images if needed.

## Robot Assignments

```text
agv1 -> radius 0.50 m -> starts at T0 +   0 s
agv2 -> radius 0.75 m -> starts at T0 +  30 s
agv3 -> radius 1.00 m -> starts at T0 +  60 s
agv4 -> radius 1.25 m -> starts at T0 +  90 s
agv5 -> radius 1.50 m -> starts at T0 + 120 s
```

Place each robot on its ring and point it tangentially clockwise before arming
motion.

## One-Robot Smoke Test

Terminal 1 - start recording and bringup:

```bash
ssh ubuntu@10.23.118.99
cd ~/slam_project
export REQUIRE_GT=false
export REQUIRE_IMU=true
export ENABLE_APRILTAG=false
bash scripts/logging/start_session.sh agv1 s1_circle_smoke
```

Wait until Terminal 1 prints that sensors are live and rosbag has started.

Terminal 2 - run 60 seconds of motion:

```bash
ssh ubuntu@10.23.118.99
cd ~/slam_project
source /opt/ros/noetic/setup.bash
[ -f ~/slam_project/myagv_ros/devel/setup.bash ] && source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python scripts/logging/drive_circle.py --radius 0.50 --linear 0.16 --duration 60 --no-prompt --verbose
```

Stop Terminal 1 with `Ctrl+C` after motion finishes.

## Full One-Robot Run

Use the same two terminals, but change Terminal 1 scenario name and Terminal 2
duration:

Terminal 1:

```bash
ssh ubuntu@10.23.118.99
cd ~/slam_project
export REQUIRE_GT=false
export REQUIRE_IMU=true
export ENABLE_APRILTAG=false
bash scripts/logging/start_session.sh agv1 s1_concentric_1robot_r1
```

Terminal 2:

```bash
ssh ubuntu@10.23.118.99
cd ~/slam_project
source /opt/ros/noetic/setup.bash
[ -f ~/slam_project/myagv_ros/devel/setup.bash ] && source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python scripts/logging/drive_circle.py --radius 0.50 --linear 0.16 --duration 600 --no-prompt --verbose
```

## Multi-Robot Run With Reliable Staggering

Start Terminal 1 recording on every robot first. Wait until every robot is
recording before arming motion.

Pick one shared start time on your laptop:

```bash
T0=$(($(date +%s)+300))
echo $T0
```

Use the printed value as `<T0>` in every motion terminal.

### Robot 1

Terminal 1:

```bash
ssh ubuntu@10.23.118.99
cd ~/slam_project
export REQUIRE_GT=false
export REQUIRE_IMU=true
export ENABLE_APRILTAG=false
bash scripts/logging/start_session.sh agv1 s1_concentric_2robot_r1
```

Terminal 2:

```bash
ssh ubuntu@10.23.118.99
cd ~/slam_project
source /opt/ros/noetic/setup.bash
[ -f ~/slam_project/myagv_ros/devel/setup.bash ] && source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python scripts/logging/drive_circle.py --radius 0.50 --linear 0.16 --duration 600 --start-at-epoch <T0> --start-delay 0 --no-prompt --verbose
```

### Robot 2

Terminal 1:

```bash
ssh ubuntu@<robot2-ip>
cd ~/slam_project
export REQUIRE_GT=false
export REQUIRE_IMU=true
export ENABLE_APRILTAG=false
bash scripts/logging/start_session.sh agv2 s1_concentric_2robot_r2
```

Terminal 2:

```bash
ssh ubuntu@<robot2-ip>
cd ~/slam_project
source /opt/ros/noetic/setup.bash
[ -f ~/slam_project/myagv_ros/devel/setup.bash ] && source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python scripts/logging/drive_circle.py --radius 0.75 --linear 0.16 --duration 600 --start-at-epoch <T0> --start-delay 30 --no-prompt --verbose
```

For 3+ robots, use the assignment table above and the same pattern:

```text
agv3: --radius 1.00 --start-delay 60
agv4: --radius 1.25 --start-delay 90
agv5: --radius 1.50 --start-delay 120
```

Do not manually time the starts. Arm every motion terminal before `T0`; the
script handles the 30 second staggering.

## Stop Motion

First try `Ctrl+C` in the motion terminal.

If that does not stop the robot, SSH into that robot and run:

```bash
pkill -f 'drive_circle.py' 2>/dev/null || true
source /opt/ros/noetic/setup.bash
[ -f ~/slam_project/myagv_ros/devel/setup.bash ] && source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
for i in $(seq 1 20); do timeout 1 rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true; sleep 0.05; done
python3 -c "import serial,time; ser=serial.Serial('/dev/ttyACM0',115200,timeout=0.1); pkt=bytes([115,115,128,128,128,128]); [ser.write(pkt) or ser.flush() or time.sleep(0.05) for _ in range(60)]; ser.close(); print('DIRECT STOP SENT')"
```

Use the hardware stop/power switch immediately if the robot still moves.

## After Recording

On each robot:

```bash
cd ~/slam_project
python3 scripts/logging/validate_bag.py $(ls -t ~/agv_data/*.bag | head -1)
python scripts/logging/audit_bag_fast.py $(ls -t ~/agv_data/*.bag | head -1)
```

Bags and manifests are written to `~/agv_data`.

## Optional Checks

General sensor readiness, without live AprilTag:

```bash
cd ~/slam_project
bash scripts/diagnostics/robot_readiness_check.sh
```

Separate live AprilTag smoke test only:

```bash
cd ~/slam_project
CHECK_APRILTAG=true bash scripts/diagnostics/robot_readiness_check.sh
```
