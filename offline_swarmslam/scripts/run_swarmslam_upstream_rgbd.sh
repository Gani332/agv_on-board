#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
IMAGE="${IMAGE:-agv-swarmslam:$ROS_DISTRO}"
PLATFORM="${PLATFORM:-linux/amd64}"
DATA_DIR="${DATA_DIR:-$ROOT_DIR/offline_slam_data}"
RESULTS_DIR="${RESULTS_DIR:-$ROOT_DIR/results/upstream}"
MODEL_DIR="${MODEL_DIR:-$ROOT_DIR/models}"
MODEL_NAME="${MODEL_NAME:-ResNet18_64_cosplace.pth}"
PLAYBACK_RATE="${PLAYBACK_RATE:-0.5}"
SETTLE_SEC="${SETTLE_SEC:-60}"

ROBOT_A_ROS2_BAG="${ROBOT_A_ROS2_BAG:-/data/ros2/robotA}"
ROBOT_B_ROS2_BAG="${ROBOT_B_ROS2_BAG:-/data/ros2/robotB}"

if [[ ! -d "$DATA_DIR/ros2/robotA" || ! -d "$DATA_DIR/ros2/robotB" ]]; then
  echo "Converted ROS 2 bags are missing under $DATA_DIR/ros2." >&2
  echo "Run: FORCE=true bash scripts/convert_bags_to_ros2.sh" >&2
  exit 1
fi

if [[ ! -f "$MODEL_DIR/$MODEL_NAME" ]]; then
  echo "CosPlace model is missing: $MODEL_DIR/$MODEL_NAME" >&2
  echo "Run: bash scripts/download_cosplace_model.sh" >&2
  exit 1
fi

mkdir -p "$RESULTS_DIR"

docker rm -f agv-swarmslam-upstream >/dev/null 2>&1 || true

docker run --rm \
  --platform "$PLATFORM" \
  --name agv-swarmslam-upstream \
  -v "$DATA_DIR:/data:ro" \
  -v "$RESULTS_DIR:/results" \
  -v "$MODEL_DIR:/models:ro" \
  "$IMAGE" bash -lc "
    set -eo pipefail
    source /opt/ros/$ROS_DISTRO/setup.bash
    source /root/Swarm-SLAM/install/setup.bash

    CSLAM_SHARE=\$(python3 - <<'PY'
from ament_index_python.packages import get_package_share_directory
print(get_package_share_directory('cslam'))
PY
)
    mkdir -p \"\$CSLAM_SHARE/models\"
    ln -sf /models/$MODEL_NAME \"\$CSLAM_SHARE/models/resnet18_64.pth\"

    ros2 launch cslam_experiments cslam_rgbd.launch.py \
      namespace:=/r0 robot_id:=0 max_nb_robots:=2 \
      config_file:=realsense_rgbd.yaml &
    PID_R0=\$!

    ros2 launch cslam_experiments cslam_rgbd.launch.py \
      namespace:=/r1 robot_id:=1 max_nb_robots:=2 \
      config_file:=realsense_rgbd.yaml &
    PID_R1=\$!

    sleep 3

    ros2 bag play $ROBOT_A_ROS2_BAG --clock --rate $PLAYBACK_RATE --remap \
      /camera/color/image_raw:=/r0/color/image_raw \
      /camera/color/camera_info:=/r0/color/camera_info \
      /camera/aligned_depth_to_color/image_raw:=/r0/aligned_depth_to_color/image_raw \
      /odom:=/r0/odom &
    PID_BAG0=\$!

    ros2 bag play $ROBOT_B_ROS2_BAG --clock --rate $PLAYBACK_RATE --remap \
      /camera/color/image_raw:=/r1/color/image_raw \
      /camera/color/camera_info:=/r1/color/camera_info \
      /camera/aligned_depth_to_color/image_raw:=/r1/aligned_depth_to_color/image_raw \
      /odom:=/r1/odom &
    PID_BAG1=\$!

    wait \$PID_BAG0 \$PID_BAG1
    echo \"Bag playback finished. Waiting $SETTLE_SEC seconds for Swarm-SLAM to finish optimization/logging...\"
    sleep $SETTLE_SEC
    kill \$PID_R0 \$PID_R1 >/dev/null 2>&1 || true
  "
