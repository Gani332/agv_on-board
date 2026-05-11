#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
IMAGE="${IMAGE:-agv-swarmslam:$ROS_DISTRO}"
PLATFORM="${PLATFORM:-linux/amd64}"
DATA_DIR="${DATA_DIR:-$ROOT_DIR/offline_slam_data}"
RESULTS_DIR="${RESULTS_DIR:-$ROOT_DIR/results/upstream_official_viz}"
MODEL_DIR="${MODEL_DIR:-$ROOT_DIR/models}"
MODEL_NAME="${MODEL_NAME:-ResNet18_64_cosplace.pth}"
VIZ_SRC_DIR="${VIZ_SRC_DIR:-$ROOT_DIR/external/cslam_visualization}"
VNC_PORT="${VNC_PORT:-5900}"
VNC_PASSWORD="${VNC_PASSWORD:-agv}"
PLAYBACK_RATE="${PLAYBACK_RATE:-1}"
SETTLE_SEC="${SETTLE_SEC:-60}"
EARLY_SCREENSHOT_SEC="${EARLY_SCREENSHOT_SEC:-35}"
MID_SCREENSHOT_SEC="${MID_SCREENSHOT_SEC:-110}"
XVFB_WIDTH="${XVFB_WIDTH:-3200}"
XVFB_HEIGHT="${XVFB_HEIGHT:-2200}"

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

if [[ ! -f "$VIZ_SRC_DIR/package.xml" ]]; then
  echo "Official cslam_visualization checkout is missing: $VIZ_SRC_DIR" >&2
  echo "Run: git clone https://github.com/lajoiepy/cslam_visualization.git external/cslam_visualization" >&2
  exit 1
fi

mkdir -p "$RESULTS_DIR"

docker rm -f agv-swarmslam-upstream-viz >/dev/null 2>&1 || true

cleanup() {
  docker rm -f agv-swarmslam-upstream-viz >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "RViz VNC server will be available at vnc://localhost:$VNC_PORT"
echo "VNC password: $VNC_PASSWORD"
echo "Screenshots/logs will be written to: $RESULTS_DIR"

docker run --rm \
  -i \
  --platform "$PLATFORM" \
  --name agv-swarmslam-upstream-viz \
  -p "127.0.0.1:$VNC_PORT:5900" \
  -e ROS_DISTRO="$ROS_DISTRO" \
  -e DISPLAY=:99 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e QT_X11_NO_MITSHM=1 \
  -e NO_AT_BRIDGE=1 \
  -e VNC_PASSWORD="$VNC_PASSWORD" \
  -e MODEL_NAME="$MODEL_NAME" \
  -e PLAYBACK_RATE="$PLAYBACK_RATE" \
  -e SETTLE_SEC="$SETTLE_SEC" \
  -e EARLY_SCREENSHOT_SEC="$EARLY_SCREENSHOT_SEC" \
  -e MID_SCREENSHOT_SEC="$MID_SCREENSHOT_SEC" \
  -e XVFB_WIDTH="$XVFB_WIDTH" \
  -e XVFB_HEIGHT="$XVFB_HEIGHT" \
  -e ROBOT_A_ROS2_BAG="$ROBOT_A_ROS2_BAG" \
  -e ROBOT_B_ROS2_BAG="$ROBOT_B_ROS2_BAG" \
  -v "$DATA_DIR:/data:ro" \
  -v "$RESULTS_DIR:/results" \
  -v "$MODEL_DIR:/models:ro" \
  -v "$VIZ_SRC_DIR:/root/viz_ws/src/cslam_visualization:ro" \
  "$IMAGE" bash -s <<'CONTAINER_SCRIPT'
set -eo pipefail

capture_screen() {
  local name="$1"
  DISPLAY=:99 python3 -c 'import sys; from PIL import ImageGrab; out="/results/"+sys.argv[1]; im=ImageGrab.grab(xdisplay=":99"); im.save(out); print("Saved", out, im.size)' "$name" || true
}

mkdir -p /results
Xvfb :99 -screen 0 "${XVFB_WIDTH}x${XVFB_HEIGHT}x24" +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 &
for _ in $(seq 1 50); do
  [[ -S /tmp/.X11-unix/X99 ]] && break
  sleep 0.1
done
fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display :99 -forever -shared -passwd "$VNC_PASSWORD" -listen 0.0.0.0 -rfbport 5900 >/tmp/x11vnc.log 2>&1 &

source "/opt/ros/$ROS_DISTRO/setup.bash"
source /root/Swarm-SLAM/install/setup.bash

# Use the official visualization source and config directly. Building the package
# requires rtabmap_ros CMake files that are not present in the Swarm-SLAM image,
# but the visualization node used here is pure Python.
export PYTHONPATH="/root/viz_ws/src/cslam_visualization:${PYTHONPATH:-}"

CSLAM_SHARE=$(python3 - <<'PY'
from ament_index_python.packages import get_package_share_directory
print(get_package_share_directory('cslam'))
PY
)
mkdir -p "$CSLAM_SHARE/models"
ln -sf "/models/$MODEL_NAME" "$CSLAM_SHARE/models/resnet18_64.pth"

python3 /root/viz_ws/src/cslam_visualization/cslam_visualization/visualization_node.py \
  --ros-args \
  --params-file /root/viz_ws/src/cslam_visualization/config/realsense.yaml \
  >/results/official_visualization_node.log 2>&1 &
PID_VIZ_NODE=$!

rviz2 -d /root/viz_ws/src/cslam_visualization/config/realsense.rviz \
  >/results/official_rviz.log 2>&1 &
PID_RVIZ=$!

python3 - <<'PY' >/results/pose_graph_diagnostics.log 2>&1 &
import rclpy
from rclpy.node import Node
from cslam_common_interfaces.msg import PoseGraph
from visualization_msgs.msg import MarkerArray

class PoseGraphDiagnostics(Node):
    def __init__(self):
        super().__init__('agv_pose_graph_diagnostics')
        self.pose_counts = {}
        self.max_pose_counts = {}
        self.marker_counts = {}
        self.create_subscription(PoseGraph, '/cslam/viz/pose_graph', self.pose_cb, 10)
        self.create_subscription(MarkerArray, '/cslam/viz/pose_graph_markers', self.marker_cb, 10)
        self.create_timer(5.0, self.report)

    def pose_cb(self, msg):
        current = (len(msg.values), len(msg.edges), msg.origin_robot_id)
        previous = self.pose_counts.get(msg.robot_id)
        self.pose_counts[msg.robot_id] = current
        max_values, max_edges, _ = self.max_pose_counts.get(msg.robot_id, (0, 0, msg.origin_robot_id))
        self.max_pose_counts[msg.robot_id] = (
            max(max_values, current[0]),
            max(max_edges, current[1]),
            msg.origin_robot_id,
        )
        if previous != current:
            print(f"pose_graph robot={msg.robot_id} origin={msg.origin_robot_id} values={current[0]} edges={current[1]}", flush=True)

    def marker_cb(self, msg):
        summary = []
        for marker in msg.markers:
            summary.append((marker.ns, marker.id, marker.header.frame_id, len(marker.points)))
        self.marker_counts = summary

    def report(self):
        print(f"snapshot pose_counts={self.pose_counts} max_pose_counts={self.max_pose_counts} marker_counts={self.marker_counts}", flush=True)

rclpy.init()
node = PoseGraphDiagnostics()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
PID_DIAG=$!

sleep 3

ros2 launch cslam_experiments cslam_rgbd.launch.py \
  namespace:=/r0 robot_id:=0 max_nb_robots:=2 \
  config_file:=realsense_rgbd.yaml \
  >/results/r0_swarmslam.log 2>&1 &
PID_R0=$!

ros2 launch cslam_experiments cslam_rgbd.launch.py \
  namespace:=/r1 robot_id:=1 max_nb_robots:=2 \
  config_file:=realsense_rgbd.yaml \
  >/results/r1_swarmslam.log 2>&1 &
PID_R1=$!

sleep 3

ros2 bag play "$ROBOT_A_ROS2_BAG" --clock --rate "$PLAYBACK_RATE" --remap \
  /camera/color/image_raw:=/r0/color/image_raw \
  /camera/color/camera_info:=/r0/color/camera_info \
  /camera/aligned_depth_to_color/image_raw:=/r0/aligned_depth_to_color/image_raw \
  /odom:=/r0/odom \
  >/results/robotA_bag_play.log 2>&1 &
PID_BAG0=$!

ros2 bag play "$ROBOT_B_ROS2_BAG" --clock --rate "$PLAYBACK_RATE" --remap \
  /camera/color/image_raw:=/r1/color/image_raw \
  /camera/color/camera_info:=/r1/color/camera_info \
  /camera/aligned_depth_to_color/image_raw:=/r1/aligned_depth_to_color/image_raw \
  /odom:=/r1/odom \
  >/results/robotB_bag_play.log 2>&1 &
PID_BAG1=$!

(
  sleep "$EARLY_SCREENSHOT_SEC"
  capture_screen rviz_early.png
  sleep "$MID_SCREENSHOT_SEC"
  capture_screen rviz_mid.png
) &
PID_CAPTURE=$!

wait "$PID_BAG0" "$PID_BAG1"
echo "Bag playback finished. Waiting $SETTLE_SEC seconds for Swarm-SLAM optimization and visualization updates..."
sleep "$SETTLE_SEC"

capture_screen rviz_final.png
ros2 topic list | sort > /results/topics.txt || true
ros2 topic info /cslam/viz/pose_graph_markers > /results/pose_graph_markers_info.txt 2>&1 || true
ros2 topic info /cslam/viz/cloudmarker > /results/cloudmarker_info.txt 2>&1 || true
timeout 10s ros2 topic echo --once /cslam/viz/pose_graph_markers > /results/pose_graph_markers_once.yaml 2>&1 || true
timeout 8s ros2 run tf2_ros tf2_echo robot0_map robot1_map > /results/robot0_to_robot1_tf.txt 2>&1 || true

kill "$PID_CAPTURE" "$PID_BAG0" "$PID_BAG1" "$PID_R0" "$PID_R1" "$PID_VIZ_NODE" "$PID_RVIZ" "$PID_DIAG" >/dev/null 2>&1 || true
cp /tmp/xvfb.log /tmp/fluxbox.log /tmp/x11vnc.log /results/ >/dev/null 2>&1 || true
CONTAINER_SCRIPT
