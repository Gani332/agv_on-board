#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
IMAGE="${IMAGE:-agv-swarmslam:$ROS_DISTRO}"
PLATFORM="${PLATFORM:-linux/amd64}"
DATA_DIR="${DATA_DIR:-$ROOT_DIR/offline_slam_data}"
RESULTS_DIR="${RESULTS_DIR:-$ROOT_DIR/results}"
MODEL_DIR="${MODEL_DIR:-$ROOT_DIR/models}"
MODEL_NAME="${MODEL_NAME:-ResNet18_64_cosplace.pth}"
RVIZ="${RVIZ:-false}"
RVIZ_BACKEND="${RVIZ_BACKEND:-auto}"
VNC_PORT="${VNC_PORT:-5900}"
VNC_PASSWORD="${VNC_PASSWORD:-agv}"
DASHBOARD="${DASHBOARD:-false}"
DASHBOARD_PORT="${DASHBOARD_PORT:-8080}"
DASHBOARD_ROTATE_IMAGES="${DASHBOARD_ROTATE_IMAGES:-180}"
PLAYBACK_RATE="${PLAYBACK_RATE:-1.0}"
APRILTAG_GRAPH_CONSTRAINTS="${APRILTAG_GRAPH_CONSTRAINTS:-auto}"
APRILTAG_ALIGNMENT_FILE="${APRILTAG_ALIGNMENT_FILE:-/results/apriltags_pass1/robotA_to_robotB_alignment.json}"
APRILTAG_DETECTIONS_FILE="${APRILTAG_DETECTIONS_FILE:-/results/apriltags_pass1/detections.csv}"
ROBOT_LABELS="${ROBOT_LABELS:-robotA,robotB}"

IFS=',' read -r -a robot_labels <<< "$ROBOT_LABELS"
if [[ "${#robot_labels[@]}" -lt 1 ]]; then
  echo "No robots configured. Set ROBOT_LABELS='robot0,robot1,...'." >&2
  exit 1
fi

robot_bags=()
for label in "${robot_labels[@]}"; do
  if [[ -z "$label" ]]; then
    echo "Empty robot label in ROBOT_LABELS=$ROBOT_LABELS" >&2
    exit 1
  fi
  if [[ ! -d "$DATA_DIR/ros2/$label" ]]; then
    echo "Converted ROS 2 bag is missing: $DATA_DIR/ros2/$label" >&2
    echo "Run: FORCE=true BAGS='${label}=/path/to/${label}.bag' bash scripts/convert_bags_to_ros2.sh" >&2
    exit 1
  fi
  robot_bags+=("/data/ros2/$label")
done

ROBOT_COUNT="${ROBOT_COUNT:-${#robot_labels[@]}}"
if [[ "$ROBOT_COUNT" != "${#robot_labels[@]}" ]]; then
  echo "ROBOT_COUNT=$ROBOT_COUNT does not match ROBOT_LABELS count ${#robot_labels[@]}." >&2
  exit 1
fi

ROBOT_BAGS_CSV="$(IFS=','; echo "${robot_bags[*]}")"

if [[ ! -f "$MODEL_DIR/$MODEL_NAME" ]]; then
  echo "CosPlace model is missing: $MODEL_DIR/$MODEL_NAME" >&2
  echo "Run: bash scripts/download_cosplace_model.sh" >&2
  exit 1
fi

mkdir -p "$RESULTS_DIR"

docker_args=(
  --rm
  --platform "$PLATFORM"
  --name agv-swarmslam-offline
  -v "$DATA_DIR:/data:ro"
  -v "$RESULTS_DIR:/results"
  -v "$MODEL_DIR:/models:ro"
  -v "$ROOT_DIR/overlay/agv_swarmslam_tools/agv_swarmslam_tools:/root/Swarm-SLAM/build/agv_swarmslam_tools/agv_swarmslam_tools:ro"
  -v "$ROOT_DIR/overlay/agv_swarmslam_tools/agv_swarmslam_tools:/root/Swarm-SLAM/install/agv_swarmslam_tools/lib/python3.10/site-packages/agv_swarmslam_tools:ro"
  -v "$ROOT_DIR/overlay/agv_swarmslam_tools/launch:/root/Swarm-SLAM/install/agv_swarmslam_tools/share/agv_swarmslam_tools/launch:ro"
  -v "$ROOT_DIR/overlay/agv_swarmslam_tools/config:/root/Swarm-SLAM/install/agv_swarmslam_tools/share/agv_swarmslam_tools/config:ro"
  -v "$ROOT_DIR/overlay/agv_swarmslam_tools/rviz:/root/Swarm-SLAM/install/agv_swarmslam_tools/share/agv_swarmslam_tools/rviz:ro"
)

if [[ -t 0 && -t 1 ]]; then
  docker_args+=(-it)
fi

USE_VNC=false

if [[ "$RVIZ" == "true" ]]; then
  if [[ "$RVIZ_BACKEND" == "vnc" || ( "$RVIZ_BACKEND" == "auto" && "$(uname -s)" == "Darwin" ) ]]; then
    USE_VNC=true
    echo "RViz will be served over VNC. Open vnc://localhost:$VNC_PORT after the container starts."
    docker_args+=(
      -p "127.0.0.1:$VNC_PORT:5900"
      -e DISPLAY=:99
      -e LIBGL_ALWAYS_SOFTWARE=1
      -e QT_X11_NO_MITSHM=1
      -e NO_AT_BRIDGE=1
      -e VNC_PASSWORD="$VNC_PASSWORD"
    )
  elif [[ "$(uname -s)" == "Darwin" ]]; then
    if [[ "$(defaults read org.xquartz.X11 enable_iglx 2>/dev/null || echo 0)" != "1" ]]; then
      cat >&2 <<'EOF'
macOS RViz requires XQuartz with indirect GLX enabled.
Run this once, then restart XQuartz before running again:

  defaults write org.xquartz.X11 enable_iglx -bool true
  defaults write org.xquartz.X11 nolisten_tcp -bool false
  osascript -e 'quit app "XQuartz"'
  open -a XQuartz
  export DISPLAY=:0
  /opt/X11/bin/xhost +

EOF
      exit 1
    fi
    echo "macOS RViz requires XQuartz. Keep XQuartz open and allow clients with: /opt/X11/bin/xhost +"
    docker_args+=(
      -e DISPLAY=host.docker.internal:0
      -e LIBGL_ALWAYS_INDIRECT=1
      -e QT_X11_NO_MITSHM=1
      -e NO_AT_BRIDGE=1
    )
  else
    docker_args+=(
      -e DISPLAY="${DISPLAY:-:0}"
      -e LIBGL_ALWAYS_SOFTWARE=1
      -e QT_X11_NO_MITSHM=1
      -e NO_AT_BRIDGE=1
      -v /tmp/.X11-unix:/tmp/.X11-unix
    )
  fi
fi

if [[ "$DASHBOARD" == "true" ]]; then
  if (( ROBOT_COUNT > 2 )); then
    echo "Dashboard is two-robot only; use RViz for $ROBOT_COUNT robots." >&2
  fi
  echo "Live dashboard will be served at http://localhost:$DASHBOARD_PORT"
  docker_args+=(-p "127.0.0.1:$DASHBOARD_PORT:8080")
fi

if [[ "$APRILTAG_GRAPH_CONSTRAINTS" == "auto" ]]; then
  if [[ "$ROBOT_COUNT" == "2" && -f "$RESULTS_DIR/apriltags_pass1/robotA_to_robotB_alignment.json" && -f "$RESULTS_DIR/apriltags_pass1/detections.csv" ]]; then
    APRILTAG_GRAPH_CONSTRAINTS=true
  else
    APRILTAG_GRAPH_CONSTRAINTS=false
  fi
fi

if [[ "$APRILTAG_GRAPH_CONSTRAINTS" == "true" ]]; then
  if (( ROBOT_COUNT > 2 )); then
    echo "AprilTag graph constraint helper is pairwise; only enable this with a pairwise alignment file." >&2
  fi
  echo "AprilTag graph constraints enabled with $APRILTAG_ALIGNMENT_FILE"
fi

echo "Launching $ROBOT_COUNT robot(s): $ROBOT_LABELS"

docker run "${docker_args[@]}" "$IMAGE" bash -lc "
  set -eo pipefail
  if [[ \"$USE_VNC\" == \"true\" ]]; then
    Xvfb :99 -screen 0 1600x1000x24 +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 &
    for _ in \$(seq 1 50); do
      [[ -S /tmp/.X11-unix/X99 ]] && break
      sleep 0.1
    done
    fluxbox >/tmp/fluxbox.log 2>&1 &
    x11vnc -display :99 -forever -shared -passwd \"\$VNC_PASSWORD\" -listen 0.0.0.0 -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
    echo \"RViz VNC server is listening on vnc://localhost:$VNC_PORT\"
    echo \"VNC password: \$VNC_PASSWORD\"
  fi
  source /opt/ros/$ROS_DISTRO/setup.bash
  source /root/Swarm-SLAM/install/setup.bash
  if [[ \"$DASHBOARD\" == \"true\" ]]; then
    python3 /root/Swarm-SLAM/build/agv_swarmslam_tools/agv_swarmslam_tools/dashboard.py --port 8080 --rotate-images $DASHBOARD_ROTATE_IMAGES >/tmp/agv_dashboard.log 2>&1 &
    echo \"Live dashboard: http://localhost:$DASHBOARD_PORT\"
  fi
  ros2 launch agv_swarmslam_tools multi_robot_offline_rgbd.launch.py \
    robot_bags:=$ROBOT_BAGS_CSV \
    robot_count:=$ROBOT_COUNT \
    playback_rate:=$PLAYBACK_RATE \
    rviz:=$RVIZ \
    apriltag_graph_constraints:=$APRILTAG_GRAPH_CONSTRAINTS \
    apriltag_alignment_file:=$APRILTAG_ALIGNMENT_FILE \
    apriltag_detections_file:=$APRILTAG_DETECTIONS_FILE
"
