#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DEFAULT_BAGS="robotA=$ROOT_DIR/downloaded_bags/scenario1_square_20260506/robotA/robotA_scenario1_square_pass1.bag,robotB=$ROOT_DIR/downloaded_bags/scenario1_square_20260506/robotB/robotB_scenario1_square_pass1.bag"
BAGS="${BAGS:-$DEFAULT_BAGS}"
OUT_DIR="${OUT_DIR:-$ROOT_DIR/offline_slam_data/ros2}"
FORCE="${FORCE:-false}"
ROS2_TYPESTORE="${ROS2_TYPESTORE:-ros2_humble}"
ROS2_BAG_VERSION="${ROS2_BAG_VERSION:-8}"

TOPICS=(
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/aligned_depth_to_color/image_raw
  /odom
)

require_file() {
  local path="$1"
  if [[ ! -f "$path" ]]; then
    echo "Missing file: $path" >&2
    exit 1
  fi
}

convert_one() {
  local label="$1"
  local src="$2"
  local dst="$3"

  require_file "$src"

  if [[ -e "$dst" ]]; then
    if [[ "$FORCE" == "true" ]]; then
      rm -rf "$dst"
    else
      echo "Output already exists: $dst" >&2
      echo "Set FORCE=true to regenerate it." >&2
      exit 1
    fi
  fi

  mkdir -p "$(dirname "$dst")"
  echo "Converting $label: $src"
  rosbags-convert \
    --src "$src" \
    --dst "$dst" \
    --dst-storage sqlite3 \
    --dst-version "$ROS2_BAG_VERSION" \
    --dst-typestore "$ROS2_TYPESTORE" \
    --include-topic "${TOPICS[@]}"
}

if ! command -v rosbags-convert >/dev/null 2>&1; then
  echo "rosbags-convert is not installed. Install with: python3 -m pip install rosbags" >&2
  exit 1
fi

IFS=',' read -r -a bag_specs <<< "$BAGS"
if [[ "${#bag_specs[@]}" -lt 1 ]]; then
  echo "No bags configured. Set BAGS='robot0=/path/a.bag,robot1=/path/b.bag'." >&2
  exit 1
fi

for spec in "${bag_specs[@]}"; do
  if [[ "$spec" != *=* ]]; then
    echo "Invalid bag spec: $spec" >&2
    echo "Use BAGS='robot0=/path/a.bag,robot1=/path/b.bag'." >&2
    exit 1
  fi
  label="${spec%%=*}"
  src="${spec#*=}"
  if [[ -z "$label" || -z "$src" ]]; then
    echo "Invalid bag spec: $spec" >&2
    exit 1
  fi
  convert_one "$label" "$src" "$OUT_DIR/$label"
done

echo "ROS 2 bags written under: $OUT_DIR"
