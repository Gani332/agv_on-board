#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
IMAGE="${IMAGE:-agv-swarmslam:$ROS_DISTRO}"
PLATFORM="${PLATFORM:-linux/amd64}"

docker build \
  --platform "$PLATFORM" \
  -f "$ROOT_DIR/scripts/Dockerfile.swarmslam" \
  --build-arg "ROS_DISTRO=$ROS_DISTRO" \
  -t "$IMAGE" \
  "$ROOT_DIR"

echo "Built Docker image: $IMAGE ($PLATFORM)"
