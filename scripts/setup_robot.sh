#!/usr/bin/env bash
# Build and prepare the AGV robot-side stack after clone/pull.
#
# Usage:
#   bash scripts/setup_robot.sh
#   bash scripts/setup_robot.sh --skip-system
#
# By default this installs expected OS/ROS packages, then builds both catkin
# workspaces. Use --skip-system only on an already provisioned/offline robot.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INSTALL_SYSTEM=true
SUPPORTED_LIBREALSENSE_VERSIONS="${SUPPORTED_LIBREALSENSE_VERSIONS:-2.57.6 2.57.7}"

for arg in "$@"; do
    case "$arg" in
        --skip-system)
            INSTALL_SYSTEM=false
            ;;
        -h|--help)
            sed -n '1,16p' "$0"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg" >&2
            exit 2
            ;;
    esac
done

section() {
    echo ""
    echo "== $1 =="
}

require_file() {
    if [ ! -f "$1" ]; then
        echo "ERROR: missing required file: $1" >&2
        exit 1
    fi
}

check_realsense_version() {
    section "realsense sdk"
    if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists realsense2; then
        local version
        version="$(pkg-config --modversion realsense2)"
        echo "pkg-config realsense2: ${version}"
        if [[ " ${SUPPORTED_LIBREALSENSE_VERSIONS} " != *" ${version} "* ]]; then
            echo "WARN: validated RealSense SDK versions are ${SUPPORTED_LIBREALSENSE_VERSIONS}, found ${version}."
            echo "      Install a validated librealsense2 runtime/dev package and rebuild agv_ws before scenario runs."
        fi
    else
        echo "WARN: realsense2 pkg-config metadata not found."
        echo "      Install a validated librealsense2 runtime/dev package before scenario runs."
    fi
}

ensure_catkin_workspace() {
    if [ ! -d "$1/src" ]; then
        echo "ERROR: missing catkin workspace src directory: $1/src" >&2
        exit 1
    fi
    if [ ! -f "$1/.catkin_workspace" ]; then
        touch "$1/.catkin_workspace"
    fi
}

section "repo"
echo "root: ${ROOT}"
ensure_catkin_workspace "${ROOT}/myagv_ros"
ensure_catkin_workspace "${ROOT}/agv_ws"

if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f /opt/ros/noetic/setup.bash ]; then
    ROS_DISTRO="noetic"
    ROS_SETUP="/opt/ros/noetic/setup.bash"
elif [ -f /opt/ros/melodic/setup.bash ]; then
    ROS_DISTRO="melodic"
    ROS_SETUP="/opt/ros/melodic/setup.bash"
else
    echo "ERROR: neither ROS Noetic nor ROS Melodic setup.bash was found." >&2
    exit 1
fi

ROS_PKG_PREFIX="ros-${ROS_DISTRO}"
echo "ros: ${ROS_DISTRO} (${ROS_SETUP})"

if [ "$INSTALL_SYSTEM" = true ]; then
    section "system dependencies"
    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        chrony \
        cmake \
        git \
        pkg-config \
        python3-opencv \
        python3-pip \
        python3-yaml \
        "${ROS_PKG_PREFIX}-apriltag-ros" \
        "${ROS_PKG_PREFIX}-cv-bridge" \
        "${ROS_PKG_PREFIX}-ddynamic-reconfigure" \
        "${ROS_PKG_PREFIX}-diagnostic-msgs" \
        "${ROS_PKG_PREFIX}-geometry-msgs" \
        "${ROS_PKG_PREFIX}-image-transport-plugins" \
        "${ROS_PKG_PREFIX}-nav-msgs" \
        "${ROS_PKG_PREFIX}-rosbag" \
        "${ROS_PKG_PREFIX}-sensor-msgs" \
        "${ROS_PKG_PREFIX}-std-msgs" \
        "${ROS_PKG_PREFIX}-tf" \
        "${ROS_PKG_PREFIX}-tf2-msgs"

    if apt-cache show librealsense2-dev >/dev/null 2>&1; then
        sudo apt-get install -y librealsense2-dev librealsense2-utils
    elif apt-cache show "${ROS_PKG_PREFIX}-librealsense2" >/dev/null 2>&1; then
        sudo apt-get install -y \
            "${ROS_PKG_PREFIX}-librealsense2" \
            "${ROS_PKG_PREFIX}-realsense2-camera" \
            "${ROS_PKG_PREFIX}-realsense2-description"
        echo "WARN: installed ROS ${ROS_DISTRO} librealsense packages because Intel packages were not available."
        echo "      This may provide an older SDK than the validated stack: ${SUPPORTED_LIBREALSENSE_VERSIONS}."
    else
        echo "WARN: librealsense2 packages not available from configured apt sources."
        echo "      Install Intel RealSense packages separately if this robot is fresh."
    fi

    sudo systemctl enable --now chrony 2>/dev/null || sudo service chrony restart || true
fi

require_file "${ROS_SETUP}"

if ! command -v chronyc >/dev/null 2>&1; then
    echo "ERROR: chronyc not found; install chrony or rerun without --skip-system." >&2
    exit 1
fi

if ! chronyc tracking >/dev/null 2>&1; then
    echo "WARN: chrony is installed but not reporting tracking status yet."
fi

check_realsense_version

section "data directories"
mkdir -p "${HOME}/agv_data"
echo "bags: ${HOME}/agv_data"

if [ "${USE_SYSTEM_REALSENSE:-false}" = true ]; then
    section "workspace package selection"
    touch "${ROOT}/agv_ws/src/realsense-ros/realsense2_camera/CATKIN_IGNORE"
    touch "${ROOT}/agv_ws/src/realsense-ros/realsense2_description/CATKIN_IGNORE"
    echo "using system ROS realsense2_camera package"
fi

section "build myagv_ros"
source "${ROS_SETUP}"
cd "${ROOT}/myagv_ros"
catkin_make

section "build agv_ws"
source "${ROS_SETUP}"
source "${ROOT}/myagv_ros/devel/setup.bash"
cd "${ROOT}/agv_ws"
catkin_make

section "workspace check"
source "${ROS_SETUP}"
source "${ROOT}/myagv_ros/devel/setup.bash"
source "${ROOT}/agv_ws/devel/setup.bash"
rospack find agv_bringup
rospack find realsense2_camera
rospack find ydlidar_ros_driver
rospack find myagv_odometry

section "script permissions"
chmod +x \
    "${ROOT}/scripts/logging/start_session.sh" \
    "${ROOT}/scripts/logging/drive_straight.py" \
    "${ROOT}/scripts/logging/drive_square.py" \
    "${ROOT}/scripts/logging/drive_forward_back.py" \
    "${ROOT}/scripts/logging/drive_odom_shuttle.py" \
    "${ROOT}/scripts/logging/launch_odom_shuttle_fleet.sh" \
    "${ROOT}/scripts/logging/validate_bag.py" \
    "${ROOT}/scripts/logging/validate_ros2_bag.py" \
    "${ROOT}/scripts/logging/audit_bag_fast.py" \
    "${ROOT}/scripts/diagnostics/dataset_run_audit.py" \
    "${ROOT}/scripts/diagnostics/fleet_doctor_summary.py" \
    "${ROOT}/scripts/diagnostics/apply_robot_doctor_fix.sh" \
    "${ROOT}/scripts/diagnostics/robot_doctor.py" \
    "${ROOT}/scripts/diagnostics/robot_doctor.sh" \
    "${ROOT}/scripts/diagnostics/robot_doctor_selftest.py" \
    "${ROOT}/scripts/diagnostics/run_fleet_doctor_remote.sh" \
    "${ROOT}/scripts/diagnostics/run_robot_doctor_remote.sh" \
    "${ROOT}/scripts/diagnostics/synthesize_robot_doctor_failure.py" \
    "${ROOT}/scripts/diagnostics/validate_robot_doctor_report.py" \
    "${ROOT}/scripts/diagnostics/"*.sh 2>/dev/null || true

section "next commands"
cat <<EOF
source ${ROS_SETUP}
source ${ROOT}/myagv_ros/devel/setup.bash
source ${ROOT}/agv_ws/devel/setup.bash

# One-command data run:
bash ${ROOT}/scripts/logging/start_session.sh agv1 square_manual

# One-command readiness diagnosis:
bash ${ROOT}/scripts/diagnostics/robot_doctor.sh agv1 --profile preflight

# Optional manual teleop in another terminal:
rosrun myagv_teleop myagv_teleop.py
EOF
