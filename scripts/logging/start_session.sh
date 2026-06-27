#!/bin/bash
# start_session.sh - Start a dataset recording session with auto-generated manifest.
#
# Usage:
#   ./start_session.sh <robot_name> <scenario>
#   ./start_session.sh agv1 corridor_loop
#
# What it does:
#   1. Validates that ROS is running and all required topics are publishing
#   2. Generates a session_manifest.yaml before recording starts
#   3. Launches roslaunch agv_bringup logging.launch
#   4. On Ctrl+C, finalises the manifest with duration and bag size
#
# Run this on the robot. It is location-independent as long as this repo is
# intact, e.g. ~/slam_project/scripts/logging/start_session.sh.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Optional robot-local overrides, e.g. per-camera exposure settings.
CAMERA_CONFIG_FILE="${CAMERA_CONFIG_FILE:-${HOME}/.agv_camera_env}"
if [ -f "${CAMERA_CONFIG_FILE}" ]; then
    # shellcheck source=/dev/null
    source "${CAMERA_CONFIG_FILE}"
fi

# ---------------------------------------------------------------------------
# Args
# ---------------------------------------------------------------------------
ROBOT_NAME="${1:-agv_unknown}"
SCENARIO="${2:-unknown_scenario}"
DATESTAMP=$(date +%Y%m%d_%H%M%S)
MOCAP_TOPIC="${MOCAP_TOPIC:-/phasespace/rigids}"
REQUIRE_GT="${REQUIRE_GT:-false}"
REQUIRE_IMU="${REQUIRE_IMU:-false}"
# Camera IMU is intentionally explicit. Default recording uses the base /imu.
# The RSUSB D455 camera-IMU path is intentionally guarded. It exposes raw
# accel/gyro on AGV0 where the kernel/HID backend does not, but long full-stack
# tests showed the motion stream can stop while RGB-D continues. Use the base
# /imu for production collection unless explicitly running an RSUSB IMU debug
# test.
ENABLE_IMU="${ENABLE_IMU:-false}"
PUBLISH_BASE_IMU="${PUBLISH_BASE_IMU:-true}"
RECORD_BASE_IMU="${RECORD_BASE_IMU:-${PUBLISH_BASE_IMU}}"
ENABLE_CAMERA_IMU_FUSER="${ENABLE_CAMERA_IMU_FUSER:-false}"
CAMERA_IMU_FUSER_START_DELAY="${CAMERA_IMU_FUSER_START_DELAY:-25.0}"
REALSENSE_ALIGN_DEPTH="${REALSENSE_ALIGN_DEPTH:-true}"
REALSENSE_INITIAL_RESET="${REALSENSE_INITIAL_RESET:-false}"
REALSENSE_UNITE_IMU_METHOD="${REALSENSE_UNITE_IMU_METHOD:-linear_interpolation}"
REQUIRE_IMU_TOPIC="${REQUIRE_IMU_TOPIC:-/imu}"
USE_RSUSB_REALSENSE="${USE_RSUSB_REALSENSE:-false}"
RSUSB_PREFIX="${RSUSB_PREFIX:-${HOME}/rsusb_test/install/lrs-2.56.5-rsusb}"
RSUSB_WS="${RSUSB_WS:-${HOME}/agv_rsusb_ws}"
RSUSB_RESET_BEFORE_START="${RSUSB_RESET_BEFORE_START:-true}"
ALLOW_EXPERIMENTAL_RSUSB_IMU="${ALLOW_EXPERIMENTAL_RSUSB_IMU:-false}"
ENABLE_REALSENSE_SYNC="${ENABLE_REALSENSE_SYNC:-true}"
ENABLE_APRILTAG="${ENABLE_APRILTAG:-false}"
ENABLE_ARUCO="${ENABLE_ARUCO:-false}"
CAMERA_COLOR_WIDTH_WAS_SET="${CAMERA_COLOR_WIDTH+x}"
CAMERA_COLOR_HEIGHT_WAS_SET="${CAMERA_COLOR_HEIGHT+x}"
CAMERA_DEPTH_WIDTH_WAS_SET="${CAMERA_DEPTH_WIDTH+x}"
CAMERA_DEPTH_HEIGHT_WAS_SET="${CAMERA_DEPTH_HEIGHT+x}"
CAMERA_COLOR_WIDTH="${CAMERA_COLOR_WIDTH:-640}"
CAMERA_COLOR_HEIGHT="${CAMERA_COLOR_HEIGHT:-480}"
CAMERA_COLOR_FPS="${CAMERA_COLOR_FPS:-15}"
CAMERA_DEPTH_WIDTH="${CAMERA_DEPTH_WIDTH:-640}"
CAMERA_DEPTH_HEIGHT="${CAMERA_DEPTH_HEIGHT:-480}"
CAMERA_DEPTH_FPS="${CAMERA_DEPTH_FPS:-15}"
CAMERA_RGB_AUTO_EXPOSURE="${CAMERA_RGB_AUTO_EXPOSURE:-true}"
CAMERA_RGB_EXPOSURE="${CAMERA_RGB_EXPOSURE:-166}"
CAMERA_RGB_GAIN="${CAMERA_RGB_GAIN:-64}"
CAMERA_RGB_MEAN_MAX="${CAMERA_RGB_MEAN_MAX:-230}"
CAMERA_RGB_SAT250_MAX_PERCENT="${CAMERA_RGB_SAT250_MAX_PERCENT:-80}"
CAMERA_RGB_PROBE_TIMEOUT="${CAMERA_RGB_PROBE_TIMEOUT:-8}"
if [ "${USE_RSUSB_REALSENSE}" = true ] && [ "${ENABLE_IMU}" = true ]; then
    # Validated on AGV0 (10.23.118.99, 2026-05-19): the normal kernel/HID
    # backend does not expose the D455 BMI085 IMU, while RSUSB does. However, a
    # 10-minute full-stack bag at 480x270@15 showed raw accel/gyro stopped after
    # 259 s while RGB-D, LiDAR and odom continued. Keep this path explicit so it
    # cannot be used accidentally for publication data collection.
    if [ "${ALLOW_EXPERIMENTAL_RSUSB_IMU}" != true ]; then
        echo "ERROR: USE_RSUSB_REALSENSE=true with ENABLE_IMU=true is experimental on AGV0." >&2
        echo "       Evidence: raw /camera/accel/sample and /camera/gyro/sample stopped after 259 s in a 10-minute full-stack test." >&2
        echo "       For production collection, use the base /imu path. To run an RSUSB IMU debug test, set ALLOW_EXPERIMENTAL_RSUSB_IMU=true." >&2
        exit 1
    fi
    REALSENSE_ALIGN_DEPTH=false
    ENABLE_REALSENSE_SYNC=false
    REALSENSE_UNITE_IMU_METHOD=none
    ENABLE_CAMERA_IMU_FUSER=false
    if [ "${REQUIRE_IMU_TOPIC}" = "/imu" ]; then
        REQUIRE_IMU_TOPIC="/camera/accel/sample"
    fi
    if [ -z "${CAMERA_COLOR_WIDTH_WAS_SET}" ]; then CAMERA_COLOR_WIDTH=480; fi
    if [ -z "${CAMERA_COLOR_HEIGHT_WAS_SET}" ]; then CAMERA_COLOR_HEIGHT=270; fi
    if [ -z "${CAMERA_DEPTH_WIDTH_WAS_SET}" ]; then CAMERA_DEPTH_WIDTH=480; fi
    if [ -z "${CAMERA_DEPTH_HEIGHT_WAS_SET}" ]; then CAMERA_DEPTH_HEIGHT=270; fi
fi
if [ "${REALSENSE_ALIGN_DEPTH}" = true ]; then
    DEPTH_IMAGE_TOPIC="/camera/aligned_depth_to_color/image_raw"
else
    DEPTH_IMAGE_TOPIC="/camera/depth/image_rect_raw"
fi
SESSION_ID="${ROBOT_NAME}_${SCENARIO}_${DATESTAMP}"
BAG_DIR="${HOME}/agv_data"
BAG_FILE="${BAG_DIR}/${SESSION_ID}.bag"
MANIFEST_FILE="${BAG_DIR}/${SESSION_ID}_manifest.yaml"
CHRONY_FILE="${BAG_DIR}/${SESSION_ID}_chrony.txt"

mkdir -p "${BAG_DIR}"

# ---------------------------------------------------------------------------
# Source ROS
# ---------------------------------------------------------------------------
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
else
    echo "ERROR: no supported ROS setup found under /opt/ros" >&2
    exit 1
fi

if [ -f "${ROOT}/myagv_ros/devel/setup.bash" ]; then
    source "${ROOT}/myagv_ros/devel/setup.bash"
fi
source "${ROOT}/agv_ws/devel/setup.bash"
if [ "${USE_RSUSB_REALSENSE}" = true ]; then
    if [ ! -f "${RSUSB_WS}/devel/setup.bash" ]; then
        echo "ERROR: USE_RSUSB_REALSENSE=true but ${RSUSB_WS}/devel/setup.bash is missing" >&2
        exit 1
    fi
    if [ ! -d "${RSUSB_PREFIX}/lib" ]; then
        echo "ERROR: USE_RSUSB_REALSENSE=true but ${RSUSB_PREFIX}/lib is missing" >&2
        exit 1
    fi
    source "${RSUSB_WS}/devel/setup.bash"
    export LD_LIBRARY_PATH="${RSUSB_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export PKG_CONFIG_PATH="${RSUSB_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
fi

case "${CAMERA_RGB_AUTO_EXPOSURE}" in
    true|True|TRUE|1|yes|Yes|YES) CAMERA_RGB_AUTO_EXPOSURE=true ;;
    false|False|FALSE|0|no|No|NO) CAMERA_RGB_AUTO_EXPOSURE=false ;;
    *)
        echo "ERROR: CAMERA_RGB_AUTO_EXPOSURE must be true or false, got '${CAMERA_RGB_AUTO_EXPOSURE}'" >&2
        exit 1
        ;;
esac

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------
echo "=== Pre-flight checks ==="

# Record clock-sync state before every run. This is evidence that robot sensor
# stamps can be compared with mocap stamps from a chrony-synced GT machine.
{
    echo "# Chrony snapshot for ${SESSION_ID}"
    echo "# Captured: $(date --iso-8601=ns)"
    echo ""
    if command -v chronyc >/dev/null 2>&1; then
        echo "## chronyc tracking"
        chronyc tracking 2>&1 || true
        echo ""
        echo "## chronyc sources -v"
        chronyc sources -v 2>&1 || true
    else
        echo "chronyc not installed"
    fi
} > "${CHRONY_FILE}"
echo "  [i] chrony snapshot: ${CHRONY_FILE}"

# Check required topics are publishing (best-effort, bounded timeout).
# If logging.launch is allowed to start bringup itself these checks may warn
# before sensors exist; validate_bag.py remains the authoritative post-run gate.
REQUIRED_TOPICS="/scan /odom /tf /camera/color/image_raw ${DEPTH_IMAGE_TOPIC}"
OPTIONAL_TOPICS=""
if [ "${RECORD_BASE_IMU}" = true ]; then
    OPTIONAL_TOPICS="${OPTIONAL_TOPICS} /imu"
fi
if [ "${ENABLE_IMU}" = true ] || [ "${ENABLE_CAMERA_IMU_FUSER}" = true ]; then
    OPTIONAL_TOPICS="${OPTIONAL_TOPICS} /camera/imu /camera/accel/sample /camera/gyro/sample"
fi
GROUND_TRUTH_TOPICS="${MOCAP_TOPIC} /mocap"
ALL_OK=true

if ! rostopic list > /dev/null 2>&1; then
    echo "  [i] ROS master not running yet; logging.launch will start bringup."
    echo "      Skipping live topic probes before launch."
else
    for topic in $REQUIRED_TOPICS; do
        if timeout 6 rostopic hz "$topic" --window 10 2>/dev/null | grep -q "average rate"; then
            echo "  [OK] $topic publishing"
        else
            # Try simpler check
            if timeout 3 rostopic echo "$topic" -n 1 > /dev/null 2>&1; then
                echo "  [OK] $topic publishing"
            else
                echo "  [!] $topic not detected - may not be running yet"
                ALL_OK=false
            fi
        fi
    done

    for topic in $OPTIONAL_TOPICS; do
        if timeout 4 rostopic hz "$topic" --window 10 2>/dev/null | grep -q "average rate"; then
            echo "  [OK] optional $topic publishing"
        else
            echo "  [i] optional $topic not detected"
        fi
    done

    GT_OK=false
    for topic in $GROUND_TRUTH_TOPICS; do
        if timeout 3 rostopic echo "$topic" -n 1 > /dev/null 2>&1; then
            echo "  [OK] ground truth topic detected: $topic"
            GT_OK=true
            break
        fi
    done
    if [ "$GT_OK" = false ]; then
        if [ "$REQUIRE_GT" = true ]; then
            echo "ERROR: no ground truth topic detected (${GROUND_TRUTH_TOPICS})"
            exit 1
        else
            echo "  [i] no ground truth topic detected yet (${GROUND_TRUTH_TOPICS})"
            echo "      Recording can proceed; use REQUIRE_GT=true when GT must be present."
        fi
    fi

    if [ "$REQUIRE_IMU" = true ]; then
        if timeout 4 rostopic hz "${REQUIRE_IMU_TOPIC}" --window 10 2>/dev/null | grep -q "average rate"; then
            echo "  [OK] required IMU topic publishing: ${REQUIRE_IMU_TOPIC}"
        else
            echo "ERROR: REQUIRE_IMU=true but ${REQUIRE_IMU_TOPIC} is not publishing."
            exit 1
        fi
    fi
fi

if [ "$ALL_OK" = false ]; then
    echo ""
    echo "WARNING: Some topics not detected. Starting logging anyway."
    echo "Run validate_bag.py after recording to check data quality."
    echo ""
fi

# ---------------------------------------------------------------------------
# Write initial manifest
# ---------------------------------------------------------------------------
ROS_DISTRO_VAL=$(echo "${ROS_DISTRO:-melodic}")
CALIB_DIR="${ROOT}/agv_ws/src/agv_bringup/calibration"
if [ -d "$CALIB_DIR" ]; then
    CALIB_HASH=$(find "$CALIB_DIR" -type f | sort | xargs sha256sum 2>/dev/null | sha256sum | cut -d' ' -f1)
else
    CALIB_HASH="unavailable"
fi

cat > "${MANIFEST_FILE}" << EOF
# Session manifest - auto-generated by start_session.sh
session_id: ${SESSION_ID}
robot_id: ${ROBOT_NAME}
scenario: ${SCENARIO}
date: $(date +%Y-%m-%d)
time_start: $(date +%H:%M:%S)
time_end: ~
operator: $(whoami)
ros_distro: ${ROS_DISTRO_VAL}
bag_file: ${SESSION_ID}.bag
chrony_file: ${SESSION_ID}_chrony.txt
bag_size_mb: ~
duration_sec: ~
calibration_hash: "sha256:${CALIB_HASH}"
mocap_topic: "${MOCAP_TOPIC}"
ground_truth_required: ${REQUIRE_GT}
imu_required: ${REQUIRE_IMU}
imu_required_topic: "${REQUIRE_IMU_TOPIC}"
enable_imu: ${ENABLE_IMU}
publish_base_imu: ${PUBLISH_BASE_IMU}
record_base_imu: ${RECORD_BASE_IMU}
enable_camera_imu_fuser: ${ENABLE_CAMERA_IMU_FUSER}
camera_imu_fuser_start_delay: ${CAMERA_IMU_FUSER_START_DELAY}
realsense_align_depth: ${REALSENSE_ALIGN_DEPTH}
realsense_initial_reset: ${REALSENSE_INITIAL_RESET}
realsense_unite_imu_method: "${REALSENSE_UNITE_IMU_METHOD}"
use_rsusb_realsense: ${USE_RSUSB_REALSENSE}
rsusb_prefix: "${RSUSB_PREFIX}"
rsusb_ws: "${RSUSB_WS}"
rsusb_reset_before_start: ${RSUSB_RESET_BEFORE_START}
allow_experimental_rsusb_imu: ${ALLOW_EXPERIMENTAL_RSUSB_IMU}
enable_realsense_sync: ${ENABLE_REALSENSE_SYNC}
enable_apriltag: ${ENABLE_APRILTAG}
enable_aruco: ${ENABLE_ARUCO}
camera_profile:
  color_width: ${CAMERA_COLOR_WIDTH}
  color_height: ${CAMERA_COLOR_HEIGHT}
  color_fps: ${CAMERA_COLOR_FPS}
  depth_width: ${CAMERA_DEPTH_WIDTH}
  depth_height: ${CAMERA_DEPTH_HEIGHT}
  depth_fps: ${CAMERA_DEPTH_FPS}
rgb_exposure_control:
  auto_exposure: ${CAMERA_RGB_AUTO_EXPOSURE}
  manual_exposure: ${CAMERA_RGB_EXPOSURE}
  manual_gain: ${CAMERA_RGB_GAIN}
  max_mean_before_record: ${CAMERA_RGB_MEAN_MAX}
  max_sat250_percent_before_record: ${CAMERA_RGB_SAT250_MAX_PERCENT}
notes: ""
usb_mode_note: "D455 observed on USB 3.2. Normal kernel/HID backend does not expose the BMI085 IMU on AGV0. RSUSB exposes raw accel/gyro, but a 10-minute full-stack test at 480x270@15 showed the motion stream stopped after 259 s while RGB-D continued. Use the base /imu for production collection; RSUSB camera-IMU is debug-only unless revalidated."
EOF

echo ""
echo "=== Session: ${SESSION_ID} ==="
echo "Bag:      ${BAG_FILE}"
echo "Manifest: ${MANIFEST_FILE}"
echo ""
echo "Press Ctrl+C to stop recording."
echo ""

# ---------------------------------------------------------------------------
# Launch bringup, wait for sensors, then record.
# ---------------------------------------------------------------------------
START_EPOCH=$(date +%s)
BRINGUP_PID=""
ARUCO_PID=""
APRILTAG_PID=""
ROSBAG_PID=""
CLEANED_UP=false

finalise_manifest() {
    echo ""
    echo "=== Finalising manifest ==="
    END_EPOCH=$(date +%s)
    DURATION=$((END_EPOCH - START_EPOCH))

    if [ -f "${BAG_FILE}" ]; then
        BAG_SIZE_MB=$(du -m "${BAG_FILE}" 2>/dev/null | cut -f1)
    else
        # rosbag appends .bag automatically but also sometimes names it differently
        ACTUAL_BAG=$(ls "${BAG_DIR}/${SESSION_ID}"*.bag 2>/dev/null | head -1)
        BAG_SIZE_MB=$(du -m "${ACTUAL_BAG}" 2>/dev/null | cut -f1 || echo "~")
    fi

    # Update manifest with final values
    sed -i "s/time_end: ~/time_end: $(date +%H:%M:%S)/" "${MANIFEST_FILE}"
    sed -i "s/bag_size_mb: ~/bag_size_mb: ${BAG_SIZE_MB:-unknown}/" "${MANIFEST_FILE}"
    sed -i "s/duration_sec: ~/duration_sec: ${DURATION}/" "${MANIFEST_FILE}"

    {
        echo ""
        echo "# Post-run chrony snapshot for ${SESSION_ID}"
        echo "# Captured: $(date --iso-8601=ns)"
        echo ""
        if command -v chronyc >/dev/null 2>&1; then
            echo "## chronyc tracking"
            chronyc tracking 2>&1 || true
            echo ""
            echo "## chronyc sources -v"
            chronyc sources -v 2>&1 || true
        else
            echo "chronyc not installed"
        fi
    } >> "${CHRONY_FILE}"

    echo "Duration: ${DURATION}s"
    echo "Bag size: ${BAG_SIZE_MB:-unknown} MB"
    echo "Manifest written: ${MANIFEST_FILE}"
    echo ""
    echo "Run quality check:"
    echo "  python3 scripts/logging/validate_bag.py ${BAG_DIR}/${SESSION_ID}.bag"
}

cleanup() {
    if [ "$CLEANED_UP" = true ]; then
        return
    fi
    CLEANED_UP=true
    trap - EXIT INT TERM

    if [ -n "${ROSBAG_PID}" ] && kill -0 "${ROSBAG_PID}" 2>/dev/null; then
        echo ""
        echo "Stopping rosbag..."
        kill -INT "${ROSBAG_PID}" 2>/dev/null || true
        wait "${ROSBAG_PID}" 2>/dev/null || true
    fi
    if [ -n "${ARUCO_PID}" ] && kill -0 "${ARUCO_PID}" 2>/dev/null; then
        echo "Stopping ArUco detector..."
        kill -INT "${ARUCO_PID}" 2>/dev/null || true
        wait "${ARUCO_PID}" 2>/dev/null || true
    fi
    if [ -n "${APRILTAG_PID}" ] && kill -0 "${APRILTAG_PID}" 2>/dev/null; then
        echo "Stopping AprilTag detector..."
        kill -INT "${APRILTAG_PID}" 2>/dev/null || true
        wait "${APRILTAG_PID}" 2>/dev/null || true
    fi
    if [ -n "${BRINGUP_PID}" ] && kill -0 "${BRINGUP_PID}" 2>/dev/null; then
        echo "Stopping bringup..."
        kill -INT "${BRINGUP_PID}" 2>/dev/null || true
        wait "${BRINGUP_PID}" 2>/dev/null || true
    fi

    finalise_manifest
}

handle_signal() {
    cleanup
    exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

# Export env vars for any launched child tools.
export ROBOT_NAME="$ROBOT_NAME"
export SCENARIO="$SCENARIO"
export DATESTAMP="$DATESTAMP"
export MOCAP_TOPIC="$MOCAP_TOPIC"
export REQUIRE_GT="$REQUIRE_GT"
export REQUIRE_IMU="$REQUIRE_IMU"
export ENABLE_IMU="$ENABLE_IMU"
export ENABLE_REALSENSE_SYNC="$ENABLE_REALSENSE_SYNC"
export ENABLE_APRILTAG="$ENABLE_APRILTAG"
export ENABLE_ARUCO="$ENABLE_ARUCO"
export CAMERA_COLOR_WIDTH="$CAMERA_COLOR_WIDTH"
export CAMERA_COLOR_HEIGHT="$CAMERA_COLOR_HEIGHT"
export CAMERA_COLOR_FPS="$CAMERA_COLOR_FPS"
export CAMERA_DEPTH_WIDTH="$CAMERA_DEPTH_WIDTH"
export CAMERA_DEPTH_HEIGHT="$CAMERA_DEPTH_HEIGHT"
export CAMERA_DEPTH_FPS="$CAMERA_DEPTH_FPS"
export CAMERA_RGB_AUTO_EXPOSURE="$CAMERA_RGB_AUTO_EXPOSURE"
export CAMERA_RGB_EXPOSURE="$CAMERA_RGB_EXPOSURE"
export CAMERA_RGB_GAIN="$CAMERA_RGB_GAIN"
export CAMERA_RGB_MEAN_MAX="$CAMERA_RGB_MEAN_MAX"
export CAMERA_RGB_SAT250_MAX_PERCENT="$CAMERA_RGB_SAT250_MAX_PERCENT"
export CAMERA_RGB_PROBE_TIMEOUT="$CAMERA_RGB_PROBE_TIMEOUT"

wait_for_topic_rate() {
    topic="$1"
    timeout_s="$2"
    end=$((SECONDS + timeout_s))
    while [ "$SECONDS" -lt "$end" ]; do
        if timeout 6 rostopic hz "$topic" --window 10 2>/dev/null | grep -q "average rate"; then
            echo "  [OK] $topic publishing"
            return 0
        fi
        if timeout 4 rostopic echo "$topic" -n 1 >/dev/null 2>&1; then
            echo "  [OK] $topic message received"
            return 0
        fi
        sleep 1
    done
    echo "ERROR: timed out waiting for $topic" >&2
    return 1
}

apply_rgb_exposure_settings() {
    echo "Applying RGB exposure settings..."
    if timeout 8 rosrun dynamic_reconfigure dynparam set /camera/rgb_camera enable_auto_exposure "${CAMERA_RGB_AUTO_EXPOSURE}" >/dev/null 2>&1; then
        echo "  [OK] /camera/rgb_camera enable_auto_exposure=${CAMERA_RGB_AUTO_EXPOSURE}"
    else
        echo "  [!] could not set /camera/rgb_camera enable_auto_exposure" >&2
    fi

    if [ "${CAMERA_RGB_AUTO_EXPOSURE}" = false ]; then
        timeout 8 rosrun dynamic_reconfigure dynparam set /camera/rgb_camera exposure "${CAMERA_RGB_EXPOSURE}" >/dev/null 2>&1 || \
            echo "  [!] could not set /camera/rgb_camera exposure" >&2
        timeout 8 rosrun dynamic_reconfigure dynparam set /camera/rgb_camera gain "${CAMERA_RGB_GAIN}" >/dev/null 2>&1 || \
            echo "  [!] could not set /camera/rgb_camera gain" >&2
        echo "  [OK] requested manual RGB exposure=${CAMERA_RGB_EXPOSURE} gain=${CAMERA_RGB_GAIN}"
    else
        sleep 2
    fi

    RGB_DYNPARAM_FILE="${BAG_DIR}/${SESSION_ID}_rgb_camera_dynparam.txt"
    timeout 8 rosrun dynamic_reconfigure dynparam get /camera/rgb_camera > "${RGB_DYNPARAM_FILE}" 2>&1 || true
    echo "  [i] RGB dynamic params: ${RGB_DYNPARAM_FILE}"
}

check_rgb_exposure_sample() {
    echo "Checking RGB exposure sample before recording..."
    python3 - <<'PY'
import os
import sys

import rospy
from sensor_msgs.msg import Image

mean_max = float(os.environ.get("CAMERA_RGB_MEAN_MAX", "230"))
sat_max = float(os.environ.get("CAMERA_RGB_SAT250_MAX_PERCENT", "80"))
timeout_s = float(os.environ.get("CAMERA_RGB_PROBE_TIMEOUT", "8"))

rospy.init_node("rgb_exposure_gate", anonymous=True, disable_signals=True)
msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=timeout_s)
data = msg.data
if not data:
    print("ERROR: /camera/color/image_raw returned an empty image", file=sys.stderr)
    sys.exit(2)

n = len(data)
mean = float(sum(data)) / n
sat250 = 100.0 * sum(1 for value in data if value >= 250) / n
print("  [i] RGB sample: encoding=%s %dx%d mean=%.2f sat250=%.2f%%" %
      (msg.encoding, msg.width, msg.height, mean, sat250))

if mean > mean_max or sat250 > sat_max:
    print("ERROR: RGB image looks overexposed before recording "
          "(mean %.2f > %.2f or sat250 %.2f%% > %.2f%%)" %
          (mean, mean_max, sat250, sat_max), file=sys.stderr)
    sys.exit(2)
PY
}

if [ "${USE_RSUSB_REALSENSE}" = true ] && [ "${RSUSB_RESET_BEFORE_START}" = true ]; then
    RESET_TOOL="${HOME}/rsusb_test/rsusb_realsense_reset"
    if [ -x "${RESET_TOOL}" ]; then
        echo "Resetting RealSense through RSUSB before bringup..."
        timeout 15 "${RESET_TOOL}" || true
        sleep 5
    else
        echo "WARNING: RSUSB reset requested but ${RESET_TOOL} is missing or not executable" >&2
    fi
fi

BRINGUP_LOG="${BAG_DIR}/${SESSION_ID}_bringup.log"
echo "Starting bringup first; log: ${BRINGUP_LOG}"
roslaunch agv_bringup bringup.launch \
    enable_imu:="${ENABLE_IMU}" \
    publish_base_imu:="${PUBLISH_BASE_IMU}" \
    enable_camera_imu_fuser:="${ENABLE_CAMERA_IMU_FUSER}" \
    camera_imu_fuser_start_delay:="${CAMERA_IMU_FUSER_START_DELAY}" \
    realsense_align_depth:="${REALSENSE_ALIGN_DEPTH}" \
    realsense_initial_reset:="${REALSENSE_INITIAL_RESET}" \
    realsense_unite_imu_method:="${REALSENSE_UNITE_IMU_METHOD}" \
    enable_realsense_sync:="${ENABLE_REALSENSE_SYNC}" \
    color_width:="${CAMERA_COLOR_WIDTH}" \
    color_height:="${CAMERA_COLOR_HEIGHT}" \
    color_fps:="${CAMERA_COLOR_FPS}" \
    depth_width:="${CAMERA_DEPTH_WIDTH}" \
    depth_height:="${CAMERA_DEPTH_HEIGHT}" \
    depth_fps:="${CAMERA_DEPTH_FPS}" \
    rgb_enable_auto_exposure:="${CAMERA_RGB_AUTO_EXPOSURE}" \
    rgb_exposure:="${CAMERA_RGB_EXPOSURE}" \
    rgb_gain:="${CAMERA_RGB_GAIN}" \
    > "${BRINGUP_LOG}" 2>&1 &
BRINGUP_PID=$!

echo "Waiting for required sensor streams before recording..."
wait_for_topic_rate /scan 45
wait_for_topic_rate /odom 45
wait_for_topic_rate /camera/color/image_raw 60
wait_for_topic_rate "${DEPTH_IMAGE_TOPIC}" 60
apply_rgb_exposure_settings
check_rgb_exposure_sample
if [ "$REQUIRE_IMU" = true ]; then
    wait_for_topic_rate "${REQUIRE_IMU_TOPIC}" 45
    if [ "${ENABLE_IMU}" = true ] && [ "${REALSENSE_UNITE_IMU_METHOD}" = none ]; then
        wait_for_topic_rate /camera/gyro/sample 45
    fi
fi

if [ "$ENABLE_APRILTAG" = true ]; then
    APRILTAG_LOG="${BAG_DIR}/${SESSION_ID}_apriltag.log"
    echo "Starting AprilTag detector; log: ${APRILTAG_LOG}"
    roslaunch agv_bringup apriltag.launch \
        publish_detection_image:=false \
        > "${APRILTAG_LOG}" 2>&1 &
    APRILTAG_PID=$!
    wait_for_topic_rate /tag_detections 45
fi

if [ "$ENABLE_ARUCO" = true ]; then
    ARUCO_LOG="${BAG_DIR}/${SESSION_ID}_aruco.log"
    echo "Starting optional ArUco detector; log: ${ARUCO_LOG}"
    roslaunch agv_bringup aruco.launch \
        dictionary:="${ARUCO_DICTIONARY:-original}" \
        marker_size:="${ARUCO_MARKER_SIZE:-0.15}" \
        target_id:="${ARUCO_TARGET_ID:-503}" \
        publish_image:=false \
        publish_pose:=true \
        > "${ARUCO_LOG}" 2>&1 &
    ARUCO_PID=$!
fi

echo "Sensors are live; starting rosbag."
START_EPOCH=$(date +%s)
ROSBAG_TOPICS=(
    /scan
    /odom
    /cmd_vel
    /tf
    /tf_static
    /camera/color/image_raw
    /camera/color/camera_info
    /camera/depth/image_rect_raw
    /camera/depth/camera_info
    /camera/aligned_depth_to_color/image_raw
    /camera/aligned_depth_to_color/camera_info
    /camera/extrinsics/depth_to_color
    /diagnostics
    /aruco/target_pose
    /tag_detections
    "${MOCAP_TOPIC}"
    /mocap
)

if [ "${RECORD_BASE_IMU}" = true ]; then
    ROSBAG_TOPICS+=(/imu)
fi

if [ "${ENABLE_IMU}" = true ] || [ "${ENABLE_CAMERA_IMU_FUSER}" = true ] || [ "${REQUIRE_IMU_TOPIC}" = "/camera/imu" ]; then
    ROSBAG_TOPICS+=(
        /camera/imu
        /camera/accel/sample
        /camera/gyro/sample
        /camera/accel/imu_info
        /camera/gyro/imu_info
    )
fi

rosbag record --buffsize=2048 --lz4 -O "${BAG_FILE}" "${ROSBAG_TOPICS[@]}" &
ROSBAG_PID=$!
wait "${ROSBAG_PID}"
ROSBAG_PID=""
