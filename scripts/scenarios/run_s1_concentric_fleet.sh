#!/usr/bin/env bash
# Run Scenario 1 across a fleet: clock-gated recording, scheduled staggered
# concentric-circle motion, synchronized shutdown, and post-run bag validation.
#
# Edit the ROBOTS/RADII defaults below, or override them with space-separated
# environment variables:
#
#   MOCAP_TARGET="ubuntu@10.23.x.y" \
#   ROBOTS_OVERRIDE="10.23.16.229 10.23.37.117 10.23.22.246 10.23.118.99" \
#   RADII_OVERRIDE="2.0 1.5 1.0 0.5" \
#   bash scripts/scenarios/run_s1_concentric_fleet.sh

set -euo pipefail

# shellcheck disable=SC2206
ROBOTS=(${ROBOTS_OVERRIDE:-"10.23.16.229 10.23.37.117 10.23.22.246 10.23.118.99"})
# shellcheck disable=SC2206
RADII=(${RADII_OVERRIDE:-"2.0 1.5 1.0 0.5"})

if [ -n "${ROBOT_NAMES_OVERRIDE:-}" ]; then
    # shellcheck disable=SC2206
    ROBOT_NAMES=(${ROBOT_NAMES_OVERRIDE})
else
    ROBOT_NAMES=()
    for i in "${!ROBOTS[@]}"; do
        ROBOT_NAMES+=("agv${i}")
    done
fi

SCENARIO="${SCENARIO:-s1_concentric}"
LINEAR="${LINEAR:-0.20}"
DURATION="${DURATION:-720.0}"
STAGGER="${STAGGER:-15}"
START_LEAD_SEC="${START_LEAD_SEC:-90}"
POST_ROLL_SEC="${POST_ROLL_SEC:-5}"
STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-180}"
MIN_START_MARGIN_SEC="${MIN_START_MARGIN_SEC:-10}"
MIN_FREE_GB="${MIN_FREE_GB:-8}"
REQUIRE_IMU="${REQUIRE_IMU:-true}"
REQUIRE_GT="${REQUIRE_GT:-false}"
ENABLE_APRILTAG="${ENABLE_APRILTAG:-true}"
NO_FRAME_DROPS="${NO_FRAME_DROPS:-true}"
REMOTE_ROOT="${REMOTE_ROOT:-/home/ubuntu/slam_project}"
CHECK_MOCAP_CHRONY="${CHECK_MOCAP_CHRONY:-true}"
MOCAP_TARGET="${MOCAP_TARGET:-}"
MOCAP_PASS="${MOCAP_PASS:-}"
MAX_CHRONY_OFFSET_SEC="${MAX_CHRONY_OFFSET_SEC:-0.005}"
CAMERA_RGB_AUTO_EXPOSURE="${CAMERA_RGB_AUTO_EXPOSURE:-true}"
CAMERA_RGB_EXPOSURE="${CAMERA_RGB_EXPOSURE:-166}"
CAMERA_RGB_GAIN="${CAMERA_RGB_GAIN:-64}"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_ROOT="${LOG_ROOT:-fleet_logs/${SCENARIO}_${STAMP}}"
SSH_OPTS=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=10)

usage_error() {
    echo "ERROR: $*" >&2
    exit 2
}

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || usage_error "missing command: $1"
}

validate_token() {
    local label="$1"
    local value="$2"
    [[ "${value}" =~ ^[A-Za-z0-9_.:-]+$ ]] || usage_error "${label} contains unsafe characters: ${value}"
}

validate_number() {
    local label="$1"
    local value="$2"
    [[ "${value}" =~ ^[0-9]+([.][0-9]+)?$ ]] || usage_error "${label} must be numeric: ${value}"
}

validate_int() {
    local label="$1"
    local value="$2"
    [[ "${value}" =~ ^[0-9]+$ ]] || usage_error "${label} must be an integer: ${value}"
}

ssh_robot() {
    local ip="$1"
    shift
    sshpass -p "${PASS}" ssh -n "${SSH_OPTS[@]}" "ubuntu@${ip}" "$@"
}

remote_bash() {
    local ip="$1"
    local script="$2"
    ssh_robot "${ip}" "bash -lc $(printf '%q' "${script}")"
}

ssh_mocap() {
    local target="$1"
    shift
    local pass="${MOCAP_PASS:-${PASS:-}}"
    if [ -n "${pass}" ]; then
        sshpass -p "${pass}" ssh -n "${SSH_OPTS[@]}" "${target}" "$@"
    else
        ssh -n -o BatchMode=yes "${SSH_OPTS[@]}" "${target}" "$@"
    fi
}

chrony_gate_script() {
    cat <<EOF
echo "--- chrony gate ---"
if ! command -v chronyc >/dev/null 2>&1; then
    echo "ERROR: chronyc is not installed or not on PATH"
    exit 40
fi

chrony_tracking=\$(chronyc tracking 2>&1) || {
    printf "%s\n" "\${chrony_tracking}"
    echo "ERROR: chronyc tracking failed"
    exit 41
}
printf "%s\n" "\${chrony_tracking}"

echo "--- chrony sources ---"
chronyc sources -v 2>&1 || true

leap_status=\$(printf "%s\n" "\${chrony_tracking}" | awk -F: '/Leap status/ {gsub(/^[ \t]+|[ \t]+$/, "", \$2); print \$2}')
if [ "\${leap_status}" != "Normal" ]; then
    echo "ERROR: chrony leap status is '\${leap_status}', expected 'Normal'"
    exit 42
fi

printf "%s\n" "\${chrony_tracking}" | awk -v max="${MAX_CHRONY_OFFSET_SEC}" '
function abs(x) { return x < 0 ? -x : x }
/^(System time|Last offset)[[:space:]]*:/ {
    value = \$4 + 0
    if (abs(value) > max) {
        printf("ERROR: %s %s exceeds max %.9f seconds; observed %.9f seconds\n", \$1, \$2, max, value) > "/dev/stderr"
        bad = 1
    }
}
END { exit bad ? 43 : 0 }
'

echo "chrony_gate=pass"
EOF
}

preflight_mocap() {
    if [ "${CHECK_MOCAP_CHRONY}" != "true" ]; then
        echo "mocap_chrony_check=skipped"
        return 0
    fi
    if [ -z "${MOCAP_TARGET}" ]; then
        echo "ERROR: CHECK_MOCAP_CHRONY=true but MOCAP_TARGET is empty."
        echo "Set MOCAP_TARGET=user@host for the machine recording PhaseSpace/mocap GT."
        echo "If GT is not being recorded for this test, run with CHECK_MOCAP_CHRONY=false."
        return 44
    fi

    local script
    script="$(chrony_gate_script)"
    ssh_mocap "${MOCAP_TARGET}" "bash -lc $(printf '%q' "${script}")"
}

wait_for_phase() {
    local rc=0
    local pid
    for pid in "$@"; do
        if ! wait "${pid}"; then
            rc=1
        fi
    done
    return "${rc}"
}

preflight_robot() {
    local idx="$1"
    local ip="${ROBOTS[$idx]}"
    local name="${ROBOT_NAMES[$idx]}"

    local remote
    local chrony_script
    chrony_script="$(chrony_gate_script)"

    remote=$(cat <<EOF
set -euo pipefail
${chrony_script}

cd "${REMOTE_ROOT}"
mkdir -p "\$HOME/agv_data"

stale_processes=\$(ps -eo pid=,args= | awk '
    /[r]osbag record|[s]tart_session[.]sh|[d]rive_circle[.]py/ &&
    \$0 !~ /bash -lc/ &&
    \$0 !~ /awk / {
        print
    }
')
if [ -n "\${stale_processes}" ]; then
    echo "ERROR: stale ROS recording/drive process is already running"
    printf "%s\n" "\${stale_processes}"
    exit 10
fi

free_gb=\$(df -BG "\$HOME/agv_data" | awk 'NR==2 {gsub("G","",\$4); print \$4}')
if [ "\${free_gb}" -lt "${MIN_FREE_GB}" ]; then
    echo "ERROR: only \${free_gb}GB free; need at least ${MIN_FREE_GB}GB"
    exit 11
fi

test -f scripts/logging/start_session.sh
test -f scripts/logging/drive_circle.py
test -f scripts/logging/validate_bag.py

echo "robot=${name}"
echo "ip=${ip}"
echo "repo=\$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
echo "free_gb=\${free_gb}"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
    rs-enumerate-devices | egrep "Firmware Version|Recommended Firmware Version|Usb Type Descriptor" || true
fi
EOF
)
    remote_bash "${ip}" "${remote}"
}

start_recording_robot() {
    local idx="$1"
    local ip="${ROBOTS[$idx]}"
    local name="${ROBOT_NAMES[$idx]}"
    local log_file="/tmp/${SCENARIO}_${name}_start_session.log"
    local pid_file="/tmp/${SCENARIO}_${name}_start_session.pid"
    local bag_path_file="/tmp/${SCENARIO}_${name}_bag_path.txt"
    local pattern="[r]osbag record.*${name}_${SCENARIO}_"

    local remote
    remote=$(cat <<EOF
set -euo pipefail
cd "${REMOTE_ROOT}"

rm -f "${pid_file}" "${log_file}" "${bag_path_file}"
mkdir -p "\$HOME/agv_data"

env \
    REQUIRE_IMU="${REQUIRE_IMU}" \
    REQUIRE_GT="${REQUIRE_GT}" \
    ENABLE_APRILTAG="${ENABLE_APRILTAG}" \
    CAMERA_RGB_AUTO_EXPOSURE="${CAMERA_RGB_AUTO_EXPOSURE}" \
    CAMERA_RGB_EXPOSURE="${CAMERA_RGB_EXPOSURE}" \
    CAMERA_RGB_GAIN="${CAMERA_RGB_GAIN}" \
    bash scripts/logging/start_session.sh "${name}" "${SCENARIO}" \
    > "${log_file}" 2>&1 &
echo \$! > "${pid_file}"

deadline=\$((SECONDS + ${STARTUP_TIMEOUT_SEC}))
while [ "\${SECONDS}" -lt "\${deadline}" ]; do
    if pgrep -f "${pattern}" >/dev/null 2>&1; then
        bag_path=\$(ps -eo args | awk '\$0 ~ /[r]osbag record/ && \$0 ~ /${name}_${SCENARIO}_/ {for (i=1;i<=NF;i++) if (\$i=="-O") print \$(i+1)}' | head -1)
        if [ -z "\${bag_path}" ]; then
            echo "ERROR: rosbag is live but bag path could not be extracted"
            exit 22
        fi
        echo "\${bag_path}" > "${bag_path_file}"
        echo "rosbag_live=true"
        echo "bag=\${bag_path}"
        echo "session_pid=\$(cat "${pid_file}")"
        exit 0
    fi
    if ! kill -0 "\$(cat "${pid_file}")" 2>/dev/null; then
        echo "ERROR: start_session exited before rosbag became live"
        tail -120 "${log_file}" || true
        exit 20
    fi
    sleep 2
done

echo "ERROR: timed out waiting for rosbag"
tail -160 "${log_file}" || true
exit 21
EOF
)
    remote_bash "${ip}" "${remote}"
}

drive_robot() {
    local idx="$1"
    local ip="${ROBOTS[$idx]}"
    local name="${ROBOT_NAMES[$idx]}"
    local radius="${RADII[$idx]}"
    local offset=$((idx * STAGGER))

    local remote
    remote=$(cat <<EOF
set -eo pipefail
cd "${REMOTE_ROOT}"

set +u
if [ -n "\${ROS_DISTRO:-}" ] && [ -f "/opt/ros/\${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/\${ROS_DISTRO}/setup.bash"
elif [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
fi
[ -f myagv_ros/devel/setup.bash ] && source myagv_ros/devel/setup.bash
source agv_ws/devel/setup.bash
set -u

scheduled_start=$(( ${BASE_START_EPOCH} + ${offset} ))
seconds_to_start=$(( scheduled_start - \$(date +%s) ))
if [ "\${seconds_to_start}" -lt "${MIN_START_MARGIN_SEC}" ]; then
    echo "ERROR: scheduled start for ${name} is only \${seconds_to_start}s away; need at least ${MIN_START_MARGIN_SEC}s"
    exit 50
fi

python3 -u scripts/logging/drive_circle.py \
    --radius "${radius}" \
    --linear "${LINEAR}" \
    --duration "${DURATION}" \
    --start-at-epoch "${BASE_START_EPOCH}" \
    --start-delay "${offset}" \
    --no-prompt \
    --verbose
EOF
)
    echo "[${name}] driving radius=${radius}m, start_offset=${offset}s"
    remote_bash "${ip}" "${remote}"
}

stop_recording_robot() {
    local idx="$1"
    local ip="${ROBOTS[$idx]}"
    local name="${ROBOT_NAMES[$idx]}"
    local pid_file="/tmp/${SCENARIO}_${name}_start_session.pid"
    local pattern="[r]osbag record.*${name}_${SCENARIO}_"

    local remote
    remote=$(cat <<EOF
set -eo pipefail

set +u
if [ -n "\${ROS_DISTRO:-}" ] && [ -f "/opt/ros/\${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/\${ROS_DISTRO}/setup.bash"
elif [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
fi
set -u

if pgrep -f "${pattern}" >/dev/null 2>&1; then
    pkill -INT -f "${pattern}" || true
    for _ in \$(seq 1 45); do
        if ! pgrep -f "${pattern}" >/dev/null 2>&1; then
            break
        fi
        sleep 1
    done
fi

if [ -f "${pid_file}" ]; then
    session_pid=\$(cat "${pid_file}")
    kill -INT "\${session_pid}" 2>/dev/null || true
    for _ in \$(seq 1 20); do
        if ! kill -0 "\${session_pid}" 2>/dev/null; then
            break
        fi
        sleep 1
    done
fi

if pgrep -f "${pattern}" >/dev/null 2>&1; then
    pkill -TERM -f "${pattern}" || true
    sleep 2
fi

if pgrep -f "drive_circle.py" >/dev/null 2>&1; then
    pkill -INT -f "drive_circle.py" || true
    sleep 1
fi

if command -v rostopic >/dev/null 2>&1; then
    rostopic pub -1 /cmd_vel geometry_msgs/Twist "{}" >/dev/null 2>&1 || true
fi

rm -f "${pid_file}"
echo "stopped=true"
EOF
)
    remote_bash "${ip}" "${remote}"
}

validate_robot_bag() {
    local idx="$1"
    local ip="${ROBOTS[$idx]}"
    local name="${ROBOT_NAMES[$idx]}"
    local bag_path_file="/tmp/${SCENARIO}_${name}_bag_path.txt"

    local gt_arg=""
    if [ "${REQUIRE_GT}" = "true" ]; then
        gt_arg="--require-gt"
    fi
    local imu_arg=""
    if [ "${REQUIRE_IMU}" = "true" ]; then
        imu_arg="--require-imu"
    fi

    local remote
    remote=$(cat <<EOF
set -euo pipefail
cd "${REMOTE_ROOT}"

bag=\$(cat "${bag_path_file}" 2>/dev/null || true)
if [ -z "\${bag}" ]; then
    echo "ERROR: no current-run bag path recorded for ${name}_${SCENARIO}"
    exit 30
fi
if [ ! -f "\${bag}" ]; then
    echo "ERROR: current-run bag was not finalized: \${bag}"
    exit 30
fi
echo "bag=\${bag}"

validation_log="/tmp/${SCENARIO}_${name}_validate.log"
set +e
python3 scripts/logging/validate_bag.py "\${bag}" ${imu_arg} ${gt_arg} | tee "\${validation_log}"
validator_rc=\${PIPESTATUS[0]}
set -e

if [ "\${validator_rc}" -ne 0 ]; then
    echo "ERROR: validate_bag.py failed with rc=\${validator_rc}"
    exit "\${validator_rc}"
fi

if [ "${NO_FRAME_DROPS}" = "true" ] && grep -Eq "/camera/(color|aligned_depth_to_color)/image_raw gaps: [1-9][0-9]* drops" "\${validation_log}"; then
    echo "ERROR: camera frame drops detected"
    exit 31
fi

echo "validation_passed=true"
EOF
)
    remote_bash "${ip}" "${remote}"
}

cleanup_on_interrupt() {
    echo ""
    echo "Interrupted; stopping motion and recording on all robots..."
    for i in "${!ROBOTS[@]}"; do
        stop_recording_robot "${i}" >/dev/null 2>&1 &
    done
    wait || true
    exit 130
}

require_cmd sshpass
require_cmd ssh

[ "${#ROBOTS[@]}" -gt 0 ] || usage_error "at least one robot is required"
[ "${#ROBOTS[@]}" -eq "${#ROBOT_NAMES[@]}" ] || usage_error "ROBOTS and ROBOT_NAMES length mismatch"
[ "${#ROBOTS[@]}" -eq "${#RADII[@]}" ] || usage_error "ROBOTS and RADII length mismatch"

validate_token "SCENARIO" "${SCENARIO}"
validate_number "LINEAR" "${LINEAR}"
validate_number "DURATION" "${DURATION}"
validate_int "STAGGER" "${STAGGER}"
validate_int "START_LEAD_SEC" "${START_LEAD_SEC}"
validate_int "MIN_START_MARGIN_SEC" "${MIN_START_MARGIN_SEC}"
validate_number "POST_ROLL_SEC" "${POST_ROLL_SEC}"
validate_int "STARTUP_TIMEOUT_SEC" "${STARTUP_TIMEOUT_SEC}"
validate_int "MIN_FREE_GB" "${MIN_FREE_GB}"
validate_number "MAX_CHRONY_OFFSET_SEC" "${MAX_CHRONY_OFFSET_SEC}"

for i in "${!ROBOTS[@]}"; do
    validate_token "robot IP" "${ROBOTS[$i]}"
    validate_token "robot name" "${ROBOT_NAMES[$i]}"
    validate_number "radius" "${RADII[$i]}"
done

read -rsp "Password: " PASS
echo ""

mkdir -p "${LOG_ROOT}"
trap cleanup_on_interrupt INT TERM

cat > "${LOG_ROOT}/run_plan.txt" <<EOF
scenario=${SCENARIO}
robots=${ROBOTS[*]}
robot_names=${ROBOT_NAMES[*]}
radii=${RADII[*]}
linear=${LINEAR}
duration=${DURATION}
stagger=${STAGGER}
start_lead_sec=${START_LEAD_SEC}
min_start_margin_sec=${MIN_START_MARGIN_SEC}
post_roll_sec=${POST_ROLL_SEC}
require_imu=${REQUIRE_IMU}
require_gt=${REQUIRE_GT}
enable_apriltag=${ENABLE_APRILTAG}
check_mocap_chrony=${CHECK_MOCAP_CHRONY}
mocap_target=${MOCAP_TARGET}
max_chrony_offset_sec=${MAX_CHRONY_OFFSET_SEC}
camera_rgb_auto_exposure=${CAMERA_RGB_AUTO_EXPOSURE}
camera_rgb_exposure=${CAMERA_RGB_EXPOSURE}
camera_rgb_gain=${CAMERA_RGB_GAIN}
remote_root=${REMOTE_ROOT}
EOF

echo "== Phase 0: mocap chrony preflight =="
if ! preflight_mocap > "${LOG_ROOT}/mocap_preflight.log" 2>&1; then
    echo "ERROR: mocap chrony preflight failed; see ${LOG_ROOT}/mocap_preflight.log" >&2
    exit 44
fi

echo "== Phase 1: preflight =="
pids=()
for i in "${!ROBOTS[@]}"; do
    (
        preflight_robot "${i}"
    ) > "${LOG_ROOT}/${ROBOT_NAMES[$i]}_preflight.log" 2>&1 &
    pids+=("$!")
done
preflight_rc=0
wait_for_phase "${pids[@]}" || preflight_rc=$?
if [ "${preflight_rc}" -ne 0 ]; then
    echo "ERROR: preflight failed; see ${LOG_ROOT}/*_preflight.log" >&2
    exit "${preflight_rc}"
fi
echo "preflight logs: ${LOG_ROOT}"

echo "== Phase 2: start recording =="
pids=()
for i in "${!ROBOTS[@]}"; do
    (
        start_recording_robot "${i}"
    ) > "${LOG_ROOT}/${ROBOT_NAMES[$i]}_recording_start.log" 2>&1 &
    pids+=("$!")
done
record_rc=0
wait_for_phase "${pids[@]}" || record_rc=$?
if [ "${record_rc}" -ne 0 ]; then
    echo "ERROR: recording did not start cleanly; stopping any sessions that did start" >&2
    for i in "${!ROBOTS[@]}"; do
        stop_recording_robot "${i}" >/dev/null 2>&1 &
    done
    wait || true
    echo "See ${LOG_ROOT}/*_recording_start.log" >&2
    exit "${record_rc}"
fi
echo "all rosbags are live"

BASE_START_EPOCH="$(( $(date +%s) + START_LEAD_SEC ))"
export BASE_START_EPOCH
echo "base_start_epoch=${BASE_START_EPOCH}"
echo "first motion begins in ${START_LEAD_SEC}s"

echo "== Phase 3: staggered circle drives =="
pids=()
for i in "${!ROBOTS[@]}"; do
    (
        drive_robot "${i}"
    ) > "${LOG_ROOT}/${ROBOT_NAMES[$i]}_drive.log" 2>&1 &
    pids+=("$!")
done
drive_rc=0
wait_for_phase "${pids[@]}" || drive_rc=$?

echo "== Phase 4: synchronized recording stop =="
sleep "${POST_ROLL_SEC}"
pids=()
for i in "${!ROBOTS[@]}"; do
    (
        stop_recording_robot "${i}"
    ) > "${LOG_ROOT}/${ROBOT_NAMES[$i]}_stop.log" 2>&1 &
    pids+=("$!")
done
wait_for_phase "${pids[@]}" || true

echo "== Phase 4.5: mocap chrony postflight =="
if ! preflight_mocap > "${LOG_ROOT}/mocap_postflight.log" 2>&1; then
    echo "ERROR: mocap chrony postflight failed; see ${LOG_ROOT}/mocap_postflight.log" >&2
    exit 45
fi

if [ "${drive_rc}" -ne 0 ]; then
    echo "ERROR: one or more drive commands failed; see ${LOG_ROOT}/*_drive.log" >&2
    exit "${drive_rc}"
fi

echo "== Phase 5: bag validation =="
pids=()
for i in "${!ROBOTS[@]}"; do
    (
        validate_robot_bag "${i}"
    ) > "${LOG_ROOT}/${ROBOT_NAMES[$i]}_validate.log" 2>&1 &
    pids+=("$!")
done
validate_rc=0
wait_for_phase "${pids[@]}" || validate_rc=$?
if [ "${validate_rc}" -ne 0 ]; then
    echo "ERROR: validation failed; see ${LOG_ROOT}/*_validate.log" >&2
    exit "${validate_rc}"
fi

echo "Mission complete. Logs: ${LOG_ROOT}"
