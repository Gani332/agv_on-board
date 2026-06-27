# Robot Diagnostic Pipeline

This pipeline operationalizes `robot_failure_modes_v3.png`.

The goal is not to make hardware failures impossible. The goal is that a robot is
either dataset-ready or the failure is classified with evidence under one of the
failure-tree branches.

## One-Command Preflight

For a freshly flashed ROS 2 robot, run the standard provisioning path first:

```bash
cd ~/slam_project
SUDO_PASSWORD=ubuntu bash scripts/setup_robot_ros2.sh agv102
```

That script installs/pins the RealSense tools used by the gate, checks
`pyrealsense2`, builds `agv2_ws`, installs the D455 autosuspend and UVC bind
udev rules, and
finishes with a static `robot_doctor` report.

The Intel RealSense apt repository has had a repository-signing-key mismatch
(`NO_PUBKEY FB0B24895113F120`) while still serving the expected packages. The
setup script therefore uses `REALSENSE_REPO_TRUST_MODE=auto`: try signed
verification first, then log an explicit fallback to `trusted=yes` and still
enforce the expected package versions. Use `REALSENSE_REPO_TRUST_MODE=signed`
when you want the setup to fail instead of falling back.

Run on the robot:

```bash
cd ~/slam_project
bash scripts/diagnostics/robot_doctor.sh agv102 --profile preflight
```

`robot_doctor` holds a per-robot lock while it runs. If a second diagnostic is
started on the same robot, it fails with `3.2 diagnostic_lock` instead of
running competing RealSense probes. Timed diagnostic commands are killed as a
process group, so a timeout should not leave child `rs-enumerate-devices`,
`ros2 topic hz`, or `python3` stream probes behind.

Before trusting edits to the diagnostic code, run the no-hardware regression
test on the laptop or robot:

```bash
python3 scripts/diagnostics/robot_doctor_selftest.py
```

Then run the acceptance audit for the diagnostic pipeline itself:

```bash
python3 scripts/diagnostics/diagnostic_pipeline_audit.py
```

This checks that the repo still contains the required doctor, validators,
remote wrappers, fleet audit, dataset gate, failure-tree coverage, evidence
validation hooks, and documentation needed to make robot failures diagnosable.

For a strict dataset gate before a publishable run:

```bash
cd ~/slam_project
bash scripts/diagnostics/robot_doctor.sh agv102 \
  --config configs/robot_doctor_dataset_gate.json \
  --mocap-topic /optitrack/rigid_bodies/orkar_agv102 \
  --cmd-topic /agv102/cmd_vel \
  --strict-ops \
  --confirm-mechanical \
  --confirm-mocap \
  --confirm-anchors
```

The command writes an evidence folder:

```text
~/agv_data/diagnostics/<robot_id>_<timestamp>/
  summary.md
  summary.json
  operator_mechanical_checklist.md
  operator_d455_swap_checklist.md   # only when D455 physical evidence is needed
  logs/
```

Use `summary.md` for quick decisions and `summary.json` for machine-readable
fleet audits.

Validate a report before relying on it:

```bash
python3 scripts/diagnostics/validate_robot_doctor_report.py \
  ~/agv_data/diagnostics/<robot_id>_<timestamp>/summary.json
```

After copying a remote report to the laptop, require its evidence logs too:

```bash
python3 scripts/diagnostics/validate_robot_doctor_report.py --check-evidence \
  diagnostic_reports/<robot_id>_<timestamp>/<robot_id>_<timestamp>/summary.json
```

The validator maps evidence paths under the robot's reported `output_dir` to the
local copied report directory, so remote absolute paths remain auditable after
`run_robot_doctor_remote.sh` copies the report back.

The validator fails if the report is internally inconsistent or missing
reproducibility fields such as `config_sha256`, `effective_gate`, `repo_state`,
or the failure-tree mapping. Every `FAIL` or `WARN` check must also include a
non-empty `next_action`; a non-actionable blocker is treated as an invalid
diagnostic report.

The `summary.json` decision contract is:

```text
decision.state = ready   -> no FAIL/WARN; dataset-ready under the configured gate
decision.state = review  -> no FAIL, but at least one WARN; okay for tests, not publishable yet
decision.state = blocked -> at least one FAIL; fix before further dataset collection
```

`decision.primary_blocker` is the first branch to fix. It contains the
failure-tree code, check name, evidence paths, and next action.

For a full post-run dataset audit on the laptop, validate copied robot reports,
bags, and session manifests together:

```bash
python3 scripts/diagnostics/dataset_run_audit.py \
  --report 'diagnostic_reports/agv*/agv*/summary.json' \
  --bag '/path/to/copied/bags/*' \
  --manifest '/path/to/copied/manifests/*_manifest.yaml' \
  --mocap-topic /optitrack/rigid_bodies/<rigid_body_name> \
  --cmd-topic /<robot_name>/cmd_vel \
  --require-gt \
  --require-imu \
  --strict \
  --json-out diagnostic_reports/dataset_run_audits/latest/summary.json
```

Use this as the final publishability check for a collected run. It fails if
robot_doctor evidence is invalid, reports are not `dataset_ready`, bags fail the
ROS1/ROS2 validators, manifests are incomplete, or the copied report/bag/manifest
artifacts do not match the same robot, scenario, and session identity.

`effective_gate` records the exact gate values used for that run. This is what
lets you prove all robots were tested against the same standard.

Every report also records:

- `schema_version`
- `tool_version`
- `config_sha256`
- `loaded_config`
- `effective_gate`
- `repo_state.branch`
- `repo_state.commit`
- `repo_state.dirty`

Archive these fields with dataset metadata so a report can be reproduced later.

## Post-Run Bag Diagnosis

ROS 1 bag:

```bash
bash scripts/diagnostics/robot_doctor.sh agv102 \
  --profile dataset \
  --require-bag \
  --bag ~/agv_data/<run>.bag \
  --require-gt \
  --require-imu
```

ROS 2 bag directory, `.db3`, or `.mcap`:

```bash
bash scripts/diagnostics/robot_doctor.sh agv102 \
  --profile dataset \
  --require-bag \
  --bag ~/agv_data/<ros2_bag_dir> \
  --require-gt \
  --require-imu
```

The doctor automatically dispatches to:

- `scripts/logging/validate_bag.py` for ROS 1 `.bag`
- `scripts/logging/validate_ros2_bag.py` for ROS 2 bag directories, `.db3`, and `.mcap`

If `--profile dataset` runs without `--bag`, the report is a partial gate and
`dataset_ready=false`. Add `--require-bag` for final post-run audits so missing
bag evidence is a hard failure instead of a warning.

## Failure-Tree Coverage

| Code | Branch | Automated evidence |
|---|---|---|
| `1.1` | Sensor device health | D455 USB presence, firmware, serial devices for LiDAR/base |
| `1.2` | Physical infrastructure | USB speed, USB disconnects, Pi throttle/brownout, power/thermal snapshots |
| `1.3` | Mechanical setup | Operator checklist for mounts, marker rigidity, wheels/chassis/slip |
| `2.1` | OS/kernel/USB | `/dev` permissions, librealsense visibility/control query, UVC/xHCI/dmesg classification, D455 `uvcvideo` binding, autosuspend/boot quirk |
| `2.2` | Drivers/launch config | librealsense packages, RealSense tools, RealSense ROS package/runtime versions, ROS environment, optional bringup command |
| `2.3` | ROS data quality | Topic presence, types, rates, IMU rate, live MoCap topic/rate |
| `3.1` | Recording pipeline | Disk space, stale recorders, bag path/readability |
| `3.2` | Validation pipeline | RealSense standalone stream gate, ROS1/ROS2 bag validation, metadata/rate/gap checks |
| `3.3` | Experiment execution | Clock sync, network/DNS, Wi-Fi management state, MoCap/operator confirmations, anchor/obstacle confirmations |

## Interpreting Results

The report uses four statuses:

- `PASS`: evidence proves the check is currently healthy.
- `WARN`: the robot may work, but the evidence is incomplete or below the dataset standard.
- `FAIL`: do not collect publishable data until fixed.
- `INFO`: context only.

Use this rule:

```text
can_run_tests=true, dataset_ready=false -> can run non-critical tests after reviewing WARNs
can_run_tests=true, dataset_ready=true  -> full dataset profile passed
can_run_tests=false                     -> fix the branch shown in summary.md
```

`dataset_ready=true` is only valid for a clean `--profile dataset` report. A
static, preflight, camera-only, or `--no-ros` run may prove a subsystem, but it
must not be treated as publishable dataset readiness until the dataset profile
also proves the required ROS graph/topics/rates and configured operator gates.

## RealSense Failure Diagnosis

The D455 checks deliberately separate six different failure modes:

1. `1.1 d455_enumeration`: camera is not seen as a USB device.
2. `2.1 d455_usb_speed`: camera is seen but not at USB3 / 5000 Mb/s.
3. `2.1 d455_uvc_binding`: camera is USB3-present, but Linux left the D455 Video interfaces unbound from `uvcvideo`.
4. `2.1 realsense_control_query`: `rs-enumerate-devices -c` fails or times out, so the issue is below ROS.
5. `2.2 realsense_ros_driver_version` / `2.2 realsense_ros_librealsense`: the installed ROS wrapper or the runtime SDK reported by the node differs from the fleet standard.
6. `2.3 topic_rate` or `3.2 bag_validation`: standalone camera is okay, but ROS/bag data is bad.

This prevents the common mistake of blaming ROS when the standalone RealSense
control path is already failing.

Kernel/UVC checks inspect a recent log window around the diagnostic run. That
keeps old boot-history messages from being treated as current evidence while
still catching errors triggered by `rs-enumerate-devices`, the standalone stream
gate, or live bringup.

If the combined standalone RGB-D stream gate fails, `robot_doctor` also runs
short isolation probes for color-only, depth-only, and motion-only when IMU is
required. This separates cases such as "color works but depth delivers zero
frames" from generic camera failure.

The current ROS 2 dataset gate standard is:

```text
D455 firmware:                  5.17.0.10
standalone librealsense tools:  2.58.1
RealSense ROS driver:           realsense2_camera 4.57.7
RealSense ROS node runtime:     LibRealSense 2.57.7
RGB-D stream gate:              640x480 at 15 Hz
USB gate:                       USB 3.x / 5000 Mb/s
```

## Common Failure Remediation

| Code / check | Meaning | First fix |
|---|---|---|
| `1.1 d455_enumeration` | Camera is not visible as a USB device | Reseat cable, check Pi USB3 port, swap known-good camera/cable |
| `2.1 d455_usb_speed` | Camera is visible but running below USB3 | Move to blue USB3 port, remove extension, replace cable |
| `2.1 d455_uvc_binding` | Camera is USB3-present but D455 Video interfaces are not bound to `uvcvideo`; RealSense tools may say no device | Run the targeted `d455-uvc-bind` fix, then rerun doctor |
| `1.1 d455_imu_hid` | D455 enumerates, but the separate HID/raw IMU path is not proven | Replug/reset D455, verify `hidraw` udev permissions, then run the standalone motion gate |
| `2.1 realsense_control_query` | `rs-enumerate-devices -c` fails, so the fault is below ROS | Try the targeted `d455-usb-reset` fix once, then inspect UVC/xHCI logs and do cable/port/camera swap if it persists |
| `2.1 realsense_depth_stream` / `2.1 realsense_color_stream` | Standalone per-stream isolation shows a specific video path does not deliver frames | Treat as below ROS; reset once, then collect cable/port/camera A/B evidence if repeatable |
| `1.1 realsense_motion_stream_gate` | Standalone D455 motion/IMU stream does not produce gyro/accel frames | Fix the HID/motion path before relying on camera IMU data |
| `2.1 realsense_stream_transport` | Standalone stream probe hangs or reports UVC/librealsense transport errors | Try the targeted `d455-usb-reset` fix once, then do cable/port/camera A/B swap if it persists |
| `2.1 viewer_passes_ros2_fails` | Standalone RealSense streaming passes, but ROS2 camera topics are absent | Check `realsense2_camera` launch, udev rules, wrapper version, and LibRealSense runtime |
| `1.2 d455_physical_swap_evidence` | A D455 physical-path failure exists but camera/cable/host-port A/B evidence is not recorded | Complete the generated D455 swap checklist, then rerun with swap confirmations or notes |
| `2.1 d455_usb_autosuspend` | Linux may autosuspend the D455 during long runs | Set D455 USB `power/control=on` and rerun doctor |
| `1.2 d455_usb_autosuspend_delay` | D455 `autosuspend_delay_ms` is not `-1` | Run the targeted `d455-autosuspend` fix, power-cycle, and rerun doctor |
| `2.2 realsense_tools` | RealSense tools are not installed | Run `SUDO_PASSWORD=ubuntu bash scripts/setup_robot_ros2.sh <robot_id> --skip-build`, or the targeted `realsense-standalone-tools` fix |
| `2.2 realsense_apt_source` | The Intel RealSense apt source is missing, duplicated, legacy, or using the explicit key-mismatch fallback | Rerun `scripts/setup_robot_ros2.sh`; `trusted=yes` is acceptable only when logged with expected package versions |
| `2.2 realsense_package_holds` | Installed RealSense packages are not pinned against accidental upgrades | Rerun `scripts/setup_robot_ros2.sh` or `apt-mark hold` the standardized packages |
| `2.2 realsense_python_binding` | `pyrealsense2` is missing or mismatched, so the standalone stream gate cannot run | Install the matching `pyrealsense2` binding before using the dataset gate |
| `2.2 librealsense_version` | Robot is not on fleet standard SDK | Run the pinned standalone RealSense fix before comparing data quality |
| `2.2 realsense_ros_driver_version` | ROS wrapper package/source version differs from the fleet standard | Reinstall/rebuild the standard `realsense2_camera` package |
| `2.2 realsense_ros_librealsense` | Running RealSense ROS node reports a different runtime SDK, or the runtime could not be proven | Restart bringup with logs and standardize the RealSense ROS build/runtime SDK |
| `2.2 d455_infra_fps_cap` | Infra stream is capped around 15 Hz when higher FPS was requested | Run `ros2 param set /camera/camera depth_module.enable_auto_exposure true`, restart the camera node, and rerun doctor |
| `2.2 dataset_bringup_context` | The dataset gate was run without live required sensor topics, or bringup ran but did not publish them | Start sensors first, or rerun doctor with `--bringup-cmd` so launch logs are captured |
| `2.3 topic_present` | ROS graph is missing a required stream | Fix launch/remap/driver before recording |
| `2.3 topic_rate` | Stream exists but rate is too low | Check CPU load, USB bandwidth, driver config, then lower load only if hardware is clean |
| `3.1 disk_free` | Bag recording may fail or truncate | Clear `~/agv_data` or use larger storage |
| `3.2 bag_validation` | Recorded data is missing/low-rate/corrupt | Use validator output; rerun only after fixing the failed branch |
| `3.3 clock_sync` / `3.3 chrony_offset` | Multi-robot timestamps are not trustworthy or parsed Chrony offset exceeds the gate threshold | Repair chrony/NTP before publishable collection |
| `3.3 wifi_management` | Manual `wpa_supplicant`/`dhclient` conflicts or unstable Wi-Fi management | Use one persistent NetworkManager/netplan path and reboot-test SSH |
| `3.3 remote_ssh_interrupted` | Remote wrapper lost SSH before robot_doctor wrote `summary.json` | Check Wi-Fi signal, robot power, and partial copied logs before rerunning |
| `3.3 mocap_topic` | Ground truth is absent or wrong | Fix OptiTrack/NatNet bridge, ROS domain, or rigid-body name |

## Targeted Remediation

Some findings have repeatable, low-risk fixes. These are intentionally separate
from `robot_doctor` and dry-run by default.

For `2.1 d455_usb_autosuspend`:

```bash
cd ~/slam_project
bash scripts/diagnostics/apply_robot_doctor_fix.sh --fix d455-autosuspend
bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-autosuspend
bash scripts/diagnostics/robot_doctor.sh agv102 --profile preflight
```

The fix writes a D455-specific udev rule and sets the currently connected D455
to `power/control=on`. It does not change any other USB device.
For non-interactive SSH automation, set `SUDO_PASSWORD=ubuntu` on the remote
command invocation.

For `2.1 d455_uvc_binding`:

```bash
cd ~/slam_project
bash scripts/diagnostics/apply_robot_doctor_fix.sh --fix d455-uvc-bind
SUDO_PASSWORD=ubuntu bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-uvc-bind
bash scripts/diagnostics/robot_doctor.sh agv102 --profile preflight
```

This writes a D455-specific udev rule and live-binds unbound D455 Video control
interfaces to `uvcvideo`. It is intended for the case where `lsusb` sees the
D455 at USB3 but `rs-enumerate-devices` says no device is detected.

For a transient `2.1 realsense_control_query` / `set_xu` UVC timeout where
`lsusb` still sees the D455:

```bash
cd ~/slam_project
bash scripts/diagnostics/apply_robot_doctor_fix.sh --fix d455-usb-reset
SUDO_PASSWORD=ubuntu bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-usb-reset
bash scripts/diagnostics/robot_doctor.sh agv102 --profile preflight
```

If the same control-query failure returns immediately after a USB reset, treat
it as persistent USB/camera/host-path evidence and move to the physical A/B swap
branch.

### D455 Physical A/B Swap Evidence

When the doctor sees a D455 physical-path failure, it writes
`operator_d455_swap_checklist.md` and adds `1.2 d455_physical_swap_evidence`.
This is not required for a healthy robot. It is required to make a persistent
D455 failure diagnosable instead of relying on a guess.

Use the generated checklist to test:

- known-good D455 camera on the suspect robot USB3 port
- suspect D455 camera on a known-good robot USB3 port
- known-good USB3 cable on the suspect robot
- suspect USB3 cable on a known-good robot
- suspect robot USB3 port with known-good camera and known-good cable

After completing the matrix, rerun:

```bash
bash scripts/diagnostics/robot_doctor.sh agv102 \
  --config configs/robot_doctor_dataset_gate.json \
  --confirm-d455-camera-swap \
  --confirm-d455-cable-swap \
  --confirm-d455-host-port-swap
```

If you keep a written swap note instead, pass it explicitly:

```bash
bash scripts/diagnostics/robot_doctor.sh agv102 \
  --config configs/robot_doctor_dataset_gate.json \
  --d455-swap-notes ~/agv_data/diagnostics/agv102_d455_swap_notes.md
```

## Optional Bringup Gate

If sensors are not already running, the doctor can launch bringup temporarily:

ROS 2 example:

```bash
bash scripts/diagnostics/robot_doctor.sh agv102 \
  --profile dataset \
  --ros ros2 \
  --bringup-cmd "ros2 launch agv_bringup bringup.launch.py" \
  --bringup-wait 45
```

ROS 1 example:

```bash
bash scripts/diagnostics/robot_doctor.sh agv1 \
  --profile dataset \
  --ros ros1 \
  --bringup-cmd "roslaunch agv_bringup bringup.launch" \
  --bringup-wait 45
```

The process is stopped at the end of the diagnostic run.

## Fleet Standardization

Before scaling to many robots, choose and document the fleet standard:

```bash
configs/robot_doctor_dataset_gate.json
```

Then run the same command on every robot and keep the `summary.json` files. A
robot is not considered equivalent to the fleet unless the version, USB, stream,
ROS topic, and bag gates all pass under the same command.

From the laptop, use the remote wrapper to deploy the latest doctor, run the
remote self-test, execute the diagnostic, and copy the evidence folder back:

```bash
SSH_PASS=ubuntu bash scripts/diagnostics/run_robot_doctor_remote.sh \
  agv102 192.168.0.71 -- \
  --config configs/robot_doctor_dataset_gate.json \
  --profile preflight
```

For multiple robots, create a host list:

```text
agv100 192.168.50.179
agv101 agv101.local
agv102 192.168.50.196
```

Then run the same gate over every robot:

```bash
SSH_PASS=ubuntu bash scripts/diagnostics/run_fleet_doctor_remote.sh hosts.txt --strict-fleet -- \
  --config configs/robot_doctor_dataset_gate.json \
  --profile preflight
```

The fleet runner continues after individual robot failures by default, copies
all available evidence into one local folder, and prints the aggregate
`fleet_doctor_summary.py` table at the end.
Wrapper-level fleet summary flags, such as `--strict-fleet`, go before the
separator `--`; robot doctor args go after it.

For a strict dataset gate over SSH:

```bash
SSH_PASS=ubuntu bash scripts/diagnostics/run_robot_doctor_remote.sh \
  agv102 192.168.0.71 -- \
  --config configs/robot_doctor_dataset_gate.json \
  --mocap-topic /optitrack/rigid_bodies/orkar_agv102 \
  --cmd-topic /agv102/cmd_vel \
  --strict-ops \
  --confirm-mechanical \
  --confirm-mocap \
  --confirm-anchors
```

To compare multiple robots from copied reports:

```bash
python3 scripts/diagnostics/fleet_doctor_summary.py \
  diagnostic_reports/agv*/*/summary.json
```

For final publishable collection, require every copied report to be
dataset-ready and comparable:

```bash
python3 scripts/diagnostics/fleet_doctor_summary.py \
  --strict-fleet \
  diagnostic_reports/agv*/*/summary.json
```

`fleet_doctor_summary.py` validates every report structure before printing the
table. `report_ok=false` means the report itself is incomplete or inconsistent,
so rerun the doctor before trusting that robot's result.

`--strict-fleet` fails if any robot is not `dataset_ready`, if any report is
missing a configured gate/config hash, if reports were generated with different
gate IDs/versions/config hashes, if repo commits differ, if any robot reports
a dirty worktree, or if copied evidence logs referenced by the reports are
missing. This is deliberately stricter than a debugging session; use it when
deciding whether a run is publishable.

## Known Limits

Some failures are physical and cannot be proven by software alone. The pipeline
therefore creates `operator_mechanical_checklist.md` and records whether the
operator confirmed it. In strict dataset mode, use:

```bash
--strict-ops --confirm-mechanical --confirm-mocap --confirm-anchors
```

That makes unconfirmed physical/setup checks hard failures instead of warnings.
For a robot with a D455 physical-path failure, strict ops also makes missing
D455 camera/cable/host-port swap evidence a hard failure.
