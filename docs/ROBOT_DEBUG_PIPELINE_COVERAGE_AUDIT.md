# Robot Debug Pipeline Coverage Audit

This answers the questions from `ORKAR_Debug_Pipeline_Codex_Context.docx`.

## Coverage: G1-G5

| Gap | Current coverage after this pass | Remaining operational note |
|---|---|---|
| G1: D455 IMU is separate from RGB-D | Covered by `1.1 d455_imu_hid` and `1.1 realsense_motion_stream_gate`. The motion-only pyrealsense probe is enabled by the dataset gate when IMU is required. | Keep deciding per dataset whether the publishable IMU is base `/imu` or D455 `/camera/imu`; the doctor can prove both live ROS topics and the D455 HID path. |
| G2: USB autosuspend | Covered by `2.1 d455_usb_autosuspend` and `1.2 d455_usb_autosuspend_delay`. The fix script and ROS2 setup now set both `power/control=on` and `autosuspend_delay_ms=-1`. | Power-cycle after applying udev rules so the setting is proven from boot, not just live sysfs. |
| G3: viewer passes but ROS2 fails | Covered by `2.1 viewer_passes_ros2_fails`. In headless SSH operation, the standalone pyrealsense stream gate is the viewer-equivalent proof; if that passes but ROS2 camera topics are absent, the fault is classified as ROS2 wrapper/udev/version/launch. | A GUI `realsense-viewer` run can still be kept as manual evidence, but it is not required for robot_doctor automation. |
| G4: clock sync threshold and pre/post logging | Covered by `3.3 chrony_offset`, using `max_clock_offset_ms` from the dataset gate, default `1.0`. `start_session.sh` logs Chrony before and after the run. | Fleet-level offset topology still needs all robots pointed at the same lab NTP/Chrony source. |
| G5: D455 infra FPS cap | Covered by `2.2 d455_infra_fps_cap`. At the current 15 FPS dataset gate it passes as not limiting; if higher FPS is requested and infra topics cap around 15 Hz, the doctor reports the exact ROS2 param fix. | The current publishable gate is RGB-D 640x480 at 15 Hz, so this is a guard for future higher-FPS profiles. |

## Architecture

The 3-branch structure remains the right architecture:

1. Robot platform: prove physical signal generation first.
2. Robot data stack: prove drivers, kernel, ROS graph, rates, and timing next.
3. Experiment dataset: prove the recorded artifact and experiment execution last.

The important rule is still no skipping: a branch-2 ROS failure is not meaningful until branch-1 USB/power/sensor checks are clean.

Additional failure modes found in the codebase that were not explicit in the DOCX:

- Wi-Fi management conflicts from manual `wpa_supplicant`/`dhclient`.
- Stale ROS/rosbag/realsense processes polluting a new session.
- Dirty repo/config drift across robots.
- Remote SSH interruption while diagnosing.
- Report/bag/manifest mismatch after copying artifacts.
- Missing ROS2 `metadata.yaml` or unsupported `.mcap` parser.

## Minimum robot_doctor Output Plan

The target output is produced in three places: printed at the end of
`robot_doctor`, embedded in `summary.md`, and written as `decision.txt` beside
`summary.json`:

```text
READY: <dataset_ready>
FAILED_STAGE: <decision.primary_blocker.code and failure-tree name>
CAUSE: <decision.primary_blocker.summary>
EVIDENCE: <decision.primary_blocker.evidence>
NEXT_ACTION: <decision.primary_blocker.next_action>
```

Implementation status:

- Every check is tagged with one of the 3x3 failure-tree codes.
- Every WARN/FAIL check must carry a `next_action`; the report validator enforces this.
- `decision.primary_blocker` picks the first hard failure, otherwise the first warning.
- `decision.txt` provides the operator-facing `READY / FAILED_STAGE / CAUSE / EVIDENCE / NEXT_ACTION` block requested by the context document.
- Remote wrapper failures synthesize a valid robot_doctor report instead of leaving missing evidence.
- `dataset_run_audit.py` ties robot reports, bags, and manifests together after collection.

## ROS2 Validator

The ROS2 validator now covers:

- `.db3` rosbag2 SQLite storage.
- `.mcap` storage when the optional Python MCAP reader is installed; otherwise it fails as `bag_integrity` with explicit evidence.
- Required topics, rates, gaps, stream coverage, storage timestamp monotonicity, GT, IMU, duration, and ROS2 `metadata.yaml`.
- Environment overrides for mocap, command, IMU, depth, and extra required topics.

Remaining optional improvement: deserialize selected message headers to prove message `header.stamp` monotonicity independently of rosbag storage timestamps.

## Fleet-Level Checks

These should be fleet-wide, not only per robot:

- Same dataset gate ID/version/config hash.
- Same repo commit and clean repo state.
- Same RealSense firmware, standalone librealsense, ROS wrapper, and ROS-node LibRealSense runtime.
- Same Chrony topology and max offset threshold.
- Same camera profile and required topic list.
- Same post-run artifact convention: one report, bag, and manifest per robot/session.

`fleet_doctor_summary.py` covers the report/config/commit side. `dataset_run_audit.py` covers the copied run artifacts. Chrony topology is enforced per robot by `chrony_offset` and should be reviewed as a fleet table before publishable runs.
