#!/usr/bin/env python3
"""Audit a collected dataset run from reports, bags, and manifests.

This command ties together the lower-level diagnostics:

- robot_doctor summary.json validation, including copied evidence files
- dataset_ready checks for post-run doctor reports
- ROS 1 / ROS 2 bag validation
- start_session manifest completeness

It is intended to run on the laptop after copying artifacts back from robots.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/diagnostics"))

from validate_robot_doctor_report import validate_report  # noqa: E402


PASS = "PASS"
WARN = "WARN"
FAIL = "FAIL"


@dataclass
class AuditItem:
    status: str
    check: str
    artifact: str
    summary: str
    next_action: str = ""


def expand_paths(values: Sequence[str]) -> List[Path]:
    paths: List[Path] = []
    for value in values:
        path = Path(value).expanduser()
        matches = sorted(path.parent.glob(path.name)) if any(ch in value for ch in "*?[") else []
        if matches:
            paths.extend(matches)
        else:
            paths.append(path)
    return sorted(dict.fromkeys(paths))


def status_from_rc(rc: int) -> str:
    if rc == 0:
        return PASS
    if rc == 2:
        return WARN
    return FAIL


def print_table(items: Iterable[AuditItem]) -> None:
    rows = list(items)
    if not rows:
        print("No audit items.")
        return
    widths = {
        "status": max(len("status"), *(len(item.status) for item in rows)),
        "check": max(len("check"), *(len(item.check) for item in rows)),
        "artifact": min(80, max(len("artifact"), *(len(item.artifact) for item in rows))),
    }
    print(f"{'status'.ljust(widths['status'])} | {'check'.ljust(widths['check'])} | {'artifact'.ljust(widths['artifact'])} | summary")
    print(f"{'-' * widths['status']}-+-{'-' * widths['check']}-+-{'-' * widths['artifact']}-+--------")
    for item in rows:
        artifact = item.artifact
        if len(artifact) > widths["artifact"]:
            artifact = "..." + artifact[-(widths["artifact"] - 3):]
        print(f"{item.status.ljust(widths['status'])} | {item.check.ljust(widths['check'])} | {artifact.ljust(widths['artifact'])} | {item.summary}")
        if item.next_action:
            print(f"{''.ljust(widths['status'])} | {''.ljust(widths['check'])} | {''.ljust(widths['artifact'])} | next: {item.next_action}")


def load_report(path: Path) -> Tuple[Optional[dict], Optional[str]]:
    if path.is_dir():
        path = path / "summary.json"
    try:
        return json.loads(path.read_text()), None
    except Exception as exc:
        return None, str(exc)


def audit_reports(paths: Sequence[Path], require_ready: bool, require_configured_gate: bool) -> List[AuditItem]:
    items: List[AuditItem] = []
    if not paths:
        items.append(
            AuditItem(
                FAIL,
                "reports_present",
                "-",
                "no robot_doctor reports supplied",
                "run robot_doctor before and after recording, then pass copied summary.json files to this audit",
            )
        )
        return items

    gate_values = set()
    config_values = set()
    for raw_path in paths:
        path = raw_path / "summary.json" if raw_path.is_dir() else raw_path
        report, error = load_report(path)
        if error or report is None:
            items.append(AuditItem(FAIL, "report_read", str(path), f"cannot read report: {error}", "copy the complete diagnostic report directory again"))
            continue

        ok, errors = validate_report(report, check_evidence=True, summary_json=path)
        if ok:
            items.append(AuditItem(PASS, "report_schema", str(path), "report schema and copied evidence are valid"))
        else:
            items.append(AuditItem(FAIL, "report_schema", str(path), "; ".join(errors[:5]), "rerun or recopy robot_doctor evidence until the report validates"))
            continue

        loaded_config = report.get("loaded_config", {}) if isinstance(report.get("loaded_config"), dict) else {}
        gate_id = loaded_config.get("gate_id", "")
        gate_version = loaded_config.get("gate_version", "")
        config_sha = report.get("config_sha256", "")
        gate_values.add((gate_id, gate_version))
        config_values.add(config_sha)

        if require_configured_gate and (not gate_id or not gate_version or not config_sha):
            items.append(
                AuditItem(
                    FAIL,
                    "report_configured_gate",
                    str(path),
                    "report is missing gate_id, gate_version, or config_sha256",
                    "rerun robot_doctor with configs/robot_doctor_dataset_gate.json",
                )
            )
        else:
            items.append(AuditItem(PASS, "report_configured_gate", str(path), gate_id or "configured gate not required"))

        if require_ready and not report.get("dataset_ready"):
            blocker = report.get("decision", {}).get("primary_blocker", {}) if isinstance(report.get("decision"), dict) else {}
            summary = blocker.get("summary") if isinstance(blocker, dict) else report.get("verdict", "not ready")
            next_action = blocker.get("next_action", "") if isinstance(blocker, dict) else "fix report blockers and rerun"
            items.append(
                AuditItem(
                    FAIL,
                    "report_dataset_ready",
                    str(path),
                    f"dataset_ready is false: {summary}",
                    next_action or "fix report blockers and rerun robot_doctor",
                )
            )
        else:
            items.append(AuditItem(PASS, "report_dataset_ready", str(path), f"dataset_ready={report.get('dataset_ready')}"))

    if len(gate_values - {("", "")}) > 1:
        items.append(
            AuditItem(
                FAIL,
                "fleet_same_gate",
                ",".join(str(path) for path in paths),
                f"reports use different gates: {sorted(gate_values)}",
                "rerun all robots against the same dataset gate config",
            )
        )
    if len(config_values - {""}) > 1:
        items.append(
            AuditItem(
                FAIL,
                "fleet_same_config",
                ",".join(str(path) for path in paths),
                f"reports use different config hashes: {sorted(config_values)}",
                "rerun all robots against the same committed config",
            )
        )
    return items


def bag_kind(path: Path) -> str:
    if path.is_file() and path.suffix == ".bag":
        return "ros1"
    if path.is_file() and path.suffix == ".db3":
        return "ros2"
    if path.is_dir() and list(path.glob("*.db3")):
        return "ros2"
    return "unknown"


def run_command(command: Sequence[str], env: Optional[Dict[str, str]] = None) -> subprocess.CompletedProcess:
    merged_env = os.environ.copy()
    if env:
        merged_env.update(env)
    return subprocess.run(
        list(command),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        cwd=str(ROOT),
        env=merged_env,
    )


def audit_bags(
    paths: Sequence[Path],
    output_dir: Path,
    require_bag: bool,
    require_gt: bool,
    require_imu: bool,
    min_duration: float,
    required_topics: Sequence[str],
    mocap_topic: str,
    cmd_topic: str,
) -> List[AuditItem]:
    items: List[AuditItem] = []
    if not paths:
        status = FAIL if require_bag else WARN
        items.append(
            AuditItem(
                status,
                "bags_present",
                "-",
                "no bag paths supplied",
                "pass every robot bag path to dataset_run_audit before declaring the run publishable",
            )
        )
        return items

    output_dir.mkdir(parents=True, exist_ok=True)
    for path in paths:
        kind = bag_kind(path)
        if kind == "unknown":
            items.append(
                AuditItem(
                    FAIL,
                    "bag_kind",
                    str(path),
                    "not a ROS1 .bag, ROS2 .db3, or rosbag2 directory",
                    "pass the completed bag file or rosbag2 directory",
                )
            )
            continue

        if kind == "ros2":
            json_out = output_dir / (path.name.replace(".", "_") + "_validate_ros2_bag.json")
            command = [
                sys.executable,
                str(ROOT / "scripts/logging/validate_ros2_bag.py"),
                str(path),
                "--min-duration",
                str(min_duration),
                "--json-out",
                str(json_out),
            ]
            if require_gt:
                command.append("--require-gt")
            if require_imu:
                command.append("--require-imu")
            env = {}
            if required_topics:
                env["REQUIRED_TOPICS"] = " ".join(required_topics)
            if mocap_topic:
                env["MOCAP_TOPIC"] = mocap_topic
            if cmd_topic:
                env["CMD_TOPIC"] = cmd_topic
            proc = run_command(command, env=env)
            log_path = output_dir / (path.name.replace(".", "_") + "_validate_ros2_bag.log")
            log_path.write_text(proc.stdout)
            status = status_from_rc(proc.returncode)
            summary = f"validate_ros2_bag rc={proc.returncode}"
            if json_out.exists():
                try:
                    data = json.loads(json_out.read_text())
                    if data.get("verdict") in {PASS, WARN, FAIL}:
                        status = data["verdict"]
                    duration = data.get("duration_sec")
                    duration_text = f"{float(duration):.1f}s" if isinstance(duration, (int, float)) else "unknown"
                    summary = f"verdict={data.get('verdict')} duration={duration_text} counts={data.get('counts')}"
                except Exception:
                    pass
            items.append(
                AuditItem(
                    status,
                    "bag_validation",
                    str(path),
                    summary,
                    "inspect bag validation log/json and fix missing or low-rate topics" if status == FAIL else "",
                )
            )
        else:
            command = [sys.executable, str(ROOT / "scripts/logging/validate_bag.py"), str(path)]
            if require_gt:
                command.append("--require-gt")
            if require_imu:
                command.append("--require-imu")
            env = {}
            if mocap_topic:
                env["MOCAP_TOPIC"] = mocap_topic
            if cmd_topic:
                env["CMD_TOPIC"] = cmd_topic
            proc = run_command(command, env=env)
            log_path = output_dir / (path.name.replace(".", "_") + "_validate_bag.log")
            log_path.write_text(proc.stdout)
            status = status_from_rc(proc.returncode)
            items.append(
                AuditItem(
                    status,
                    "bag_validation",
                    str(path),
                    f"validate_bag rc={proc.returncode}",
                    "inspect bag validation log and fix missing or low-rate topics" if status == FAIL else "",
                )
            )
    return items


def parse_manifest(path: Path) -> Dict[str, str]:
    data: Dict[str, str] = {}
    for line in path.read_text(errors="replace").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        data[key.strip()] = value.strip().strip('"').strip("'")
    return data


def incomplete(value: str) -> bool:
    return value in {"", "~", "unknown", "null", "None"}


def audit_manifests(paths: Sequence[Path], require_manifest: bool) -> List[AuditItem]:
    items: List[AuditItem] = []
    if not paths:
        status = FAIL if require_manifest else WARN
        items.append(
            AuditItem(
                status,
                "manifests_present",
                "-",
                "no session manifests supplied",
                "copy *_manifest.yaml files with the bags and include them in the audit",
            )
        )
        return items

    required_keys = ["session_id", "robot_id", "scenario", "date", "time_start", "time_end", "bag_file", "duration_sec", "bag_size_mb"]
    for path in paths:
        try:
            data = parse_manifest(path)
        except Exception as exc:
            items.append(AuditItem(FAIL, "manifest_read", str(path), f"cannot read manifest: {exc}", "copy the complete manifest again"))
            continue
        missing = [key for key in required_keys if incomplete(data.get(key, ""))]
        if missing:
            items.append(
                AuditItem(
                    FAIL,
                    "manifest_complete",
                    str(path),
                    "missing/incomplete fields: " + ", ".join(missing),
                    "ensure start_session.sh shut down cleanly and finalized the manifest",
                )
            )
            continue
        try:
            duration = float(data["duration_sec"])
        except Exception:
            duration = 0.0
        if duration <= 0:
            items.append(AuditItem(FAIL, "manifest_duration", str(path), f"duration_sec={data['duration_sec']}", "rerun or repair manifest duration"))
        else:
            items.append(AuditItem(PASS, "manifest_complete", str(path), f"{data['session_id']} duration={duration:.1f}s"))

        bag_ref = path.parent / data["bag_file"]
        if bag_ref.exists():
            items.append(AuditItem(PASS, "manifest_bag_reference", str(path), f"bag_file exists: {bag_ref.name}"))
        else:
            items.append(
                AuditItem(
                    WARN,
                    "manifest_bag_reference",
                    str(path),
                    f"bag_file does not exist beside manifest: {data['bag_file']}",
                    "verify the bag was copied with the manifest or pass the bag explicitly with --bag",
                )
            )
    return items


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", action="append", default=[], help="robot_doctor summary.json or report directory; may be repeated/globbed")
    parser.add_argument("--bag", action="append", default=[], help="ROS1 .bag, ROS2 .db3, or rosbag2 directory; may be repeated/globbed")
    parser.add_argument("--manifest", action="append", default=[], help="start_session *_manifest.yaml; may be repeated/globbed")
    parser.add_argument("--output-dir", default="diagnostic_reports/dataset_run_audits/latest")
    parser.add_argument("--json-out", help="write machine-readable audit JSON")
    parser.add_argument("--min-duration", type=float, default=30.0)
    parser.add_argument("--required-topic", action="append", default=[])
    parser.add_argument("--mocap-topic", default=os.environ.get("MOCAP_TOPIC", ""))
    parser.add_argument("--cmd-topic", default=os.environ.get("CMD_TOPIC", ""))
    parser.add_argument("--require-gt", action="store_true")
    parser.add_argument("--require-imu", action="store_true")
    parser.add_argument("--no-require-bag", action="store_true")
    parser.add_argument("--no-require-manifest", action="store_true")
    parser.add_argument("--allow-non-ready-reports", action="store_true")
    parser.add_argument("--no-require-configured-gate", action="store_true")
    parser.add_argument("--strict", action="store_true", help="exit nonzero on WARN as well as FAIL")
    args = parser.parse_args()

    report_paths = expand_paths(args.report)
    bag_paths = expand_paths(args.bag)
    manifest_paths = expand_paths(args.manifest)
    output_dir = Path(args.output_dir).expanduser()

    items: List[AuditItem] = []
    items.extend(
        audit_reports(
            report_paths,
            require_ready=not args.allow_non_ready_reports,
            require_configured_gate=not args.no_require_configured_gate,
        )
    )
    items.extend(
        audit_bags(
            bag_paths,
            output_dir,
            require_bag=not args.no_require_bag,
            require_gt=args.require_gt,
            require_imu=args.require_imu,
            min_duration=args.min_duration,
            required_topics=args.required_topic,
            mocap_topic=args.mocap_topic,
            cmd_topic=args.cmd_topic,
        )
    )
    items.extend(audit_manifests(manifest_paths, require_manifest=not args.no_require_manifest))

    print_table(items)
    counts = {PASS: 0, WARN: 0, FAIL: 0}
    for item in items:
        counts[item.status] = counts.get(item.status, 0) + 1
    verdict = FAIL if counts[FAIL] else (WARN if counts[WARN] else PASS)
    print("")
    print(f"Verdict: {verdict}  PASS={counts[PASS]} WARN={counts[WARN]} FAIL={counts[FAIL]}")

    out = {
        "created_at": datetime.utcnow().isoformat() + "Z",
        "verdict": verdict,
        "counts": counts,
        "reports": [str(path) for path in report_paths],
        "bags": [str(path) for path in bag_paths],
        "manifests": [str(path) for path in manifest_paths],
        "items": [asdict(item) for item in items],
    }
    if args.json_out:
        json_out = Path(args.json_out).expanduser()
        json_out.parent.mkdir(parents=True, exist_ok=True)
        json_out.write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")

    if counts[FAIL] or (args.strict and counts[WARN]):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
