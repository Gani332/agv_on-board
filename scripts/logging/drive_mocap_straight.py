#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Drive a straight segment using mocap PoseStamped feedback.

The controller records the initial mocap pose, treats the initial yaw as the
line direction, and publishes low-speed /cmd_vel until the requested distance
is reached. It is intended for first mocap-feedback validation runs.
"""

from __future__ import print_function

import argparse
import math
import os
import signal
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist


pose = None
last_pose_wall_time = 0.0
stop_requested = False


def request_stop(signum=None, frame=None):
    global stop_requested
    stop_requested = True
    if signum is not None:
        print("\nStop requested; sending zero velocity.")


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_cb(msg):
    global pose, last_pose_wall_time
    p = msg.pose.position
    pose = (p.x, p.y, p.z, yaw_from_quat(msg.pose.orientation), msg.header.frame_id)
    last_pose_wall_time = time.time()


def angle_delta(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def clamp(value, low, high):
    return max(low, min(high, value))


def default_config_path():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    return os.path.join(
        repo_root,
        "agv_ws",
        "src",
        "agv_bringup",
        "calibration",
        "optitrack_agv1.yaml",
    )


def provided_options(argv):
    provided = set()
    for token in argv:
        if token.startswith("--"):
            provided.add(token[2:].split("=", 1)[0].replace("-", "_"))
    return provided


def load_yaml(path):
    try:
        import yaml
    except ImportError:
        print("WARN: PyYAML is not available; skipping config file %s" % path)
        return {}

    try:
        with open(path, "r") as handle:
            return yaml.safe_load(handle) or {}
    except IOError:
        print("WARN: config file not found: %s" % path)
        return {}


def apply_config(args, argv):
    if not args.config:
        return args

    config = load_yaml(args.config)
    if not config:
        return args

    supplied = provided_options(argv)
    mocap = config.get("mocap", {})
    straight = config.get("control", {}).get("straight", {})
    values = {
        "pose_topic": mocap.get("pose_topic"),
        "cmd_topic": mocap.get("cmd_topic"),
        "line_yaw_offset_deg": mocap.get("line_yaw_offset_deg"),
        "linear": straight.get("linear"),
        "min_linear": straight.get("min_linear"),
        "heading_kp": straight.get("heading_kp"),
        "max_angular": straight.get("max_angular"),
        "slowdown_distance": straight.get("slowdown_distance"),
        "position_tolerance": straight.get("position_tolerance"),
        "max_lateral_error": straight.get("max_lateral_error"),
        "pose_timeout": straight.get("pose_timeout"),
    }

    for name, value in values.items():
        if value is not None and name not in supplied:
            setattr(args, name, value)

    return args


def publish_zero(pub, seconds=0.8):
    msg = Twist()
    rate = rospy.Rate(20)
    end = time.time() + seconds
    while not rospy.is_shutdown() and time.time() < end:
        pub.publish(msg)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            time.sleep(0.05)


def wait_for_pose(timeout):
    start = time.time()
    while not rospy.is_shutdown() and pose is None:
        if time.time() - start > timeout:
            raise RuntimeError("Timed out waiting for mocap pose")
        rospy.sleep(0.05)


def line_coordinates(current_pose, origin_pose, line_yaw_offset):
    x, y, _, _, _ = current_pose
    ox, oy, _, oyaw, _ = origin_pose
    line_yaw = oyaw + line_yaw_offset
    dx = x - ox
    dy = y - oy

    forward_x = math.cos(line_yaw)
    forward_y = math.sin(line_yaw)
    left_x = -math.sin(line_yaw)
    left_y = math.cos(line_yaw)

    along = dx * forward_x + dy * forward_y
    lateral = dx * left_x + dy * left_y
    return along, lateral


def drive(pub, args):
    if not args.dry_run and not args.yes:
        raise RuntimeError("Refusing to move without --yes. Use --dry-run for a non-moving check.")

    wait_for_pose(args.wait_timeout)
    origin_pose = pose
    origin_yaw = origin_pose[3]
    line_yaw_offset = math.radians(args.line_yaw_offset_deg)
    target = args.distance
    start = time.time()
    last_report = 0.0
    best_along = -1e9
    last_progress_time = start
    max_abs_lateral = 0.0
    max_abs_heading = 0.0
    abort_reason = None
    reached = False
    rate = rospy.Rate(args.rate)

    print(
        "Mocap straight: topic=%s frame=%s start=(%.3f, %.3f, %.3f) rigid_yaw=%.1fdeg line_yaw=%.1fdeg target=%.3fm dry_run=%s"
        % (
            args.pose_topic,
            origin_pose[4],
            origin_pose[0],
            origin_pose[1],
            origin_pose[2],
            math.degrees(origin_yaw),
            math.degrees(origin_yaw + line_yaw_offset),
            target,
            args.dry_run,
        )
    )

    while not rospy.is_shutdown() and not stop_requested:
        now = time.time()
        elapsed = now - start

        if args.dry_run and elapsed >= args.dry_run_seconds:
            abort_reason = "dry-run complete"
            break

        if elapsed >= args.timeout:
            abort_reason = "timeout %.1fs" % args.timeout
            break

        pose_age = now - last_pose_wall_time
        if pose_age > args.pose_timeout:
            abort_reason = "mocap pose stale for %.3fs" % pose_age
            break

        along, lateral = line_coordinates(pose, origin_pose, line_yaw_offset)
        yaw = pose[3]
        heading_error = angle_delta(origin_yaw, yaw)
        remaining = target - along
        max_abs_lateral = max(max_abs_lateral, abs(lateral))
        max_abs_heading = max(max_abs_heading, abs(heading_error))

        if along > best_along + args.progress_epsilon:
            best_along = along
            last_progress_time = now
        elif elapsed > args.progress_grace and now - last_progress_time > args.stuck_timeout:
            abort_reason = "no mocap progress for %.1fs" % args.stuck_timeout
            break

        if elapsed > args.progress_grace and along < -args.reverse_abort_distance:
            abort_reason = "robot moved opposite target direction: along=%.3fm" % along
            break

        if abs(lateral) > args.max_lateral_error:
            abort_reason = "lateral error %.3fm exceeds limit %.3fm" % (
                lateral,
                args.max_lateral_error,
            )
            break

        if remaining <= args.position_tolerance:
            reached = True
            print(
                "Target reached: along=%.3fm remaining=%.3fm lateral=%.3fm heading=%.1fdeg"
                % (along, remaining, lateral, math.degrees(heading_error))
            )
            break

        linear = min(args.linear, max(args.min_linear, args.linear * remaining / args.slowdown_distance))
        linear = clamp(linear, 0.0, args.linear)

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = clamp(args.heading_kp * heading_error, -args.max_angular, args.max_angular)

        if not args.disable_lateral:
            msg.linear.y = clamp(-args.lateral_kp * lateral, -args.max_lateral, args.max_lateral)

        if not args.dry_run:
            pub.publish(msg)

        if args.verbose and (now - last_report >= args.report_period):
            print(
                "mocap: t=%.1fs along=%.3f/%.3f lateral=%.3f heading=%.1fdeg pose_age=%.3fs cmd=(%.3f, %.3f, %.3f)"
                % (
                    elapsed,
                    along,
                    target,
                    lateral,
                    math.degrees(heading_error),
                    pose_age,
                    msg.linear.x,
                    msg.linear.y,
                    msg.angular.z,
                )
            )
            last_report = now

        rate.sleep()

    publish_zero(pub)
    along, lateral = line_coordinates(pose, origin_pose, line_yaw_offset)
    print(
        "Mocap straight complete: reached=%s along=%.3fm lateral=%.3fm max_lateral=%.3fm max_heading=%.1fdeg%s"
        % (
            reached,
            along,
            lateral,
            max_abs_lateral,
            math.degrees(max_abs_heading),
            "" if abort_reason is None else " abort='%s'" % abort_reason,
        )
    )

    if not reached and not args.dry_run:
        return 2
    return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Drive a straight segment using mocap feedback")
    parser.add_argument("--config", default=default_config_path(),
                        help="Robot mocap calibration/control config YAML")
    parser.add_argument("--pose-topic", default="/gt/agv1/pose",
                        help="Mocap PoseStamped topic")
    parser.add_argument("--cmd-topic", default="/cmd_vel",
                        help="Twist command topic")
    parser.add_argument("--distance", type=float, default=1.0,
                        help="Target forward distance in mocap metres")
    parser.add_argument("--linear", type=float, default=0.08,
                        help="Maximum forward command")
    parser.add_argument("--min-linear", type=float, default=0.025,
                        help="Minimum forward command while not at target")
    parser.add_argument("--slowdown-distance", type=float, default=0.35,
                        help="Start linearly reducing speed inside this remaining distance")
    parser.add_argument("--heading-kp", type=float, default=0.55,
                        help="P gain from mocap heading error to angular.z")
    parser.add_argument("--max-angular", type=float, default=0.25,
                        help="Maximum angular command magnitude")
    parser.add_argument("--lateral-kp", type=float, default=0.0,
                        help="P gain from mocap lateral error to linear.y")
    parser.add_argument("--max-lateral", type=float, default=0.05,
                        help="Maximum lateral command magnitude")
    parser.add_argument("--line-yaw-offset-deg", type=float, default=0.0,
                        help="Offset from mocap rigid-body yaw to robot forward line, in degrees")
    parser.add_argument("--max-lateral-error", type=float, default=0.25,
                        help="Abort if lateral position error exceeds this many metres")
    parser.add_argument("--disable-lateral", action="store_true", default=True,
                        help="Disable linear.y correction for the first calibration runs")
    parser.add_argument("--position-tolerance", type=float, default=0.04,
                        help="Stop when remaining distance is within this tolerance")
    parser.add_argument("--timeout", type=float, default=25.0,
                        help="Maximum run duration")
    parser.add_argument("--stuck-timeout", type=float, default=5.0,
                        help="Abort if mocap along-progress does not improve for this long")
    parser.add_argument("--progress-grace", type=float, default=2.0,
                        help="Seconds before reverse/stuck checks become active")
    parser.add_argument("--progress-epsilon", type=float, default=0.01,
                        help="Minimum along-position improvement counted as progress")
    parser.add_argument("--reverse-abort-distance", type=float, default=0.08,
                        help="Abort if along-position goes this far opposite target direction")
    parser.add_argument("--pose-timeout", type=float, default=0.25,
                        help="Abort if no mocap pose arrives for this long")
    parser.add_argument("--wait-timeout", type=float, default=8.0,
                        help="Initial wait for mocap pose")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="Control loop rate")
    parser.add_argument("--report-period", type=float, default=0.5)
    parser.add_argument("--dry-run", action="store_true",
                        help="Compute and print commands without publishing motion commands")
    parser.add_argument("--dry-run-seconds", type=float, default=3.0,
                        help="Dry-run duration")
    parser.add_argument("--yes", action="store_true",
                        help="Required for actual motion")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)
    args = apply_config(args, argv)

    args.distance = max(0.01, args.distance)
    args.linear = max(0.0, min(args.linear, 0.25))
    args.min_linear = max(0.0, min(args.min_linear, args.linear))
    args.slowdown_distance = max(args.position_tolerance, args.slowdown_distance)
    args.max_angular = max(0.0, args.max_angular)
    args.max_lateral = max(0.0, args.max_lateral)
    args.timeout = max(1.0, args.timeout)
    args.pose_timeout = max(0.05, args.pose_timeout)
    args.rate = max(2.0, args.rate)
    return args


def main(argv):
    args = parse_args(argv)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rospy.init_node("drive_mocap_straight", anonymous=True)
    rospy.Subscriber(args.pose_topic, PoseStamped, pose_cb, queue_size=5)
    pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=5)
    try:
        return drive(pub, args)
    except Exception as exc:
        publish_zero(pub)
        print("ERROR: %s" % exc)
        return 1


if __name__ == "__main__":
    sys.exit(main(rospy.myargv()[1:]))
