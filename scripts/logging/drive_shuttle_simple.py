#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Endpoint-position P-controller shuttle.

This script only cares about start/end positions:
  - capture current /odom pose as the cycle start
  - compute a forward target point X metres ahead of that start pose
  - drive toward that target point using position P control
  - drive back toward the original start point using position P control

It does not track centre-line deviation and does not use a line-following
controller. Odom is used only as position feedback for the endpoint targets.
"""

import argparse
import math
import signal
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


pose = None
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


def odom_cb(msg):
    global pose
    p = msg.pose.pose.position
    pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))


def clamp(value, low, high):
    return max(low, min(high, value))


def point_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def wait_for_odom(timeout):
    start = time.time()
    while not rospy.is_shutdown() and pose is None:
        if time.time() - start > timeout:
            raise RuntimeError("Timed out waiting for /odom")
        rospy.sleep(0.1)


def publish_zero(pub, seconds=1.0):
    msg = Twist()
    rate = rospy.Rate(20)
    end = time.time() + seconds
    while time.time() < end and not rospy.is_shutdown():
        pub.publish(msg)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


def forward_target(start_pose, distance):
    x, y, yaw = start_pose
    return (
        x + distance * math.cos(yaw),
        y + distance * math.sin(yaw),
    )


def world_error_to_body(error_x, error_y, yaw):
    """Convert world-frame position error into robot body-frame command axes."""
    body_x = math.cos(yaw) * error_x + math.sin(yaw) * error_y
    body_y = -math.sin(yaw) * error_x + math.cos(yaw) * error_y
    return body_x, body_y


def apply_min_command(value, min_abs):
    if value == 0.0 or abs(value) >= min_abs:
        return value
    return math.copysign(min_abs, value)


def drive_to_point(pub, args, target, segment_index, label):
    rate = rospy.Rate(args.rate)
    start_time = time.time()
    last_report = start_time
    closest = point_distance(pose, target)

    print(
        "Segment %d starting (%s): target=(%.3f, %.3f), initial_error=%.3fm"
        % (segment_index, label, target[0], target[1], closest)
    )

    while not rospy.is_shutdown() and not stop_requested:
        now = time.time()
        error_x = target[0] - pose[0]
        error_y = target[1] - pose[1]
        error = math.hypot(error_x, error_y)
        closest = min(closest, error)

        if error <= args.position_tolerance:
            break
        if args.timeout > 0.0 and now - start_time >= args.timeout:
            print(
                "WARN segment %d timeout after %.1fs; error=%.3fm closest=%.3fm"
                % (segment_index, args.timeout, error, closest)
            )
            break

        body_x, body_y = world_error_to_body(error_x, error_y, pose[2])

        cmd_x = clamp(args.position_kp * body_x, -args.linear, args.linear)
        cmd_y = clamp(args.position_kp * body_y, -args.max_lateral, args.max_lateral)

        cmd_x = apply_min_command(cmd_x, args.min_linear)
        cmd_y = apply_min_command(cmd_y, args.min_lateral)

        msg = Twist()
        msg.linear.x = cmd_x
        msg.linear.y = cmd_y
        msg.angular.z = args.bias
        pub.publish(msg)

        if args.verbose and now - last_report >= args.report_period:
            print(
                "segment %d %s: elapsed=%.1fs error=%.3fm body_error=(%.3f, %.3f) cmd=(%.3f, %.3f, %.3f)"
                % (
                    segment_index,
                    label,
                    now - start_time,
                    error,
                    body_x,
                    body_y,
                    msg.linear.x,
                    msg.linear.y,
                    msg.angular.z,
                )
            )
            last_report = now

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break

    publish_zero(pub, seconds=args.stop_time)
    final_error = point_distance(pose, target)
    print(
        "Segment %d done (%s): final_error=%.3fm closest=%.3fm"
        % (segment_index, label, final_error, closest)
    )
    return final_error


def drive_shuttle(pub, args):
    for cycle in range(args.cycles):
        if stop_requested:
            break

        start_pose = pose
        far_target = forward_target(start_pose, args.distance)
        start_target = (start_pose[0], start_pose[1])

        print(
            "Cycle %d start: x=%.3f y=%.3f yaw=%.1fdeg"
            % (cycle + 1, start_pose[0], start_pose[1], math.degrees(start_pose[2]))
        )

        drive_to_point(pub, args, far_target, 2 * cycle + 1, "forward-target")
        if stop_requested:
            break
        time.sleep(args.pause)

        drive_to_point(pub, args, start_target, 2 * cycle + 2, "return-start")
        if stop_requested:
            break
        time.sleep(args.pause)

        cycle_error = point_distance(pose, start_target)
        print("Cycle %d complete: start_error=%.3fm." % (cycle + 1, cycle_error))


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Endpoint P-controller shuttle")
    parser.add_argument("--distance", type=float, default=3.0,
                        help="Forward target distance from cycle start in odom metres")
    parser.add_argument("--cycles", type=int, default=1,
                        help="Number of forward/back cycles")
    parser.add_argument("--linear", type=float, default=0.15,
                        help="Maximum absolute linear.x command")
    parser.add_argument("--max-lateral", type=float, default=0.08,
                        help="Maximum absolute linear.y command")
    parser.add_argument("--position-kp", type=float, default=0.45,
                        help="P gain from target position error to x/y command")
    parser.add_argument("--position-tolerance", type=float, default=0.08,
                        help="Stop when this close to target point")
    parser.add_argument("--min-linear", type=float, default=0.025,
                        help="Minimum non-zero linear.x command")
    parser.add_argument("--min-lateral", type=float, default=0.0,
                        help="Minimum non-zero linear.y command")
    parser.add_argument("--bias", type=float, default=0.0,
                        help="Constant angular.z bias; negative = right, positive = left")
    parser.add_argument("--pause", type=float, default=1.5,
                        help="Pause between segments in seconds")
    parser.add_argument("--stop-time", type=float, default=1.0,
                        help="Seconds to publish zero velocity after each segment")
    parser.add_argument("--timeout", type=float, default=50.0,
                        help="Max seconds per segment; 0 disables timeout")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="Command publish rate in Hz")
    parser.add_argument("--report-period", type=float, default=2.0,
                        help="Verbose report period in seconds")
    parser.add_argument("--no-prompt", action="store_true",
                        help="Start immediately without pressing Enter")
    parser.add_argument("--verbose", action="store_true",
                        help="Print periodic odom target feedback")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    args.distance = max(0.01, args.distance)
    args.cycles = max(1, args.cycles)
    args.linear = clamp(abs(args.linear), 0.0, 1.0)
    args.max_lateral = clamp(abs(args.max_lateral), 0.0, 1.0)
    args.position_kp = max(0.0, args.position_kp)
    args.position_tolerance = max(0.01, args.position_tolerance)
    args.min_linear = clamp(abs(args.min_linear), 0.0, args.linear)
    args.min_lateral = clamp(abs(args.min_lateral), 0.0, args.max_lateral)
    args.bias = clamp(args.bias, -1.0, 1.0)
    args.pause = max(0.0, args.pause)
    args.stop_time = max(0.0, args.stop_time)
    args.timeout = max(0.0, args.timeout)
    args.rate = max(1.0, args.rate)
    args.report_period = max(0.5, args.report_period)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rospy.init_node("drive_shuttle_endpoint_p")
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=20)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    print("Waiting for /odom...")
    wait_for_odom(timeout=10.0)
    publish_zero(pub, seconds=0.5)

    print("Endpoint shuttle ready:")
    print(
        "  distance=%.2fm cycles=%d linear=%.3f max_lateral=%.3f kp=%.2f tolerance=%.2fm bias=%.3f"
        % (
            args.distance,
            args.cycles,
            args.linear,
            args.max_lateral,
            args.position_kp,
            args.position_tolerance,
            args.bias,
        )
    )
    print("  Ctrl+C stops the robot.")

    if not args.no_prompt:
        input("Press Enter to start, or Ctrl+C to cancel...")

    try:
        drive_shuttle(pub, args)
    finally:
        publish_zero(pub, seconds=1.0)
        print("Finished; zero velocity sent.")


if __name__ == "__main__":
    main(sys.argv[1:])
