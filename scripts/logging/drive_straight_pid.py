#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Straight-line odometry P controller.

Drives straight using:
  omega = heading_kp * heading_error - lateral_kp * lateral_error + bias

This keeps the robot on the original odom line. It supports repeated
out-and-back cycles for scenario 2. The return leg reverses along the same line
while keeping the same heading target; no endpoint forced rotation is used.
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


def angle_delta(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def clamp(value, low, high):
    return max(low, min(high, value))


def wait_for_odom(timeout):
    start = time.time()
    while not rospy.is_shutdown() and pose is None:
        if time.time() - start > timeout:
            raise RuntimeError("Timed out waiting for /odom")
        rospy.sleep(0.05)


def publish_zero(pub, seconds=0.8):
    msg = Twist()
    rate = rospy.Rate(20)
    end = time.time() + seconds
    while not rospy.is_shutdown() and time.time() < end:
        pub.publish(msg)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


def line_errors(current_pose, origin_pose):
    x, y, yaw = current_pose
    ox, oy, oyaw = origin_pose
    dx = x - ox
    dy = y - oy

    along = dx * math.cos(oyaw) + dy * math.sin(oyaw)
    lateral = -dx * math.sin(oyaw) + dy * math.cos(oyaw)
    heading = angle_delta(oyaw, yaw)
    return along, lateral, heading


def wait_before_motion(pub, args):
    if args.start_at_epoch > 0.0:
        target_epoch = args.start_at_epoch + args.start_delay
        print(
            "Waiting for scheduled start epoch %.3f (base %.3f + delay %.1fs)."
            % (target_epoch, args.start_at_epoch, args.start_delay)
        )
    elif args.start_delay > 0.0:
        target_epoch = time.time() + args.start_delay
        print("Waiting %.1fs before moving..." % args.start_delay)
    else:
        return

    msg = Twist()
    rate = rospy.Rate(10)
    last_report = 0.0
    while not rospy.is_shutdown() and not stop_requested:
        remaining = target_epoch - time.time()
        if remaining <= 0.0:
            break
        pub.publish(msg)
        now = time.time()
        if args.verbose and (now - last_report >= 10.0 or remaining <= 10.0):
            print("start wait: %.1fs remaining" % remaining)
            last_report = now
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


def drive_leg(pub, args, origin_pose, target_along, direction, leg_name, deadline):
    msg = Twist()
    rate = rospy.Rate(args.rate)
    start = time.time()
    last_report = start
    max_abs_lateral = 0.0
    max_abs_heading = 0.0

    while not rospy.is_shutdown() and not stop_requested:
        now = time.time()
        if deadline is not None and now >= deadline:
            print("Duration reached during %s; stopping." % leg_name)
            break

        along, lateral, heading_error = line_errors(pose, origin_pose)
        max_abs_lateral = max(max_abs_lateral, abs(lateral))
        max_abs_heading = max(max_abs_heading, abs(heading_error))

        remaining = direction * (target_along - along)
        if remaining <= args.position_tolerance:
            print(
                "%s target reached: along=%.3fm target=%.3fm remaining=%.3fm lateral=%.3fm"
                % (leg_name, along, target_along, remaining, lateral)
            )
            break

        if args.timeout > 0.0 and now - start >= args.timeout:
            print(
                "WARN %s timeout after %.1fs; along=%.3fm target=%.3fm remaining=%.3fm lateral=%.3fm"
                % (leg_name, args.timeout, along, target_along, remaining, lateral)
            )
            break

        bias = args.outbound_bias if direction > 0.0 else args.return_bias
        omega = (
            args.heading_kp * heading_error
            - direction * args.lateral_sign * args.lateral_kp * lateral
            + bias
        )

        msg.linear.x = direction * args.linear
        msg.linear.y = 0.0
        msg.angular.z = clamp(omega, -args.max_angular, args.max_angular)
        pub.publish(msg)

        if args.verbose and now - last_report >= args.report_period:
            print(
                "%s: elapsed=%.1fs along=%.3fm target=%.3fm lateral=%.3fm heading=%.1fdeg cmd=(%.3f, %.3f)"
                % (
                    leg_name,
                    now - start,
                    along,
                    target_along,
                    lateral,
                    math.degrees(heading_error),
                    msg.linear.x,
                    msg.angular.z,
                )
            )
            last_report = now

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break

    publish_zero(pub, seconds=args.pause)
    return {
        "elapsed": time.time() - start,
        "max_lateral": max_abs_lateral,
        "max_heading": max_abs_heading,
    }


def drive_out_and_back(pub, args):
    origin_pose = pose
    start_time = time.time()
    deadline = start_time + args.duration if args.duration > 0.0 else None
    cycle = 0
    last_out = None
    last_back = None

    print(
        "Start odom: x=%.3f y=%.3f yaw=%.1fdeg"
        % (origin_pose[0], origin_pose[1], math.degrees(origin_pose[2]))
    )

    while not rospy.is_shutdown() and not stop_requested:
        if deadline is not None and time.time() >= deadline:
            print("Duration reached; stopping straight path.")
            break

        cycle += 1
        print("Starting straight cycle %d." % cycle)

        last_out = drive_leg(
            pub, args, origin_pose, args.distance, 1.0,
            "cycle%d outbound" % cycle, deadline
        )
        if rospy.is_shutdown() or stop_requested:
            break
        if deadline is not None and time.time() >= deadline:
            break

        last_back = drive_leg(
            pub, args, origin_pose, 0.0, -1.0,
            "cycle%d return" % cycle, deadline
        )

        if args.duration <= 0.0:
            break
        if args.cycles > 0 and cycle >= args.cycles:
            break

    publish_zero(pub, seconds=1.0)
    final_along, final_lateral, final_heading = line_errors(pose, origin_pose)
    final_error = math.hypot(final_along, final_lateral)
    print(
        "Straight PID complete: elapsed=%.1fs cycles=%d final_error=%.3fm final_along=%.3fm final_lateral=%.3fm final_heading=%.1fdeg"
        % (
            time.time() - start_time,
            cycle,
            final_error,
            final_along,
            final_lateral,
            math.degrees(final_heading),
        )
    )
    if last_out is not None:
        print(
            "  last outbound: elapsed=%.1fs max_lateral=%.3fm max_heading=%.1fdeg"
            % (
                last_out["elapsed"],
                last_out["max_lateral"],
                math.degrees(last_out["max_heading"]),
            )
        )
    if last_back is not None:
        print(
            "  last return: elapsed=%.1fs max_lateral=%.3fm max_heading=%.1fdeg"
            % (
                last_back["elapsed"],
                last_back["max_lateral"],
                math.degrees(last_back["max_heading"]),
            )
        )


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Straight-line odom P controller")
    parser.add_argument("--distance", type=float, default=3.0)
    parser.add_argument("--linear", type=float, default=0.10)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Run duration in seconds; 0 means one out-and-back cycle")
    parser.add_argument("--cycles", type=int, default=0,
                        help="Optional cycle count; 0 means use duration/one cycle")
    parser.add_argument("--heading-kp", type=float, default=0.8)
    parser.add_argument("--lateral-kp", type=float, default=0.4)
    parser.add_argument("--lateral-sign", type=float, default=1.0,
                        help="Flip to -1 if lateral correction moves the wrong way")
    parser.add_argument("--max-angular", type=float, default=0.25)
    parser.add_argument("--outbound-bias", type=float, default=0.0)
    parser.add_argument("--return-bias", type=float, default=0.0)
    parser.add_argument("--position-tolerance", type=float, default=0.05)
    parser.add_argument("--pause", type=float, default=0.8)
    parser.add_argument("--timeout", type=float, default=50.0)
    parser.add_argument("--start-delay", type=float, default=0.0)
    parser.add_argument("--start-at-epoch", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--report-period", type=float, default=2.0)
    parser.add_argument("--no-prompt", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    args.distance = max(0.05, args.distance)
    args.linear = clamp(abs(args.linear), 0.0, 1.0)
    args.duration = max(0.0, args.duration)
    args.cycles = max(0, args.cycles)
    args.heading_kp = max(0.0, args.heading_kp)
    args.lateral_kp = max(0.0, args.lateral_kp)
    args.lateral_sign = 1.0 if args.lateral_sign >= 0.0 else -1.0
    args.max_angular = clamp(abs(args.max_angular), 0.0, 1.0)
    args.outbound_bias = clamp(args.outbound_bias, -args.max_angular, args.max_angular)
    args.return_bias = clamp(args.return_bias, -args.max_angular, args.max_angular)
    args.position_tolerance = clamp(abs(args.position_tolerance), 0.01, 0.25)
    args.pause = max(0.0, args.pause)
    args.timeout = max(0.0, args.timeout)
    args.start_delay = max(0.0, args.start_delay)
    args.start_at_epoch = max(0.0, args.start_at_epoch)
    args.rate = max(5.0, args.rate)
    args.report_period = max(0.5, args.report_period)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rospy.init_node("drive_straight_pid")
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=20)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    wait_for_odom(timeout=10.0)
    publish_zero(pub, seconds=0.5)

    print("Straight PID ready:")
    print(
        "  distance=%.2fm linear=%.2f duration=%.1fs heading_kp=%.2f lateral_kp=%.2f max_angular=%.2f"
        % (
            args.distance,
            args.linear,
            args.duration,
            args.heading_kp,
            args.lateral_kp,
            args.max_angular,
        )
    )
    print(
        "  lateral_sign=%.0f outbound_bias=%.3f return_bias=%.3f tolerance=%.2fm"
        % (args.lateral_sign, args.outbound_bias, args.return_bias, args.position_tolerance)
    )

    if not args.no_prompt:
        input("Press Enter to start, or Ctrl+C to cancel...")

    wait_before_motion(pub, args)

    try:
        if not stop_requested:
            drive_out_and_back(pub, args)
    finally:
        publish_zero(pub, seconds=1.0)
        print("Straight PID finished; zero velocity sent.")


if __name__ == "__main__":
    main(sys.argv[1:])
