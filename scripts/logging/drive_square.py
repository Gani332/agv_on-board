#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Drive a conservative square using /odom feedback and /cmd_vel commands.

Run this only with the robot on the floor, clear space around it, and an
operator ready to stop it. Ctrl+C publishes zero velocity before exiting.
"""

from __future__ import print_function

import argparse
import math
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


pose = None


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def odom_cb(msg):
    global pose
    p = msg.pose.pose.position
    pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))


def angle_delta(current, start):
    return math.atan2(math.sin(current - start), math.cos(current - start))


def clamp(value, low, high):
    return max(low, min(high, value))


def publish_zero(pub, seconds=0.7):
    msg = Twist()
    end = rospy.Time.now() + rospy.Duration(seconds)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and rospy.Time.now() < end:
        pub.publish(msg)
        rate.sleep()


def wait_for_odom(timeout):
    start = time.time()
    while not rospy.is_shutdown() and pose is None:
        if time.time() - start > timeout:
            raise RuntimeError("Timed out waiting for /odom")
        rospy.sleep(0.05)


def drive_side(pub, side_m, linear_cmd, timeout_s):
    start_x, start_y, _ = pose
    msg = Twist()
    msg.linear.x = linear_cmd
    rate = rospy.Rate(20)
    start = time.time()

    while not rospy.is_shutdown():
        x, y, _ = pose
        travelled = math.hypot(x - start_x, y - start_y)
        if travelled >= side_m:
            break
        if time.time() - start > timeout_s:
            print("WARN side timeout after %.1fs; travelled %.3fm" % (timeout_s, travelled))
            break
        pub.publish(msg)
        rate.sleep()

    publish_zero(pub)


def turn(pub, turn_rad, angular_cmd, timeout_s):
    _, _, start_yaw = pose
    direction = 1.0 if turn_rad >= 0.0 else -1.0
    target = abs(turn_rad)
    msg = Twist()
    msg.angular.z = direction * abs(angular_cmd)
    rate = rospy.Rate(20)
    start = time.time()

    while not rospy.is_shutdown():
        _, _, yaw = pose
        turned = abs(angle_delta(yaw, start_yaw))
        if turned >= target:
            break
        if time.time() - start > timeout_s:
            print("WARN turn timeout after %.1fs; turned %.1f deg" %
                  (timeout_s, math.degrees(turned)))
            break
        pub.publish(msg)
        rate.sleep()

    publish_zero(pub)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Drive a square on /cmd_vel")
    parser.add_argument("--side", type=float, default=0.75,
                        help="Target side length in odom metres")
    parser.add_argument("--linear", type=float, default=0.22,
                        help="Normalized forward command, clamped to [-1, 1]")
    parser.add_argument("--angular", type=float, default=0.28,
                        help="Normalized turn command, clamped to [-1, 1]")
    parser.add_argument("--turn-deg", type=float, default=90.0,
                        help="Turn angle per corner")
    parser.add_argument("--cycles", type=int, default=1,
                        help="Number of squares")
    parser.add_argument("--side-timeout", type=float, default=18.0,
                        help="Max seconds per side")
    parser.add_argument("--turn-timeout", type=float, default=12.0,
                        help="Max seconds per turn")
    parser.add_argument("--pause", type=float, default=0.8,
                        help="Pause between moves")
    parser.add_argument("--clockwise", action="store_true",
                        help="Turn clockwise instead of counter-clockwise")
    parser.add_argument("--no-prompt", action="store_true",
                        help="Start immediately without pressing Enter")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    args.linear = clamp(args.linear, -1.0, 1.0)
    args.angular = clamp(abs(args.angular), 0.0, 1.0)
    turn_rad = math.radians(args.turn_deg)
    if args.clockwise:
        turn_rad = -turn_rad

    rospy.init_node("agv_drive_square")
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=20)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    wait_for_odom(timeout=10.0)
    publish_zero(pub)

    print("Square drive ready:")
    print("  side=%.2fm linear=%.2f angular=%.2f turn=%.1fdeg cycles=%d" %
          (args.side, args.linear, args.angular, args.turn_deg, args.cycles))
    print("  Ctrl+C stops the robot.")
    if not args.no_prompt:
        try:
            raw_input("Press Enter to start, or Ctrl+C to cancel...")
        except NameError:
            input("Press Enter to start, or Ctrl+C to cancel...")

    try:
        for cycle in range(args.cycles):
            print("Starting square %d/%d" % (cycle + 1, args.cycles))
            for side_idx in range(4):
                print("  side %d: forward" % (side_idx + 1))
                drive_side(pub, args.side, args.linear, args.side_timeout)
                rospy.sleep(args.pause)
                print("  corner %d: turn" % (side_idx + 1))
                turn(pub, turn_rad, args.angular, args.turn_timeout)
                rospy.sleep(args.pause)
    finally:
        publish_zero(pub, seconds=1.0)
        print("Square drive finished; zero velocity sent.")


if __name__ == "__main__":
    main(sys.argv[1:])
