#!/usr/bin/env python3
"""Publish an OptiTrack NatNet rigid body into ROS 1 as PoseStamped."""

import argparse
import socket
import sys
import time

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
except ImportError:
    print("Missing ROS 1 Python packages. Run this inside a sourced ROS 1 shell.", file=sys.stderr)
    raise

try:
    from natnet import NatNetClient
except ImportError:
    print(
        "Missing Python package 'natnet'. Install it with: python3 -m pip install natnet",
        file=sys.stderr,
    )
    raise


def guess_local_ip(server):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((server, 1510))
        return sock.getsockname()[0]
    finally:
        sock.close()


def parse_args():
    parser = argparse.ArgumentParser(description="Publish a NatNet rigid body as ROS 1 PoseStamped.")
    parser.add_argument("--server", default="192.168.50.200",
                        help="Motive/NatNet server IP")
    parser.add_argument("--local", default=None,
                        help="Local interface IP. Defaults to auto-detect.")
    parser.add_argument("--name", default="orkar_agv1",
                        help="Rigid body name to publish")
    parser.add_argument("--topic", default="/optitrack/rigid_bodies/orkar_agv1",
                        help="ROS 1 PoseStamped output topic")
    parser.add_argument("--frame-id", default="world",
                        help="PoseStamped header frame_id")
    parser.add_argument("--multicast", action="store_true",
                        help="Use multicast data reception instead of unicast")
    parser.add_argument("--status-period", type=float, default=2.0,
                        help="Console status print period in seconds")
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    local_ip = args.local or guess_local_ip(args.server)

    rospy.init_node("natnet_ros1_pose_publisher", anonymous=False)
    pub = rospy.Publisher(args.topic, PoseStamped, queue_size=20)

    state = {
        "names": [],
        "printed_defs": False,
        "published": 0,
        "last_status": 0.0,
    }

    def on_descriptions(desc):
        state["names"] = [rb.name for rb in desc.rigid_bodies]
        if state["printed_defs"]:
            return

        rospy.loginfo("NatNet server: %s", args.server)
        rospy.loginfo("Local interface: %s", local_ip)
        for rb in desc.rigid_bodies:
            marker_count = len(rb.markers) if rb.markers is not None else 0
            rospy.loginfo("Rigid body: name=%s id=%s markers=%d", rb.name, rb.id_num, marker_count)
        if args.name not in state["names"]:
            rospy.logwarn("Requested rigid body '%s' is not in model definitions.", args.name)
        state["printed_defs"] = True

    def on_frame(frame):
        if args.name not in state["names"]:
            return
        index = state["names"].index(args.name)
        if index >= len(frame.rigid_bodies):
            return

        rb = frame.rigid_bodies[index]
        if not rb.tracking_valid:
            return

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = args.frame_id
        msg.pose.position.x = rb.pos[0]
        msg.pose.position.y = rb.pos[1]
        msg.pose.position.z = rb.pos[2]
        msg.pose.orientation.x = rb.rot[0]
        msg.pose.orientation.y = rb.rot[1]
        msg.pose.orientation.z = rb.rot[2]
        msg.pose.orientation.w = rb.rot[3]
        pub.publish(msg)

        state["published"] += 1
        now = time.time()
        if now - state["last_status"] >= args.status_period:
            rospy.loginfo(
                "Publishing %s to %s: pos=(%.3f, %.3f, %.3f), count=%d",
                args.name,
                args.topic,
                rb.pos[0],
                rb.pos[1],
                rb.pos[2],
                state["published"],
            )
            state["last_status"] = now

    client = NatNetClient(
        server_ip_address=args.server,
        local_ip_address=local_ip,
        use_multicast=args.multicast,
    )
    client.on_data_description_received_event.handlers.append(on_descriptions)
    client.on_data_frame_received_event.handlers.append(on_frame)

    try:
        client.connect(timeout=3.0)
        client.request_modeldef()
        rospy.loginfo("Publishing NatNet rigid body '%s' on %s", args.name, args.topic)
        while not rospy.is_shutdown():
            client.update_sync()
            time.sleep(0.005)
    finally:
        client.shutdown()


if __name__ == "__main__":
    main()
