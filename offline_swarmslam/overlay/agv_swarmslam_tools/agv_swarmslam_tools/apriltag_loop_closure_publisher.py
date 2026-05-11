import csv
import json
import math
import time
from pathlib import Path

import rclpy
from cslam_common_interfaces.msg import InterRobotLoopClosure, KeyframeOdom
from geometry_msgs.msg import Transform
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


DEFAULT_ALIGNMENT_FILE = "/results/apriltags_pass1/robotA_to_robotB_alignment.json"
DEFAULT_DETECTIONS_FILE = "/results/apriltags_pass1/detections.csv"


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


def quaternion_from_yaw(transform, yaw):
    transform.rotation.x = 0.0
    transform.rotation.y = 0.0
    transform.rotation.z = math.sin(yaw * 0.5)
    transform.rotation.w = math.cos(yaw * 0.5)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_pose(a, b):
    ax, ay, ayaw = a
    bx, by, byaw = b
    cos_yaw = math.cos(ayaw)
    sin_yaw = math.sin(ayaw)
    return (
        ax + cos_yaw * bx - sin_yaw * by,
        ay + sin_yaw * bx + cos_yaw * by,
        normalize_angle(ayaw + byaw),
    )


def invert_pose(pose):
    x, y, yaw = pose
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        -cos_yaw * x - sin_yaw * y,
        sin_yaw * x - cos_yaw * y,
        normalize_angle(-yaw),
    )


def odom_msg_to_pose(msg):
    pose = msg.odom.pose.pose
    return (
        float(pose.position.x),
        float(pose.position.y),
        yaw_from_quaternion(pose.orientation),
    )


class AprilTagLoopClosurePublisher(Node):
    def __init__(self):
        super().__init__("agv_apriltag_loop_closure_publisher")
        self.declare_parameter("alignment_file", DEFAULT_ALIGNMENT_FILE)
        self.declare_parameter("detections_file", DEFAULT_DETECTIONS_FILE)
        self.declare_parameter("tag_id", 1)
        self.declare_parameter("max_constraints", 12)
        self.declare_parameter("min_keyframe_separation", 8)
        self.declare_parameter("timestamp_tolerance_sec", 2.0)
        self.declare_parameter("publish_after_clock_quiet_sec", 4.0)
        self.declare_parameter("publish_period_sec", 0.5)

        self.alignment_file = Path(
            self.get_parameter("alignment_file").get_parameter_value().string_value
        )
        self.detections_file = Path(
            self.get_parameter("detections_file").get_parameter_value().string_value
        )
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.max_constraints = int(self.get_parameter("max_constraints").value)
        self.min_keyframe_separation = int(
            self.get_parameter("min_keyframe_separation").value
        )
        self.timestamp_tolerance_sec = float(
            self.get_parameter("timestamp_tolerance_sec").value
        )
        self.publish_after_clock_quiet_sec = float(
            self.get_parameter("publish_after_clock_quiet_sec").value
        )

        self.source_robot_id = None
        self.target_robot_id = None
        self.source_label = None
        self.target_label = None
        self.source_time_offset_sec = 0.0
        self.source_odom_to_target_odom = (0.0, 0.0, 0.0)
        self.target_odom_to_source_odom = (0.0, 0.0, 0.0)
        self.frame_range = None
        self.detection_stamps = []
        self.keyframes = {0: [], 1: []}
        self.published_pairs = set()
        self.last_source_keyframe_id = None
        self.last_target_keyframe_id = None
        self.next_detection_index = 0
        self.last_keyframe_monotonic = None
        self.last_clock_monotonic = None
        self.waiting_logged = False
        self.no_constraints_logged = False

        if not self.load_alignment() or not self.load_detections():
            self.disabled = True
            self.get_logger().error("AprilTag graph constraints are disabled.")
            return

        self.disabled = False
        self.publisher = self.create_publisher(
            InterRobotLoopClosure, "/cslam/inter_robot_loop_closure", 100
        )
        self.create_subscription(
            KeyframeOdom,
            "/r%d/cslam/keyframe_odom" % self.source_robot_id,
            lambda msg: self.keyframe_callback(self.source_robot_id, msg),
            1000,
        )
        self.create_subscription(
            KeyframeOdom,
            "/r%d/cslam/keyframe_odom" % self.target_robot_id,
            lambda msg: self.keyframe_callback(self.target_robot_id, msg),
            1000,
        )
        clock_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, "/clock", self.clock_callback, clock_qos)
        publish_period_sec = float(self.get_parameter("publish_period_sec").value)
        self.create_timer(publish_period_sec, self.publish_available_constraints)
        self.get_logger().info(
            "AprilTag graph constraints enabled: robot %d -> robot %d, detections=%d, max_constraints=%d"
            % (
                self.source_robot_id,
                self.target_robot_id,
                len(self.detection_stamps),
                self.max_constraints,
            )
        )

    def load_alignment(self):
        if not self.alignment_file.exists():
            self.get_logger().error("Missing alignment file: %s" % self.alignment_file)
            return False
        with self.alignment_file.open() as alignment_file:
            data = json.load(alignment_file)

        transform = data["transform_source_odom_to_observer_odom"]
        self.source_robot_id = int(data.get("source_robot_index", 0))
        self.target_robot_id = int(data.get("target_robot_index", 1))
        self.source_label = data.get("source_robot", "robotA")
        self.target_label = data.get("observer_robot", "robotB")
        self.source_time_offset_sec = float(data.get("source_time_offset_sec", 0.0))
        self.source_odom_to_target_odom = (
            float(transform["x_m"]),
            float(transform["y_m"]),
            float(transform["yaw_rad"]),
        )
        self.target_odom_to_source_odom = invert_pose(self.source_odom_to_target_odom)
        frame_range = data.get("frame_range_inliers")
        if frame_range and len(frame_range) == 2:
            self.frame_range = (int(frame_range[0]), int(frame_range[1]))
        return True

    def load_detections(self):
        if not self.detections_file.exists():
            self.get_logger().error("Missing detections file: %s" % self.detections_file)
            return False

        rows = []
        with self.detections_file.open(newline="") as detections_file:
            for row in csv.DictReader(detections_file):
                if row.get("robot") != self.target_label:
                    continue
                if int(row.get("tag_id", -1)) != self.tag_id:
                    continue
                inferred_owner = row.get("inferred_tag_owner", "")
                if inferred_owner and inferred_owner != self.source_label:
                    continue
                frame = int(row.get("frame", -1))
                if self.frame_range is not None:
                    if frame < self.frame_range[0] or frame > self.frame_range[1]:
                        continue
                rows.append(float(row["stamp_sec"]))

        if not rows:
            self.get_logger().error(
                "No usable AprilTag detections for %s seeing tag %d from %s"
                % (self.target_label, self.tag_id, self.source_label)
            )
            return False

        rows = sorted(rows)
        self.detection_stamps = rows
        return True

    def keyframe_callback(self, robot_id, msg):
        keyframe = {
            "id": int(msg.id),
            "stamp": stamp_sec(msg.odom.header.stamp),
            "pose": odom_msg_to_pose(msg),
        }
        frames = self.keyframes[robot_id]
        if frames and frames[-1]["id"] == keyframe["id"]:
            frames[-1] = keyframe
        else:
            frames.append(keyframe)
            frames.sort(key=lambda item: item["stamp"])
        self.last_keyframe_monotonic = time.monotonic()

    def clock_callback(self, _msg):
        self.last_clock_monotonic = time.monotonic()

    def nearest_keyframe(self, robot_id, stamp):
        frames = self.keyframes[robot_id]
        if not frames:
            return None
        nearest = min(frames, key=lambda item: abs(item["stamp"] - stamp))
        if abs(nearest["stamp"] - stamp) > self.timestamp_tolerance_sec:
            return None
        return nearest

    def enough_separation(self, source_keyframe_id, target_keyframe_id):
        if self.last_source_keyframe_id is None or self.last_target_keyframe_id is None:
            return True
        return (
            abs(source_keyframe_id - self.last_source_keyframe_id)
            >= self.min_keyframe_separation
            and abs(target_keyframe_id - self.last_target_keyframe_id)
            >= self.min_keyframe_separation
        )

    def relative_source_to_target(self, source_pose, target_pose):
        target_pose_in_source_odom = compose_pose(
            self.target_odom_to_source_odom,
            target_pose,
        )
        return compose_pose(invert_pose(source_pose), target_pose_in_source_odom)

    def make_loop_closure(self, source_keyframe, target_keyframe):
        x, y, yaw = self.relative_source_to_target(
            source_keyframe["pose"],
            target_keyframe["pose"],
        )

        msg = InterRobotLoopClosure()
        msg.robot0_id = int(self.source_robot_id)
        msg.robot0_keyframe_id = int(source_keyframe["id"])
        msg.robot1_id = int(self.target_robot_id)
        msg.robot1_keyframe_id = int(target_keyframe["id"])
        msg.success = True
        msg.transform = Transform()
        msg.transform.translation.x = float(x)
        msg.transform.translation.y = float(y)
        msg.transform.translation.z = 0.0
        quaternion_from_yaw(msg.transform, yaw)
        return msg

    def publish_available_constraints(self):
        if self.disabled or len(self.published_pairs) >= self.max_constraints:
            return

        if self.last_keyframe_monotonic is None or self.last_clock_monotonic is None:
            return
        clock_quiet_sec = time.monotonic() - self.last_clock_monotonic
        if clock_quiet_sec < self.publish_after_clock_quiet_sec:
            if not self.waiting_logged and self.keyframes[self.source_robot_id] and self.keyframes[self.target_robot_id]:
                self.waiting_logged = True
                self.get_logger().info(
                    "Waiting for rosbag playback to finish before publishing AprilTag graph constraints."
                )
            return

        while (
            self.next_detection_index < len(self.detection_stamps)
            and len(self.published_pairs) < self.max_constraints
        ):
            target_stamp = self.detection_stamps[self.next_detection_index]
            self.next_detection_index += 1

            if len(self.published_pairs) >= self.max_constraints:
                break

            source_stamp = target_stamp + self.source_time_offset_sec
            source_keyframe = self.nearest_keyframe(self.source_robot_id, source_stamp)
            target_keyframe = self.nearest_keyframe(self.target_robot_id, target_stamp)
            if source_keyframe is None or target_keyframe is None:
                continue

            pair = (source_keyframe["id"], target_keyframe["id"])
            if pair in self.published_pairs:
                continue
            if not self.enough_separation(source_keyframe["id"], target_keyframe["id"]):
                continue

            msg = self.make_loop_closure(source_keyframe, target_keyframe)
            self.publisher.publish(msg)
            self.published_pairs.add(pair)
            self.last_source_keyframe_id = source_keyframe["id"]
            self.last_target_keyframe_id = target_keyframe["id"]
            self.get_logger().info(
                "Published AprilTag inter-robot factor: (%d,%d) -> (%d,%d)"
                % (
                    msg.robot0_id,
                    msg.robot0_keyframe_id,
                    msg.robot1_id,
                    msg.robot1_keyframe_id,
                )
            )

        if (
            self.next_detection_index >= len(self.detection_stamps)
            and not self.published_pairs
            and not self.no_constraints_logged
        ):
            self.no_constraints_logged = True
            self.get_logger().warning(
                "No AprilTag graph constraints matched available keyframes within %.2fs."
                % self.timestamp_tolerance_sec
            )


def main():
    rclpy.init()
    node = AprilTagLoopClosurePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
