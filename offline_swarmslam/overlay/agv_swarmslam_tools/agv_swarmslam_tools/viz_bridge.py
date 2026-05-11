from collections import defaultdict
import json
import math
from pathlib import Path as FilePath
import struct

import rclpy
from cslam_common_interfaces.msg import PoseGraph, VizPointCloud
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


ROBOT_COLORS = [
    ColorRGBA(r=0.1, g=0.45, b=1.0, a=1.0),
    ColorRGBA(r=1.0, g=0.55, b=0.1, a=1.0),
    ColorRGBA(r=0.25, g=0.8, b=0.35, a=1.0),
    ColorRGBA(r=0.8, g=0.25, b=0.9, a=1.0),
    ColorRGBA(r=0.95, g=0.2, b=0.25, a=1.0),
    ColorRGBA(r=0.0, g=0.75, b=0.8, a=1.0),
    ColorRGBA(r=0.95, g=0.85, b=0.1, a=1.0),
    ColorRGBA(r=0.55, g=0.55, b=0.55, a=1.0),
]
VIS_FRAME = "map"
MAX_ODOM_POSES = 5000
MAX_POINTS_PER_KEYFRAME = 1500
MAX_CLOUD_POINTS_PER_ROBOT = 90000
DEFAULT_ALIGNMENT_FILE = "/results/apriltags_pass1/robotA_to_robotB_alignment.json"


IMAGE_CHANNELS = {
    "rgb8": 3,
    "bgr8": 3,
    "rgba8": 4,
    "bgra8": 4,
    "mono8": 1,
    "8uc1": 1,
}


def color_for(robot_id):
    return ROBOT_COLORS[robot_id % len(ROBOT_COLORS)]


def point_from_pose(pose):
    return Point(x=pose.position.x, y=pose.position.y, z=pose.position.z)


def yaw_from_quaternion(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


def quaternion_from_yaw(yaw):
    quat = Pose().orientation
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def pose_to_xyyaw(pose):
    return (
        float(pose.position.x),
        float(pose.position.y),
        yaw_from_quaternion(pose.orientation),
    )


def compose_planar_pose(base_pose, delta_pose):
    base_yaw = yaw_from_quaternion(base_pose.orientation)
    delta_yaw = yaw_from_quaternion(delta_pose.orientation)
    cos_yaw = math.cos(base_yaw)
    sin_yaw = math.sin(base_yaw)

    pose = Pose()
    pose.position.x = (
        base_pose.position.x
        + cos_yaw * delta_pose.position.x
        - sin_yaw * delta_pose.position.y
    )
    pose.position.y = (
        base_pose.position.y
        + sin_yaw * delta_pose.position.x
        + cos_yaw * delta_pose.position.y
    )
    pose.position.z = base_pose.position.z + delta_pose.position.z
    pose.orientation = quaternion_from_yaw(base_yaw + delta_yaw)
    return pose


def load_alignment():
    path = FilePath(DEFAULT_ALIGNMENT_FILE)
    if not path.exists():
        return None
    with path.open() as alignment_file:
        data = json.load(alignment_file)
    transform = data["transform_source_odom_to_observer_odom"]
    return {
        "source_robot_index": int(data.get("source_robot_index", 0)),
        "rotation": transform["rotation_2d"],
        "x_m": float(transform["x_m"]),
        "y_m": float(transform["y_m"]),
        "yaw_rad": float(transform["yaw_rad"]),
    }


def point_offsets(cloud):
    offsets = {field.name: field.offset for field in cloud.fields}
    if not {"x", "y", "z"}.issubset(offsets):
        return None
    field_by_name = {field.name: field for field in cloud.fields}
    if any(field_by_name[name].datatype != PointField.FLOAT32 for name in ("x", "y", "z")):
        return None
    return offsets


def cloud_to_local_display_points(cloud):
    offsets = point_offsets(cloud)
    if offsets is None or cloud.point_step <= 0:
        return []

    count = int(cloud.width) * int(cloud.height)
    if count <= 0:
        return []

    step = max(1, math.ceil(count / MAX_POINTS_PER_KEYFRAME))
    data = bytes(cloud.data)
    points = []
    for idx in range(0, count, step):
        base = idx * cloud.point_step
        try:
            optical_x = struct.unpack_from("<f", data, base + offsets["x"])[0]
            optical_y = struct.unpack_from("<f", data, base + offsets["y"])[0]
            optical_z = struct.unpack_from("<f", data, base + offsets["z"])[0]
        except struct.error:
            break
        if not (
            math.isfinite(optical_x)
            and math.isfinite(optical_y)
            and math.isfinite(optical_z)
        ):
            continue

        # Swarm-SLAM RGB-D keyframe clouds are in the optical camera frame.
        # Convert to the display convention used by the AGV paths:
        # x forward, y left, z up.
        points.append((optical_z, -optical_x, -optical_y))
    return points


def transform_cloud_points(points, pose):
    if not points:
        return []
    x, y, yaw = pose_to_xyyaw(pose)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    z_offset = float(pose.position.z)
    transformed = []
    for px, py, pz in points:
        transformed.append(
            (
                x + cos_yaw * px - sin_yaw * py,
                y + sin_yaw * px + cos_yaw * py,
                z_offset + pz,
            )
        )
    return transformed


def make_xyz_cloud(points, frame_id, stamp):
    cloud = PointCloud2()
    cloud.header = Header()
    cloud.header.frame_id = frame_id
    cloud.header.stamp = stamp
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = False
    data = bytearray(cloud.row_step)
    for idx, (x, y, z) in enumerate(points):
        struct.pack_into("<fff", data, idx * cloud.point_step, float(x), float(y), float(z))
    cloud.data = bytes(data)
    return cloud


class SwarmSlamVizBridge(Node):
    def __init__(self):
        super().__init__("agv_swarmslam_viz_bridge")
        self.declare_parameter("robot_count", 2)
        self.robot_count = int(self.get_parameter("robot_count").value)
        self.alignment = load_alignment()
        if self.alignment is not None:
            self.get_logger().info(
                "Applying display-only AprilTag alignment to robot %d: x=%.3f y=%.3f yaw=%.1fdeg"
                % (
                    self.alignment["source_robot_index"],
                    self.alignment["x_m"],
                    self.alignment["y_m"],
                    math.degrees(self.alignment["yaw_rad"]),
                )
            )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/agv_swarmslam/pose_graph_markers", 10
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/agv_swarmslam/keyframe_pointcloud", 10
        )
        self.robot_cloud_pubs = {}
        self.path_pubs = {}
        self.odom_path_pubs = {}
        self.image_display_pubs = {}
        self.pose_values = {}
        self.pose_edges = {}
        self.display_pose_lookup = {}
        self.local_cloud_points = defaultdict(dict)
        self.odom_paths = {}
        for robot_id in range(self.robot_count):
            self.robot_cloud_pubs[robot_id] = self.create_publisher(
                PointCloud2,
                f"/agv_swarmslam/robot_{robot_id}/keyframe_cloud",
                10,
            )
            self.image_display_pubs[robot_id] = self.create_publisher(
                Image,
                f"/agv_swarmslam/robot_{robot_id}/color_display",
                10,
            )
            self.odom_paths[robot_id] = Path()
            self.path_pub(robot_id)
            self.odom_path_pub(robot_id)

        for robot_id, path in self.odom_paths.items():
            path.header.frame_id = VIS_FRAME

        self.create_subscription(
            PoseGraph, "/cslam/viz/pose_graph", self.pose_graph_callback, 10
        )
        for robot_id in range(self.robot_count):
            self.create_subscription(
                Odometry,
                f"/r{robot_id}/odom",
                lambda msg, rid=robot_id: self.odom_callback(rid, msg),
                50,
            )
            self.create_subscription(
                Image,
                f"/r{robot_id}/color/image_raw",
                lambda msg, rid=robot_id: self.image_display_callback(rid, msg),
                10,
            )
        self.create_subscription(
            VizPointCloud,
            "/cslam/viz/keyframe_pointcloud",
            self.pointcloud_callback,
            10,
        )

    def path_pub(self, robot_id):
        if robot_id not in self.path_pubs:
            self.path_pubs[robot_id] = self.create_publisher(
                Path, f"/agv_swarmslam/robot_{robot_id}/path", 10
            )
        return self.path_pubs[robot_id]

    def odom_path_pub(self, robot_id):
        if robot_id not in self.odom_path_pubs:
            self.odom_path_pubs[robot_id] = self.create_publisher(
                Path, f"/agv_swarmslam/robot_{robot_id}/odom_path", 10
            )
        return self.odom_path_pubs[robot_id]

    def odom_callback(self, robot_id, msg):
        path = self.odom_paths[robot_id]
        path.header.frame_id = VIS_FRAME
        path.header.stamp = self.get_clock().now().to_msg()

        pose_stamped = PoseStamped()
        pose_stamped.header = path.header
        pose_stamped.pose = self.display_pose(robot_id, msg.pose.pose)
        path.poses.append(pose_stamped)
        if len(path.poses) > MAX_ODOM_POSES:
            path.poses = path.poses[-MAX_ODOM_POSES:]

        self.odom_path_pub(robot_id).publish(path)

    def display_pose(self, robot_id, pose):
        if (
            self.alignment is None
            or robot_id != self.alignment["source_robot_index"]
        ):
            return pose

        rot = self.alignment["rotation"]
        x = pose.position.x
        y = pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)

        display = Pose()
        display.position.x = rot[0][0] * x + rot[0][1] * y + self.alignment["x_m"]
        display.position.y = rot[1][0] * x + rot[1][1] * y + self.alignment["y_m"]
        display.position.z = pose.position.z
        display.orientation = quaternion_from_yaw(yaw + self.alignment["yaw_rad"])
        return display

    def image_display_callback(self, robot_id, msg):
        encoding = msg.encoding.lower()
        channels = IMAGE_CHANNELS.get(encoding)
        if channels is None or msg.height <= 0 or msg.width <= 0:
            return

        row_bytes = msg.width * channels
        if msg.step < row_bytes or len(msg.data) < msg.height * msg.step:
            return

        rotated = bytearray()
        data = bytes(msg.data)
        for row in range(msg.height - 1, -1, -1):
            start = row * msg.step
            pixels = data[start : start + row_bytes]
            padding = data[start + row_bytes : start + msg.step]
            for col in range(msg.width - 1, -1, -1):
                pixel_start = col * channels
                rotated.extend(pixels[pixel_start : pixel_start + channels])
            rotated.extend(padding)

        display_msg = Image()
        display_msg.header = msg.header
        display_msg.height = msg.height
        display_msg.width = msg.width
        display_msg.encoding = msg.encoding
        display_msg.is_bigendian = msg.is_bigendian
        display_msg.step = msg.step
        display_msg.data = bytes(rotated)
        self.image_display_pubs[robot_id].publish(display_msg)

    def pose_graph_callback(self, msg):
        for value in msg.values:
            self.pose_values[(value.key.robot_id, value.key.keyframe_id)] = value

        for edge in msg.edges:
            edge_key = (
                edge.key_from.robot_id,
                edge.key_from.keyframe_id,
                edge.key_to.robot_id,
                edge.key_to.keyframe_id,
            )
            self.pose_edges[edge_key] = edge

        self.publish_pose_graph()

    def edge_chain_poses(self, robot_id):
        chain_edges = {}
        for edge in self.pose_edges.values():
            if edge.key_from.robot_id != robot_id or edge.key_to.robot_id != robot_id:
                continue
            if edge.key_to.keyframe_id != edge.key_from.keyframe_id + 1:
                continue
            chain_edges[edge.key_from.keyframe_id] = edge

        if not chain_edges:
            return []

        pose = Pose()
        pose.orientation.w = 1.0
        poses = [(0, pose)]
        current_id = 0
        while current_id in chain_edges:
            edge = chain_edges[current_id]
            pose = compose_planar_pose(pose, edge.measurement)
            current_id = edge.key_to.keyframe_id
            poses.append((current_id, pose))
        return poses

    def publish_pose_graph(self):
        frame_id = VIS_FRAME
        stamp = self.get_clock().now().to_msg()
        values_by_robot = defaultdict(list)
        for value in self.pose_values.values():
            values_by_robot[value.key.robot_id].append(value)

        robots = set(values_by_robot.keys())
        for edge in self.pose_edges.values():
            robots.add(edge.key_from.robot_id)
            robots.add(edge.key_to.robot_id)

        display_poses_by_robot = {}
        pose_lookup = {}
        fallback_robots = set()
        for robot_id in sorted(robots):
            values = sorted(values_by_robot[robot_id], key=lambda item: item.key.keyframe_id)
            if len(values) > 1:
                poses = [
                    (value.key.keyframe_id, self.display_pose(robot_id, value.pose))
                    for value in values
                ]
            else:
                poses = [
                    (keyframe_id, self.display_pose(robot_id, pose))
                    for keyframe_id, pose in self.edge_chain_poses(robot_id)
                ]
                if len(poses) > 1:
                    fallback_robots.add(robot_id)
                elif values:
                    poses = [
                        (
                            values[0].key.keyframe_id,
                            self.display_pose(robot_id, values[0].pose),
                        )
                    ]
            display_poses_by_robot[robot_id] = poses
            for keyframe_id, pose in poses:
                pose_lookup[(robot_id, keyframe_id)] = pose

        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = frame_id
        delete_marker.header.stamp = stamp
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        marker_id = 1
        for robot_id, poses in sorted(display_poses_by_robot.items()):
            color = color_for(robot_id)

            path = Path()
            path.header.frame_id = frame_id
            path.header.stamp = stamp

            pose_marker = Marker()
            pose_marker.header = path.header
            pose_marker.ns = f"robot_{robot_id}_poses"
            pose_marker.id = marker_id
            marker_id += 1
            pose_marker.type = Marker.SPHERE_LIST
            pose_marker.action = Marker.ADD
            pose_marker.scale.x = 0.14
            pose_marker.scale.y = 0.14
            pose_marker.scale.z = 0.14
            pose_marker.color = color

            path_marker = Marker()
            path_marker.header = path.header
            path_marker.ns = f"robot_{robot_id}_path"
            path_marker.id = marker_id
            marker_id += 1
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.08
            path_marker.color = color

            for _, pose in poses:
                pose_stamped = PoseStamped()
                pose_stamped.header = path.header
                pose_stamped.pose = pose
                path.poses.append(pose_stamped)
                pose_marker.points.append(point_from_pose(pose))
                path_marker.points.append(point_from_pose(pose))

            markers.markers.append(pose_marker)
            markers.markers.append(path_marker)
            self.path_pub(robot_id).publish(path)

            if poses:
                label_marker = Marker()
                label_marker.header = path.header
                label_marker.ns = f"robot_{robot_id}_label"
                label_marker.id = marker_id
                marker_id += 1
                label_marker.type = Marker.TEXT_VIEW_FACING
                label_marker.action = Marker.ADD
                label_marker.pose.position = point_from_pose(poses[-1][1])
                label_marker.pose.position.z += 0.35
                label_marker.pose.orientation.w = 1.0
                label_marker.scale.z = 0.12
                label_marker.color = color
                suffix = " edge path" if robot_id in fallback_robots else ""
                label_marker.text = f"r{robot_id} {len(poses)} KF{suffix}"
                markers.markers.append(label_marker)

        edge_marker = Marker()
        edge_marker.header.frame_id = frame_id
        edge_marker.header.stamp = stamp
        edge_marker.ns = "pose_graph_edges"
        edge_marker.id = marker_id
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.04
        edge_marker.color = ColorRGBA(r=0.75, g=0.75, b=0.75, a=0.75)

        inter_edge_marker = Marker()
        inter_edge_marker.header = edge_marker.header
        inter_edge_marker.ns = "inter_robot_edges"
        inter_edge_marker.id = marker_id + 1
        inter_edge_marker.type = Marker.LINE_LIST
        inter_edge_marker.action = Marker.ADD
        inter_edge_marker.scale.x = 0.09
        inter_edge_marker.color = ColorRGBA(r=1.0, g=0.9, b=0.15, a=1.0)

        for edge in self.pose_edges.values():
            start = pose_lookup.get((edge.key_from.robot_id, edge.key_from.keyframe_id))
            end = pose_lookup.get((edge.key_to.robot_id, edge.key_to.keyframe_id))
            if start is None or end is None:
                continue
            target = (
                inter_edge_marker
                if edge.key_from.robot_id != edge.key_to.robot_id
                else edge_marker
            )
            target.points.append(point_from_pose(start))
            target.points.append(point_from_pose(end))

        markers.markers.append(edge_marker)
        markers.markers.append(inter_edge_marker)
        self.marker_pub.publish(markers)
        self.display_pose_lookup = pose_lookup
        self.publish_clouds()

    def pointcloud_callback(self, msg):
        robot_id = int(msg.robot_id)
        keyframe_id = int(msg.keyframe_id)
        points = cloud_to_local_display_points(msg.pointcloud)
        if not points:
            return
        self.local_cloud_points[robot_id][keyframe_id] = points
        self.publish_clouds()

    def publish_clouds(self):
        if not self.display_pose_lookup:
            return
        stamp = self.get_clock().now().to_msg()
        latest_points = None
        for robot_id, clouds in sorted(self.local_cloud_points.items()):
            chunks = []
            for keyframe_id, local_points in sorted(clouds.items()):
                pose = self.display_pose_lookup.get((robot_id, keyframe_id))
                if pose is None:
                    continue
                transformed = transform_cloud_points(local_points, pose)
                if transformed:
                    chunks.extend(transformed)
                    latest_points = transformed
            if not chunks:
                continue
            if len(chunks) > MAX_CLOUD_POINTS_PER_ROBOT:
                step = max(1, math.ceil(len(chunks) / MAX_CLOUD_POINTS_PER_ROBOT))
                chunks = chunks[::step]
            cloud = make_xyz_cloud(chunks, VIS_FRAME, stamp)
            if robot_id not in self.robot_cloud_pubs:
                self.robot_cloud_pubs[robot_id] = self.create_publisher(
                    PointCloud2,
                    f"/agv_swarmslam/robot_{robot_id}/keyframe_cloud",
                    10,
                )
            self.robot_cloud_pubs[robot_id].publish(cloud)
        if latest_points:
            self.cloud_pub.publish(make_xyz_cloud(latest_points, VIS_FRAME, stamp))


def main():
    rclpy.init()
    node = SwarmSlamVizBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if "Unable to convert call argument to Python object" not in str(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
