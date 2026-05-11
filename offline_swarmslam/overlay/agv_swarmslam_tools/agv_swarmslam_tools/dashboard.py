import argparse
import io
import json
import math
import struct
import threading
import time
from pathlib import Path as FilePath

import matplotlib

matplotlib.use("Agg")

from flask import Flask, Response
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np
import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

try:
    from cslam_common_interfaces.msg import PoseGraph, VizPointCloud
except ImportError:
    PoseGraph = None
    VizPointCloud = None

try:
    import cv2
except ImportError:
    cv2 = None


COLORS = {
    "slam0": "#1f77ff",
    "slam1": "#ff8c1a",
    "odom0": "#7fc8ff",
    "odom1": "#ffd27a",
    "cloud": "#d8d8d8",
    "cloud0": "#78b7ff",
    "cloud1": "#ffbd73",
}

TAG_IDS = {0, 1}
TAG_SIZE_M = 0.10
MAX_CLOUD_POINTS_PER_KEYFRAME = 1800
MAX_RENDER_CLOUD_POINTS_PER_ROBOT = 25000


def load_alignment(path):
    if path is None:
        return None
    path = FilePath(path)
    if not path.exists():
        return None
    with path.open() as alignment_file:
        data = json.load(alignment_file)
    transform = data["transform_source_odom_to_observer_odom"]
    return {
        "source_robot_index": int(data.get("source_robot_index", 0)),
        "target_robot_index": int(data.get("target_robot_index", 1)),
        "rotation": np.asarray(transform["rotation_2d"], dtype=np.float32),
        "translation": np.asarray([transform["x_m"], transform["y_m"]], dtype=np.float32),
        "yaw_deg": float(transform["yaw_deg"]),
        "residual_median": float(data.get("residual_m", {}).get("median_inlier", 0.0)),
        "num_inliers": int(data.get("num_inliers", 0)),
    }


def transform_points(points, alignment, robot_index):
    if alignment is None or robot_index != alignment["source_robot_index"] or len(points) == 0:
        return points
    return points @ alignment["rotation"].T + alignment["translation"]


def transform_cloud_alignment(points, alignment, robot_index):
    if alignment is None or robot_index != alignment["source_robot_index"] or len(points) == 0:
        return points
    transformed = points.copy()
    transformed[:, :2] = points[:, :2] @ alignment["rotation"].T + alignment["translation"]
    return transformed


def path_points(msg):
    return np.array(
        [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses],
        dtype=np.float32,
    )


def path_length(points):
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def yaw_from_quaternion(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


def pose_to_xyyaw(pose):
    return (
        float(pose.position.x),
        float(pose.position.y),
        yaw_from_quaternion(pose.orientation),
    )


def compose_planar_pose(base_pose, delta_pose):
    base_x, base_y, base_yaw = base_pose
    delta_x, delta_y, delta_yaw = pose_to_xyyaw(delta_pose)
    cos_yaw = math.cos(base_yaw)
    sin_yaw = math.sin(base_yaw)
    return (
        base_x + cos_yaw * delta_x - sin_yaw * delta_y,
        base_y + sin_yaw * delta_x + cos_yaw * delta_y,
        base_yaw + delta_yaw,
    )


def image_array(msg):
    enc = msg.encoding.lower()
    channels = 1
    if enc in ("rgb8", "bgr8"):
        channels = 3
    elif enc in ("rgba8", "bgra8"):
        channels = 4
    elif enc in ("mono8", "8uc1"):
        channels = 1
    else:
        return None

    expected = msg.height * msg.width * channels
    if len(msg.data) < expected:
        return None

    arr = np.frombuffer(msg.data, dtype=np.uint8, count=expected)
    if channels == 1:
        return arr.reshape(msg.height, msg.width)

    arr = arr.reshape(msg.height, msg.width, channels)
    if enc in ("bgr8", "bgra8"):
        arr = arr[..., [2, 1, 0] + ([3] if channels == 4 else [])]
    if channels == 4:
        arr = arr[..., :3]
    return arr


def make_apriltag_detector():
    if cv2 is None or not hasattr(cv2, "aruco"):
        return None, None
    if not hasattr(cv2.aruco, "DICT_APRILTAG_36h11"):
        return None, None

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()

    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(dictionary, parameters), None
    return dictionary, parameters


def detect_apriltags(detector, parameters, image):
    if detector is None:
        return [], None

    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    else:
        gray = image

    if hasattr(detector, "detectMarkers"):
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, detector, parameters=parameters)

    if ids is None:
        return [], None

    detections = []
    for idx, tag_id in enumerate(ids.flatten()):
        tag_id = int(tag_id)
        if tag_id not in TAG_IDS:
            continue
        detections.append((tag_id, corners[idx].reshape(4, 2)))
    return detections, ids


def annotate_apriltags(image, detector, parameters):
    detections, _ = detect_apriltags(detector, parameters, image)
    if not detections:
        return image, []

    annotated = image.copy()
    for tag_id, pts in detections:
        pts_i = np.round(pts).astype(np.int32)
        cv2.polylines(annotated, [pts_i], True, (0, 255, 0), 3)
        x, y = pts_i[0]
        cv2.putText(
            annotated,
            f"id {tag_id}",
            (int(x), max(18, int(y) - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 0, 0),
            2,
            cv2.LINE_AA,
        )
    return annotated, detections


def cloud_points(msg, max_points=MAX_CLOUD_POINTS_PER_KEYFRAME):
    field_offsets = {field.name: field.offset for field in msg.fields}
    if not {"x", "y", "z"}.issubset(field_offsets):
        return np.empty((0, 3), dtype=np.float32)

    count = msg.width * msg.height
    if count <= 0 or msg.point_step <= 0:
        return np.empty((0, 3), dtype=np.float32)

    step = max(1, math.ceil(count / max_points))
    points = []
    data = msg.data
    for idx in range(0, count, step):
        base = idx * msg.point_step
        try:
            x = struct.unpack_from("<f", data, base + field_offsets["x"])[0]
            y = struct.unpack_from("<f", data, base + field_offsets["y"])[0]
            z = struct.unpack_from("<f", data, base + field_offsets["z"])[0]
        except struct.error:
            break
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            # Swarm-SLAM RGB-D keyframe clouds are generated from optical-frame
            # depth. Convert to a robot-like display frame: x forward, y left,
            # z up. This is for visualization only.
            points.append((z, -x, -y))
    return np.asarray(points, dtype=np.float32)


def transform_cloud_by_pose(points, pose):
    if len(points) == 0:
        return points
    x, y, yaw = pose
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    transformed = points.copy()
    px = points[:, 0]
    py = points[:, 1]
    transformed[:, 0] = x + cos_yaw * px - sin_yaw * py
    transformed[:, 1] = y + sin_yaw * px + cos_yaw * py
    return transformed


def sample_points(points, max_points):
    if len(points) <= max_points:
        return points
    step = max(1, math.ceil(len(points) / max_points))
    return points[::step]


def set_equal_3d_axes(ax, points):
    if len(points) == 0:
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        ax.set_zlim(-0.2, 1.2)
        return

    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) / 2.0
    span = float(max(maxs - mins))
    span = max(span, 0.8)
    pad = span * 0.12
    half = span / 2.0 + pad
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    z_min = max(-0.4, float(mins[2]) - pad)
    z_max = min(3.0, float(maxs[2]) + pad)
    if z_max <= z_min:
        z_max = z_min + 0.5
    ax.set_zlim(z_min, z_max)


class LiveDashboard(Node):
    def __init__(self, rotate_images, alignment_file):
        super().__init__("agv_swarmslam_dashboard")
        self.rotate_images = rotate_images
        self.alignment = load_alignment(alignment_file)
        self.lock = threading.Lock()
        self.paths = {
            "slam0": np.empty((0, 2), dtype=np.float32),
            "slam1": np.empty((0, 2), dtype=np.float32),
            "odom0": np.empty((0, 2), dtype=np.float32),
            "odom1": np.empty((0, 2), dtype=np.float32),
        }
        self.images = {"r0": None, "r1": None}
        self.cloud = np.empty((0, 3), dtype=np.float32)
        self.clouds_by_robot = {}
        self.pose_values = {}
        self.pose_edges = {}
        self.keyframe_poses_by_robot = {}
        self.updated = {}
        self.tag_detector, self.tag_parameters = make_apriltag_detector()

        subscriptions = [
            (Path, "/agv_swarmslam/robot_0/path", lambda msg: self.set_path("slam0", msg)),
            (Path, "/agv_swarmslam/robot_1/path", lambda msg: self.set_path("slam1", msg)),
            (
                Path,
                "/agv_swarmslam/robot_0/odom_path",
                lambda msg: self.set_path("odom0", msg),
            ),
            (
                Path,
                "/agv_swarmslam/robot_1/odom_path",
                lambda msg: self.set_path("odom1", msg),
            ),
            (Image, "/r0/color/image_raw", lambda msg: self.set_image("r0", msg)),
            (Image, "/r1/color/image_raw", lambda msg: self.set_image("r1", msg)),
            (PointCloud2, "/agv_swarmslam/keyframe_pointcloud", self.set_cloud),
        ]
        if VizPointCloud is not None:
            subscriptions.append((VizPointCloud, "/cslam/viz/keyframe_pointcloud", self.set_keyframe_cloud))
        if PoseGraph is not None:
            subscriptions.append((PoseGraph, "/cslam/viz/pose_graph", self.set_pose_graph))

        for msg_type, topic, callback in subscriptions:
            self.create_subscription(msg_type, topic, callback, 10)

    def set_path(self, key, msg):
        with self.lock:
            self.paths[key] = path_points(msg)
            self.updated[key] = time.time()

    def set_image(self, key, msg):
        arr = image_array(msg)
        if arr is None:
            return
        if self.rotate_images == 180:
            arr = np.rot90(arr, 2).copy()
        with self.lock:
            self.images[key] = arr.copy()
            self.updated[key] = time.time()

    def set_cloud(self, msg):
        with self.lock:
            self.cloud = cloud_points(msg)
            self.updated["cloud"] = time.time()

    def set_keyframe_cloud(self, msg):
        points = cloud_points(msg.pointcloud)
        if len(points) == 0:
            return
        robot_id = int(msg.robot_id)
        keyframe_id = int(msg.keyframe_id)
        with self.lock:
            self.clouds_by_robot.setdefault(robot_id, {})[keyframe_id] = points
            self.updated["cloud"] = time.time()

    def set_pose_graph(self, msg):
        with self.lock:
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
            self.keyframe_poses_by_robot = self.compute_keyframe_poses()

    def edge_chain_poses(self, robot_id):
        chain_edges = {}
        for edge in self.pose_edges.values():
            if edge.key_from.robot_id != robot_id or edge.key_to.robot_id != robot_id:
                continue
            if edge.key_to.keyframe_id != edge.key_from.keyframe_id + 1:
                continue
            chain_edges[edge.key_from.keyframe_id] = edge

        poses = {}
        pose = (0.0, 0.0, 0.0)
        current_id = 0
        poses[current_id] = pose
        while current_id in chain_edges:
            edge = chain_edges[current_id]
            pose = compose_planar_pose(pose, edge.measurement)
            current_id = edge.key_to.keyframe_id
            poses[current_id] = pose
        return poses

    def compute_keyframe_poses(self):
        values_by_robot = {}
        for value in self.pose_values.values():
            values_by_robot.setdefault(value.key.robot_id, []).append(value)

        robots = set(values_by_robot.keys())
        for edge in self.pose_edges.values():
            robots.add(edge.key_from.robot_id)
            robots.add(edge.key_to.robot_id)

        result = {}
        for robot_id in sorted(robots):
            values = sorted(values_by_robot.get(robot_id, []), key=lambda item: item.key.keyframe_id)
            if len(values) > 1:
                result[robot_id] = {
                    value.key.keyframe_id: pose_to_xyyaw(value.pose)
                    for value in values
                }
            else:
                fallback = self.edge_chain_poses(robot_id)
                if fallback:
                    result[robot_id] = fallback
                elif values:
                    result[robot_id] = {
                        values[0].key.keyframe_id: pose_to_xyyaw(values[0].pose)
                    }
        return result

    def snapshot(self):
        with self.lock:
            clouds_by_robot = {
                robot_id: {keyframe_id: points.copy() for keyframe_id, points in clouds.items()}
                for robot_id, clouds in self.clouds_by_robot.items()
            }
            poses_by_robot = {
                robot_id: dict(poses)
                for robot_id, poses in self.keyframe_poses_by_robot.items()
            }
            latest_clouds = {}
            for robot_id, clouds in self.clouds_by_robot.items():
                if clouds:
                    latest_clouds[robot_id] = list(clouds.values())[-1].copy()
            return (
                {key: value.copy() for key, value in self.paths.items()},
                {key: None if value is None else value.copy() for key, value in self.images.items()},
                self.cloud.copy(),
                clouds_by_robot,
                latest_clouds,
                poses_by_robot,
                dict(self.updated),
            )

    def render_png(self):
        paths, images, latest_cloud, clouds_by_robot, latest_clouds, poses_by_robot, updated = self.snapshot()
        display_paths = {
            "slam0": transform_points(paths["slam0"], self.alignment, 0),
            "slam1": transform_points(paths["slam1"], self.alignment, 1),
            "odom0": transform_points(paths["odom0"], self.alignment, 0),
            "odom1": transform_points(paths["odom1"], self.alignment, 1),
        }
        map_clouds = {}
        latest_only_clouds = {}
        for robot_id, clouds in clouds_by_robot.items():
            chunks = []
            pose_lookup = poses_by_robot.get(robot_id, {})
            for keyframe_id, points in clouds.items():
                pose = pose_lookup.get(keyframe_id)
                if pose is None:
                    continue
                chunks.append(transform_cloud_by_pose(points, pose))
            if chunks:
                map_clouds[robot_id] = sample_points(
                    np.vstack(chunks),
                    MAX_RENDER_CLOUD_POINTS_PER_ROBOT,
                )
            elif robot_id in latest_clouds:
                latest_only_clouds[robot_id] = latest_clouds[robot_id]
        if not map_clouds and len(latest_cloud):
            latest_only_clouds[None] = latest_cloud

        combined_clouds = {
            robot_id: transform_cloud_alignment(points, self.alignment, robot_id)
            for robot_id, points in map_clouds.items()
        }

        fig = Figure(figsize=(15.2, 9.2), dpi=120, facecolor="#101214")
        canvas = FigureCanvasAgg(fig)
        grid = fig.add_gridspec(
            4,
            4,
            width_ratios=[1.1, 1.1, 0.82, 0.82],
            height_ratios=[0.82, 0.82, 0.68, 0.68],
            wspace=0.32,
            hspace=0.44,
        )

        ax_combined = fig.add_subplot(grid[:2, :2], projection="3d")
        ax_combined.set_facecolor("#171a1d")
        ax_combined.set_title("Combined 3D keyframe map", color="white", loc="left", fontsize=11)
        ax_combined.set_xlabel("x (m)", color="#cfd4d8", labelpad=3)
        ax_combined.set_ylabel("y (m)", color="#cfd4d8", labelpad=3)
        ax_combined.set_zlabel("z up (m)", color="#cfd4d8", labelpad=3)
        ax_combined.tick_params(colors="#cfd4d8", labelsize=7)
        ax_combined.view_init(elev=26, azim=-58)
        combined_points = []
        for robot_id, cloud in sorted(combined_clouds.items()):
            color = COLORS.get(f"cloud{robot_id}", COLORS["cloud"])
            if len(cloud):
                combined_points.append(cloud)
                ax_combined.scatter(
                    cloud[:, 0],
                    cloud[:, 1],
                    cloud[:, 2],
                    s=1.3,
                    c=color,
                    alpha=0.23,
                    depthshade=False,
                    label=f"robot {robot_id}",
                )
        if combined_points:
            set_equal_3d_axes(ax_combined, np.vstack(combined_points))
            ax_combined.legend(loc="upper right", fontsize=8, facecolor="#202428", edgecolor="#555", labelcolor="white")
            if self.alignment is None:
                ax_combined.text2D(
                    0.02,
                    0.93,
                    "unregistered robot frames",
                    transform=ax_combined.transAxes,
                    color="#ffdf7e",
                    fontsize=8,
                )
        else:
            ax_combined.text2D(
                0.5,
                0.5,
                "waiting for registered keyframe maps",
                transform=ax_combined.transAxes,
                color="#cfd4d8",
                ha="center",
                va="center",
            )
            set_equal_3d_axes(ax_combined, np.empty((0, 3), dtype=np.float32))

        for robot_id in (0, 1):
            ax_map = fig.add_subplot(grid[robot_id, 2], projection="3d")
            ax_map.set_facecolor("#171a1d")
            ax_map.set_title(f"robot {robot_id} local map", color="white", loc="left", fontsize=9)
            ax_map.set_xlabel("x", color="#cfd4d8", labelpad=1)
            ax_map.set_ylabel("y", color="#cfd4d8", labelpad=1)
            ax_map.set_zlabel("z", color="#cfd4d8", labelpad=1)
            ax_map.tick_params(colors="#cfd4d8", labelsize=6)
            ax_map.view_init(elev=24, azim=-58)
            color = COLORS.get(f"cloud{robot_id}", COLORS["cloud"])
            cloud = map_clouds.get(robot_id)
            if cloud is not None and len(cloud):
                ax_map.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], s=1.5, c=color, alpha=0.25, depthshade=False)
                set_equal_3d_axes(ax_map, cloud)
            else:
                fallback = latest_only_clouds.get(robot_id)
                if fallback is not None and len(fallback):
                    ax_map.scatter(fallback[:, 0], fallback[:, 1], fallback[:, 2], s=1.5, c=color, alpha=0.25, depthshade=False)
                    set_equal_3d_axes(ax_map, fallback)
                    ax_map.text2D(0.02, 0.92, "waiting for keyframe poses", transform=ax_map.transAxes, color="#ffdf7e", fontsize=8)
                else:
                    ax_map.text2D(0.5, 0.5, "waiting for keyframe map", transform=ax_map.transAxes, color="#cfd4d8", ha="center", va="center")
                    set_equal_3d_axes(ax_map, np.empty((0, 3), dtype=np.float32))

        tag_suffix = " (tag aligned)" if self.alignment is not None else ""
        labels = {
            "slam0": f"robot 0 Swarm-SLAM{tag_suffix}",
            "slam1": "robot 1 Swarm-SLAM",
            "odom0": f"robot 0 raw odom{tag_suffix}",
            "odom1": "robot 1 raw odom",
        }
        widths = {"slam0": 2.6, "slam1": 2.6, "odom0": 1.2, "odom1": 1.2}
        styles = {"slam0": "-", "slam1": "-", "odom0": "--", "odom1": "--"}

        ax_paths = fig.add_subplot(grid[2:, :3])
        ax_paths.set_facecolor("#171a1d")
        ax_paths.grid(True, color="#3a3f44", linewidth=0.6)
        ax_paths.set_title("Pose graph paths only", color="white", loc="left", fontsize=10)
        ax_paths.set_xlabel("x (m)", color="#cfd4d8")
        ax_paths.set_ylabel("y (m)", color="#cfd4d8")
        ax_paths.tick_params(colors="#cfd4d8")

        all_points = []
        for key, pts in display_paths.items():
            if len(pts):
                all_points.append(pts)
                ax_paths.plot(
                    pts[:, 0],
                    pts[:, 1],
                    styles[key],
                    color=COLORS[key],
                    linewidth=widths[key],
                    label=labels[key],
                )
                ax_paths.scatter(pts[-1, 0], pts[-1, 1], s=42, color=COLORS[key])

        if all_points:
            merged = np.vstack(all_points)
            x_min, y_min = merged.min(axis=0)
            x_max, y_max = merged.max(axis=0)
            span = max(x_max - x_min, y_max - y_min, 0.5)
            cx, cy = (x_min + x_max) / 2.0, (y_min + y_max) / 2.0
            pad = span * 0.2
            ax_paths.set_xlim(cx - span / 2 - pad, cx + span / 2 + pad)
            ax_paths.set_ylim(cy - span / 2 - pad, cy + span / 2 + pad)
        ax_paths.set_aspect("equal", adjustable="box")
        ax_paths.legend(loc="upper right", fontsize=8, facecolor="#202428", edgecolor="#555", labelcolor="white")

        ax_info = fig.add_subplot(grid[:2, 3])
        ax_info.axis("off")
        ax_info.set_facecolor("#101214")
        lines = ["Live status", ""]
        for key in ("slam0", "slam1", "odom0", "odom1"):
            pts = display_paths[key]
            length = path_length(pts)
            age = time.time() - updated.get(key, 0) if key in updated else None
            age_text = "no data" if age is None else f"{age:.1f}s ago"
            lines.append(f"{labels[key]}")
            lines.append(f"  poses: {len(pts):4d}   length: {length:.2f} m   updated: {age_text}")
        lines.extend(
            [
                "",
                f"robot 0 map points shown: {len(map_clouds.get(0, latest_only_clouds.get(0, [])))}",
                f"robot 1 map points shown: {len(map_clouds.get(1, latest_only_clouds.get(1, [])))}",
                "",
                "Interpretation:",
                "top-left = combined registered 3D map",
                "top-middle = separated robot-local maps",
                "bottom = pose graph / odom paths only",
                "solid = Swarm-SLAM graph path",
                "dashed = raw bag odometry",
                "edge-chain fallback is used when",
                "optimized values are not published",
            ]
        )
        if self.alignment is not None:
            lines.extend(
                [
                    "",
                    "AprilTag alignment:",
                    "robot 0 displayed in robot 1 odom frame",
                    f"inliers: {self.alignment['num_inliers']}  median residual: {self.alignment['residual_median']:.2f} m",
                    f"yaw offset: {self.alignment['yaw_deg']:.1f} deg",
                ]
            )
        ax_info.text(
            0,
            1,
            "\n".join(lines),
            va="top",
            ha="left",
            color="#e8edf2",
            fontsize=8,
            family="monospace",
        )

        for idx, robot in enumerate(("r0", "r1")):
            ax_img = fig.add_subplot(grid[idx + 2, 3], frame_on=True)
            ax_img.set_title(f"{robot} RGB replay", color="white", loc="left", fontsize=10)
            ax_img.axis("off")
            img = images[robot]
            if img is None:
                ax_img.text(0.5, 0.5, "waiting for image", color="white", ha="center", va="center")
            else:
                annotated, detections = annotate_apriltags(img, self.tag_detector, self.tag_parameters)
                ax_img.imshow(annotated)
                tag_text = ", ".join(f"id {tag_id}" for tag_id, _ in detections)
                if not tag_text:
                    tag_text = "no tag 0/1"
                ax_img.text(
                    0.02,
                    0.94,
                    f"tag36h11 {TAG_SIZE_M * 1000:.0f} mm: {tag_text}",
                    transform=ax_img.transAxes,
                    color="white",
                    fontsize=8,
                    bbox={"facecolor": "#111", "alpha": 0.65, "edgecolor": "none", "pad": 3},
                )

        buf = io.BytesIO()
        fig.subplots_adjust(left=0.045, right=0.985, top=0.94, bottom=0.07)
        canvas.print_png(buf)
        return buf.getvalue()


def create_app(node):
    app = Flask(__name__)

    @app.route("/")
    def index():
        return """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>AGV Swarm-SLAM Dashboard</title>
  <style>
    body { margin: 0; background: #101214; color: #e8edf2; font-family: system-ui, sans-serif; }
    img { width: 100vw; height: auto; display: block; }
  </style>
</head>
<body>
  <img id="frame" src="/frame.png">
  <script>
    setInterval(() => {
      document.getElementById('frame').src = '/frame.png?t=' + Date.now();
    }, 1000);
  </script>
</body>
</html>"""

    @app.route("/frame.png")
    def frame_png():
        return Response(node.render_png(), mimetype="image/png")

    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--rotate-images",
        type=int,
        choices=(0, 180),
        default=180,
        help="Rotate replay camera panels for display only.",
    )
    parser.add_argument(
        "--alignment-file",
        default="/results/apriltags_pass1/robotA_to_robotB_alignment.json",
        help="Optional display-only 2D alignment JSON from estimate_tag_alignment.py.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = LiveDashboard(args.rotate_images, args.alignment_file)
    app = create_app(node)

    thread = threading.Thread(
        target=lambda: app.run(host=args.host, port=args.port, debug=False, use_reloader=False),
        daemon=True,
    )
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
