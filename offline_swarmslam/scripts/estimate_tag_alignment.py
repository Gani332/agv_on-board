#!/usr/bin/env python3
"""Estimate a 2D robot-odom alignment from robot-mounted AprilTag detections."""

import argparse
import csv
import json
import math
from collections import deque
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


ODOM_TOPIC = "/odom"
TF_TOPICS = ("/tf", "/tf_static")
DEFAULT_ROOT = Path(__file__).resolve().parents[1]


def stamp_sec(stamp):
    if hasattr(stamp, "sec"):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def quaternion_matrix(quat):
    x, y, z, w = quat
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float64,
    )


def transform_matrix(translation, quaternion):
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = quaternion_matrix(quaternion)
    mat[:3, 3] = np.asarray(translation, dtype=np.float64)
    return mat


def invert_transform(mat):
    inv = np.eye(4, dtype=np.float64)
    inv[:3, :3] = mat[:3, :3].T
    inv[:3, 3] = -inv[:3, :3] @ mat[:3, 3]
    return inv


def yaw_from_quaternion(quat):
    x, y, z, w = quat
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def planar_pose_matrix(x, y, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    mat = np.eye(4, dtype=np.float64)
    mat[0, 0] = cos_yaw
    mat[0, 1] = -sin_yaw
    mat[1, 0] = sin_yaw
    mat[1, 1] = cos_yaw
    mat[0, 3] = x
    mat[1, 3] = y
    return mat


def clean_frame(name):
    return name.lstrip("/")


def read_odom(path):
    rows = []
    with AnyReader([path]) as reader:
        conns = [conn for conn in reader.connections if conn.topic == ODOM_TOPIC]
        for conn, _, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            rows.append(
                (
                    stamp_sec(msg.header.stamp),
                    float(p.x),
                    float(p.y),
                    yaw_from_quaternion((float(q.x), float(q.y), float(q.z), float(q.w))),
                )
            )
    return rows


def interpolate_odom(rows, stamp):
    if not rows or stamp < rows[0][0] or stamp > rows[-1][0]:
        return None
    lo = 0
    hi = len(rows) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if rows[mid][0] < stamp:
            lo = mid + 1
        else:
            hi = mid - 1

    if lo == 0:
        return rows[0][1:]
    if lo >= len(rows):
        return rows[-1][1:]

    t0, x0, y0, yaw0 = rows[lo - 1]
    t1, x1, y1, yaw1 = rows[lo]
    if t1 <= t0:
        return x0, y0, yaw0
    alpha = (stamp - t0) / (t1 - t0)
    dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    return (
        x0 + alpha * (x1 - x0),
        y0 + alpha * (y1 - y0),
        yaw0 + alpha * dyaw,
    )


def read_tf_graph(path):
    graph = {}
    with AnyReader([path]) as reader:
        conns = [conn for conn in reader.connections if conn.topic in TF_TOPICS]
        for conn, _, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            for transform in msg.transforms:
                parent = clean_frame(transform.header.frame_id)
                child = clean_frame(transform.child_frame_id)
                if (parent, child) in graph:
                    continue
                t = transform.transform.translation
                q = transform.transform.rotation
                mat = transform_matrix(
                    (float(t.x), float(t.y), float(t.z)),
                    (float(q.x), float(q.y), float(q.z), float(q.w)),
                )
                graph[(parent, child)] = mat
    return graph


def chain_transform(graph, source, target):
    source = clean_frame(source)
    target = clean_frame(target)
    queue = deque([(source, np.eye(4, dtype=np.float64))])
    visited = {source}
    while queue:
        frame, mat = queue.popleft()
        if frame == target:
            return mat
        for (parent, child), edge in graph.items():
            if parent == frame and child not in visited:
                visited.add(child)
                queue.append((child, mat @ edge))
            if child == frame and parent not in visited:
                visited.add(parent)
                queue.append((parent, mat @ invert_transform(edge)))
    raise RuntimeError("No TF chain from {} to {}".format(source, target))


def read_detections(path, observer_robot, source_robot, tag_id, tag_owner_rule):
    detections = []
    with path.open(newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            if row["robot"] != observer_robot or int(row["tag_id"]) != tag_id:
                continue
            if tag_owner_rule == "observer_other_robot":
                inferred_owner = row.get("inferred_tag_owner", "")
                if inferred_owner and inferred_owner != source_robot:
                    continue
            if not row["tx_m"] or not row["ty_m"] or not row["tz_m"]:
                continue
            detections.append(
                {
                    "stamp": float(row["stamp_sec"]),
                    "frame": int(row["frame"]),
                    "p_cam_tag": np.array(
                        [float(row["tx_m"]), float(row["ty_m"]), float(row["tz_m"]), 1.0],
                        dtype=np.float64,
                    ),
                }
            )
    return detections


def fit_rigid_2d(source_points, target_points):
    source = np.asarray(source_points, dtype=np.float64)
    target = np.asarray(target_points, dtype=np.float64)
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    u, _, vt = np.linalg.svd(source_centered.T @ target_centered)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0.0:
        vt[-1, :] *= -1.0
        rot = vt.T @ u.T
    trans = target_mean - rot @ source_mean
    residuals = np.linalg.norm((source @ rot.T + trans) - target, axis=1)
    return rot, trans, residuals


def robust_fit(source_points, target_points):
    source = np.asarray(source_points, dtype=np.float64)
    target = np.asarray(target_points, dtype=np.float64)
    keep = np.ones(len(source), dtype=bool)
    for _ in range(4):
        rot, trans, residuals = fit_rigid_2d(source[keep], target[keep])
        median = float(np.median(residuals))
        mad = float(np.median(np.abs(residuals - median)))
        threshold = max(0.25, median + 2.5 * max(mad, 1e-6))
        next_keep = np.zeros(len(source), dtype=bool)
        full_residuals = np.linalg.norm((source @ rot.T + trans) - target, axis=1)
        next_keep[full_residuals <= threshold] = True
        if next_keep.sum() < 6 or np.array_equal(next_keep, keep):
            keep = next_keep if next_keep.sum() >= 6 else keep
            break
        keep = next_keep
    rot, trans, residuals = fit_rigid_2d(source[keep], target[keep])
    full_residuals = np.linalg.norm((source @ rot.T + trans) - target, axis=1)
    return rot, trans, keep, full_residuals


def build_point_pairs(detections, source_odom, observer_odom, observer_base_to_camera, source_time_offset_sec):
    source_points = []
    target_points = []
    used_frames = []
    for detection in detections:
        source_pose = interpolate_odom(source_odom, detection["stamp"] + source_time_offset_sec)
        observer_pose = interpolate_odom(observer_odom, detection["stamp"])
        if source_pose is None or observer_pose is None:
            continue
        source_x, source_y, _ = source_pose
        observer_x, observer_y, observer_yaw = observer_pose
        observer_odom_to_base = planar_pose_matrix(observer_x, observer_y, observer_yaw)
        observed_in_observer_odom = (
            observer_odom_to_base @ observer_base_to_camera @ detection["p_cam_tag"]
        )
        source_points.append([source_x, source_y])
        target_points.append(
            [float(observed_in_observer_odom[0]), float(observed_in_observer_odom[1])]
        )
        used_frames.append(detection["frame"])
    return source_points, target_points, used_frames


def estimate_with_offset(detections, source_odom, observer_odom, observer_base_to_camera, offset):
    source_points, target_points, used_frames = build_point_pairs(
        detections,
        source_odom,
        observer_odom,
        observer_base_to_camera,
        offset,
    )
    if len(source_points) < 6:
        return None
    rot, trans, keep, residuals = robust_fit(source_points, target_points)
    if keep.sum() < 6:
        return None
    inlier_residuals = residuals[keep]
    return {
        "offset": offset,
        "source_points": source_points,
        "target_points": target_points,
        "used_frames": used_frames,
        "rot": rot,
        "trans": trans,
        "keep": keep,
        "residuals": residuals,
        "score": float(np.median(inlier_residuals)),
    }


def search_time_offset(detections, source_odom, observer_odom, observer_base_to_camera, search_sec, step_sec):
    candidates = []
    for offset in np.arange(-search_sec, search_sec + step_sec * 0.5, step_sec):
        result = estimate_with_offset(
            detections,
            source_odom,
            observer_odom,
            observer_base_to_camera,
            float(offset),
        )
        if result is not None:
            candidates.append(result)
    if not candidates:
        return None

    best = min(candidates, key=lambda item: item["score"])
    refine_start = best["offset"] - step_sec
    refine_stop = best["offset"] + step_sec
    refine_step = max(step_sec / 5.0, 0.02)
    refined = []
    for offset in np.arange(refine_start, refine_stop + refine_step * 0.5, refine_step):
        result = estimate_with_offset(
            detections,
            source_odom,
            observer_odom,
            observer_base_to_camera,
            float(offset),
        )
        if result is not None:
            refined.append(result)
    return min(refined or candidates, key=lambda item: item["score"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--detections", type=Path, default=DEFAULT_ROOT / "results/apriltags_pass1/detections.csv")
    parser.add_argument("--robot-a-bag", type=Path, default=DEFAULT_ROOT / "downloaded_bags/scenario1_square_20260506/robotA/robotA_scenario1_square_pass1.bag")
    parser.add_argument("--robot-b-bag", type=Path, default=DEFAULT_ROOT / "downloaded_bags/scenario1_square_20260506/robotB/robotB_scenario1_square_pass1.bag")
    parser.add_argument("--observer-robot", default="robotB")
    parser.add_argument("--source-robot", default="robotA")
    parser.add_argument("--target-tag-id", type=int, default=1)
    parser.add_argument(
        "--tag-base-note",
        default="duplicate ID 1 tags; robotA tag is rear-facing, robotB tag is left-facing",
    )
    parser.add_argument(
        "--tag-owner-rule",
        choices=("unique_id", "observer_other_robot"),
        default="observer_other_robot",
        help="Use 'observer_other_robot' for the duplicate-ID two-robot scenario: a tag observed by one robot is assigned to the other robot.",
    )
    parser.add_argument("--out", type=Path, default=DEFAULT_ROOT / "results/apriltags_pass1/robotA_to_robotB_alignment.json")
    parser.add_argument("--time-offset-search-sec", type=float, default=12.0)
    parser.add_argument("--time-offset-step-sec", type=float, default=0.25)
    args = parser.parse_args()

    bag_by_robot = {
        "robotA": args.robot_a_bag,
        "robotB": args.robot_b_bag,
    }
    robot_index_by_label = {
        "robotA": 0,
        "robotB": 1,
    }
    if args.observer_robot not in bag_by_robot or args.source_robot not in bag_by_robot:
        raise RuntimeError("observer_robot and source_robot must be robotA or robotB")

    observer_odom = read_odom(bag_by_robot[args.observer_robot])
    source_odom = read_odom(bag_by_robot[args.source_robot])
    tf_graph = read_tf_graph(bag_by_robot[args.observer_robot])
    observer_base_to_camera = chain_transform(tf_graph, "base_footprint", "camera_color_optical_frame")
    if args.tag_owner_rule == "observer_other_robot" and args.observer_robot == args.source_robot:
        raise RuntimeError("observer_other_robot requires observer_robot and source_robot to be different")
    detections = read_detections(
        args.detections,
        args.observer_robot,
        args.source_robot,
        args.target_tag_id,
        args.tag_owner_rule,
    )

    result = search_time_offset(
        detections,
        source_odom,
        observer_odom,
        observer_base_to_camera,
        args.time_offset_search_sec,
        args.time_offset_step_sec,
    )
    if result is None:
        raise RuntimeError("Could not estimate alignment with enough paired tag/odom observations")

    source_points = result["source_points"]
    target_points = result["target_points"]
    used_frames = result["used_frames"]
    rot = result["rot"]
    trans = result["trans"]
    keep = result["keep"]
    residuals = result["residuals"]
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    inlier_residuals = residuals[keep]
    source = np.asarray(source_points, dtype=np.float64)
    target = np.asarray(target_points, dtype=np.float64)

    output = {
        "description": "Display-only 2D transform from {} odom into {} odom, estimated from duplicate-ID AprilTag detections under the two-robot observer-other-robot rule.".format(
            args.source_robot,
            args.observer_robot,
        ),
        "source_robot": args.source_robot,
        "observer_robot": args.observer_robot,
        "target_tag_id": args.target_tag_id,
        "tag_owner_rule": args.tag_owner_rule,
        "tag_owner_rule_note": (
            "For this two-robot scenario, a configured tag seen by observer_robot is inferred to belong to source_robot. "
            "This is valid only if cameras cannot see their own robot's tag and no third duplicate tag is present."
        ),
        "tag_mount_note": args.tag_base_note,
        "source_robot_index": robot_index_by_label[args.source_robot],
        "target_robot_index": robot_index_by_label[args.observer_robot],
        "transform_source_odom_to_observer_odom": {
            "x_m": float(trans[0]),
            "y_m": float(trans[1]),
            "yaw_rad": float(yaw),
            "yaw_deg": float(math.degrees(yaw)),
            "rotation_2d": rot.tolist(),
        },
        "source_time_offset_sec": float(result["offset"]),
        "source_time_offset_note": "source odom was sampled at observer_detection_stamp + source_time_offset_sec",
        "num_detections": len(detections),
        "num_pairs": len(source_points),
        "num_inliers": int(keep.sum()),
        "residual_m": {
            "median_all": float(np.median(residuals)),
            "median_inlier": float(np.median(inlier_residuals)),
            "p95_inlier": float(np.percentile(inlier_residuals, 95)),
            "max_inlier": float(np.max(inlier_residuals)),
        },
        "frame_range_inliers": [
            int(min(np.asarray(used_frames)[keep])),
            int(max(np.asarray(used_frames)[keep])),
        ],
        "source_points_sample": source[keep][:5].tolist(),
        "target_points_sample": target[keep][:5].tolist(),
    }

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(output, indent=2, sort_keys=True))

    print("AprilTag alignment estimated")
    print("output={}".format(args.out))
    print("pairs={} inliers={}".format(len(source_points), int(keep.sum())))
    print("source_time_offset_sec={:.3f}".format(result["offset"]))
    print(
        "{}_odom -> {}_odom: x={:.3f} m y={:.3f} m yaw={:.1f} deg".format(
            args.source_robot,
            args.observer_robot,
            trans[0],
            trans[1],
            math.degrees(yaw),
        )
    )
    print("inlier residual median={:.3f} m p95={:.3f} m".format(np.median(inlier_residuals), np.percentile(inlier_residuals, 95)))


if __name__ == "__main__":
    main()
