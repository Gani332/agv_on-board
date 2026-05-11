#!/usr/bin/env python3
"""Scan recorded RGB images for tag36h11 AprilTags."""

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw
from rosbags.highlevel import AnyReader


RGB_TOPIC = "/camera/color/image_raw"
INFO_TOPIC = "/camera/color/camera_info"
DEFAULT_ROOT = Path(__file__).resolve().parents[1]


def parse_bag(value):
    if "=" in value:
        label, path = value.split("=", 1)
        return label, Path(path)
    path = Path(value)
    return path.stem, path


def stamp_sec(stamp):
    if hasattr(stamp, "sec"):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def image_to_rgb(msg):
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    encoding = msg.encoding.lower()
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if encoding in ("rgb8", "bgr8"):
        arr = data.reshape((height, step))[:, : width * 3].reshape((height, width, 3))
        if encoding == "bgr8":
            arr = arr[:, :, ::-1]
        return arr.copy()

    if encoding in ("rgba8", "bgra8"):
        arr = data.reshape((height, step))[:, : width * 4].reshape((height, width, 4))[:, :, :3]
        if encoding == "bgra8":
            arr = arr[:, :, ::-1]
        return arr.copy()

    if encoding in ("mono8", "8uc1"):
        gray = data.reshape((height, step))[:, :width]
        return np.repeat(gray[:, :, None], 3, axis=2)

    raise RuntimeError("Unsupported image encoding: {}".format(msg.encoding))


def make_detector():
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "DICT_APRILTAG_36h11"):
        raise RuntimeError("OpenCV was built without cv2.aruco.DICT_APRILTAG_36h11")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()

    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(dictionary, params), None
    return dictionary, params


def detect(detector, params, rgb):
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    if hasattr(detector, "detectMarkers"):
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, detector, parameters=params)
    if ids is None:
        return []
    return [(int(tag_id), corners[idx].reshape(4, 2)) for idx, tag_id in enumerate(ids.flatten())]


def estimate_pose(corners, camera_matrix, dist_coeffs, tag_size):
    if camera_matrix is None:
        return None

    half = tag_size / 2.0
    object_points = np.array(
        [
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ],
        dtype=np.float32,
    )
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        corners.astype(np.float32),
        camera_matrix,
        dist_coeffs if dist_coeffs is not None else None,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None
    t = tvec.reshape(3)
    return {
        "tx_m": float(t[0]),
        "ty_m": float(t[1]),
        "tz_m": float(t[2]),
        "distance_m": float(np.linalg.norm(t)),
    }


def draw_detection(rgb, detections):
    annotated = rgb.copy()
    for tag_id, corners in detections:
        pts = np.round(corners).astype(np.int32)
        cv2.polylines(annotated, [pts], True, (0, 255, 0), 3)
        x, y = pts[0]
        cv2.putText(
            annotated,
            "id {}".format(tag_id),
            (int(x), max(18, int(y) - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 0, 0),
            2,
            cv2.LINE_AA,
        )
    return annotated


def build_other_robot_owner_map(bag_specs):
    labels = [label for label, _ in bag_specs]
    if len(labels) != 2:
        raise RuntimeError("--duplicate-id-other-robot requires exactly two --bag entries")
    return {
        labels[0]: labels[1],
        labels[1]: labels[0],
    }


def save_contact_sheet(label, frames, output_path):
    if not frames:
        return None

    thumb_w, thumb_h = 320, 180
    rows = math.ceil(len(frames) / 2.0)
    sheet = Image.new("RGB", (thumb_w * 2, rows * (thumb_h + 30)), (30, 30, 30))
    draw = ImageDraw.Draw(sheet)

    for idx, item in enumerate(frames):
        image = Image.fromarray(item["image"]).resize((thumb_w, thumb_h))
        x = (idx % 2) * thumb_w
        y = (idx // 2) * (thumb_h + 30)
        sheet.paste(image, (x, y))
        draw.text((x + 4, y + thumb_h + 7), item["label"], fill=(255, 255, 255))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(output_path, quality=92)
    return str(output_path)


def scan_bag(label, path, args, writer):
    detector, params = make_detector()
    target_ids = set(args.ids)
    counts = Counter()
    frames_with_target = 0
    frames_scanned = 0
    first_stamp = None
    last_stamp = None
    distances = defaultdict(list)
    sheet_frames = []

    camera_matrix = None
    dist_coeffs = None
    info_seen = False

    with AnyReader([path]) as reader:
        topic_connections = [conn for conn in reader.connections if conn.topic in (RGB_TOPIC, INFO_TOPIC)]
        for conn, _, rawdata in reader.messages(connections=topic_connections):
            msg = reader.deserialize(rawdata, conn.msgtype)
            if conn.topic == INFO_TOPIC and not info_seen:
                camera_matrix = np.array(msg.K, dtype=np.float64).reshape((3, 3))
                dist_coeffs = np.array(msg.D, dtype=np.float64)
                info_seen = True
                continue

            if conn.topic != RGB_TOPIC:
                continue

            frame_idx = frames_scanned
            frames_scanned += 1
            if frame_idx % args.sample_step != 0:
                continue

            rgb = image_to_rgb(msg)
            stamp = stamp_sec(msg.header.stamp)
            first_stamp = stamp if first_stamp is None else first_stamp
            last_stamp = stamp

            detections = [(tag_id, corners) for tag_id, corners in detect(detector, params, rgb) if tag_id in target_ids]
            if not detections:
                continue

            frames_with_target += 1
            annotated = draw_detection(rgb, detections)

            detected_ids = []
            inferred_owner = args.owner_by_observer.get(label, "")
            for tag_id, corners in detections:
                counts[tag_id] += 1
                detected_ids.append(tag_id)
                center = corners.mean(axis=0)
                pose = estimate_pose(corners, camera_matrix, dist_coeffs, args.tag_size)
                if pose is not None:
                    distances[tag_id].append(pose["distance_m"])
                row = {
                    "robot": label,
                    "frame": frame_idx,
                    "stamp_sec": "{:.9f}".format(stamp),
                    "tag_id": tag_id,
                    "center_x": "{:.2f}".format(float(center[0])),
                    "center_y": "{:.2f}".format(float(center[1])),
                    "distance_m": "" if pose is None else "{:.4f}".format(pose["distance_m"]),
                    "tx_m": "" if pose is None else "{:.4f}".format(pose["tx_m"]),
                    "ty_m": "" if pose is None else "{:.4f}".format(pose["ty_m"]),
                    "tz_m": "" if pose is None else "{:.4f}".format(pose["tz_m"]),
                    "inferred_tag_owner": inferred_owner,
                }
                writer.writerow(row)

            if len(sheet_frames) < args.max_contact_frames:
                owner_suffix = " owner={}".format(inferred_owner) if inferred_owner else ""
                sheet_frames.append(
                    {
                        "image": annotated,
                        "label": "f{} ids={}{}".format(
                            frame_idx,
                            ",".join(str(tag_id) for tag_id in sorted(set(detected_ids))),
                            owner_suffix,
                        ),
                    }
                )

    duration = 0.0 if first_stamp is None or last_stamp is None else max(0.0, last_stamp - first_stamp)
    sheet_path = save_contact_sheet(label, sheet_frames, args.out_dir / "{}_apriltags.jpg".format(label))

    summary = {
        "robot": label,
        "bag": str(path),
        "frames_scanned": frames_scanned,
        "sample_step": args.sample_step,
        "frames_with_target": frames_with_target,
        "duration_sec": duration,
        "detections_by_id": {str(tag_id): counts[tag_id] for tag_id in sorted(target_ids)},
        "contact_sheet": sheet_path,
    }
    if label in args.owner_by_observer:
        summary["duplicate_id_owner_rule"] = "any configured tag seen by {} is inferred to belong to {}".format(
            label,
            args.owner_by_observer[label],
        )
    if info_seen:
        summary["camera_fx"] = float(camera_matrix[0, 0])
        summary["camera_fy"] = float(camera_matrix[1, 1])
    if distances:
        summary["distance_median_by_id"] = {
            str(tag_id): float(np.median(values)) for tag_id, values in sorted(distances.items())
        }
    return summary


def main():
    parser = argparse.ArgumentParser(description="Offline AprilTag check for recorded AGV bags.")
    parser.add_argument("--bag", action="append", required=True, type=parse_bag, help="LABEL=/path/to/file.bag")
    parser.add_argument("--ids", default="0,1", help="Comma-separated tag IDs to keep.")
    parser.add_argument("--tag-size", type=float, default=0.10, help="Tag code side length in metres.")
    parser.add_argument("--sample-step", type=int, default=1, help="Scan every Nth RGB frame.")
    parser.add_argument("--max-contact-frames", type=int, default=24)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_ROOT / "results/apriltags")
    parser.add_argument(
        "--duplicate-id-other-robot",
        action="store_true",
        help="For a two-robot scenario with duplicate tag IDs, infer that any observed configured tag belongs to the other robot.",
    )
    args = parser.parse_args()

    args.ids = [int(value) for value in args.ids.split(",") if value.strip()]
    args.owner_by_observer = build_other_robot_owner_map(args.bag) if args.duplicate_id_other_robot else {}
    args.out_dir.mkdir(parents=True, exist_ok=True)

    csv_path = args.out_dir / "detections.csv"
    summaries = []
    with csv_path.open("w", newline="") as csv_file:
        fieldnames = [
            "robot",
            "frame",
            "stamp_sec",
            "tag_id",
            "center_x",
            "center_y",
            "distance_m",
            "tx_m",
            "ty_m",
            "tz_m",
            "inferred_tag_owner",
        ]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for label, path in args.bag:
            summaries.append(scan_bag(label, path, args, writer))

    summary_path = args.out_dir / "summary.json"
    summary_path.write_text(json.dumps(summaries, indent=2, sort_keys=True))

    print("AprilTag offline scan complete")
    print("tag_family=tag36h11 tag_size_m={:.3f} ids={}".format(args.tag_size, ",".join(map(str, args.ids))))
    print("csv={}".format(csv_path))
    print("summary={}".format(summary_path))
    for summary in summaries:
        print(
            "{robot}: frames={frames_scanned} frames_with_tags={frames_with_target} detections={detections_by_id} sheet={contact_sheet}".format(
                **summary
            )
        )


if __name__ == "__main__":
    main()
