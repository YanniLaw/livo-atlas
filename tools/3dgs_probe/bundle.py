"""Read and validate the temporary offline reconstruction-view bundle."""

import json
import math
from pathlib import Path
import re

import numpy as np
from PIL import Image


def _unsigned(value, name):
    if not isinstance(value, str) or not re.fullmatch(r"0|[1-9][0-9]*", value):
        raise ValueError("Invalid " + name)
    result = int(value)
    if result > 2**64 - 1:
        raise ValueError("Invalid " + name)
    return result


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("Invalid " + name)
    try:
        result = float(value)
    except OverflowError as error:
        raise ValueError("Invalid " + name) from error
    if not math.isfinite(result):
        raise ValueError("Invalid " + name)
    return result


def _asset(root, value):
    if not isinstance(value, str) or not value:
        raise ValueError("Invalid asset path")
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("Asset path must stay inside the bundle")
    path = (root / relative).resolve(strict=True)
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError("Asset path must stay inside the bundle")
    return path


def _read_png(path, size, mode):
    with Image.open(path) as image:
        if image.format != "PNG" or image.mode != mode or image.size != size:
            raise ValueError("Unexpected PNG format or dimensions")
        return np.array(image, dtype=np.uint8, copy=True)


def _pose(value):
    try:
        pose = np.asarray(value, dtype=np.float64).reshape(4, 4)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("Invalid camera pose") from error
    rotation = pose[:3, :3]
    if (not np.isfinite(pose).all() or
            not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-9, rtol=0) or
            not np.allclose(rotation.T @ rotation, np.eye(3),
                            atol=1e-6, rtol=0) or
            abs(np.linalg.det(rotation) - 1) > 1e-6):
        raise ValueError("Camera pose must be a finite rigid transform")
    return pose


def _view(root, data, source_sha):
    if not isinstance(data, dict):
        raise ValueError("Invalid view")
    keyframe_id = _unsigned(data.get("keyframe_id"), "keyframe ID")
    calibration_id = _unsigned(data.get("calibration_id"), "calibration ID")
    pose_version = _unsigned(data.get("pose_version"), "pose version")
    frame_count = _unsigned(data.get("frame_count"), "frame count")
    optimized_count = _unsigned(data.get("optimized_frame_count"),
                                "optimized frame count")
    if (keyframe_id >= frame_count or pose_version != 0 or
            optimized_count != 0 or data.get("parent_frame") != "odom"):
        raise ValueError("Unsupported pose snapshot")
    split = data.get("split")
    if split not in ("train", "validation"):
        raise ValueError("Invalid view split")
    times = {name: _number(data.get(name), name) for name in (
        "state_timestamp", "cloud_timestamp", "image_timestamp",
        "inverse_exposure_time")}
    width, height = data.get("width"), data.get("height")
    if (type(width) is not int or type(height) is not int or
            width <= 0 or height <= 0):
        raise ValueError("Invalid image dimensions")
    fx, fy, cx, cy = (_number(data.get(name), name)
                      for name in ("fx", "fy", "cx", "cy"))
    if fx <= 0 or fy <= 0:
        raise ValueError("Focal lengths must be positive")
    intrinsic = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
                         dtype=np.float64)
    pose = _pose(data.get("camera_from_parent"))
    depth_path = _asset(root, data.get("depth_m"))
    if depth_path.stat().st_size != width * height * 4:
        raise ValueError("Unexpected depth byte count")
    size = (width, height)
    rgb = _read_png(_asset(root, data.get("image")), size, "RGB")
    image_mask = _read_png(_asset(root, data.get("image_mask")), size, "L")
    depth_mask = _read_png(_asset(root, data.get("depth_mask")), size, "L")
    if np.any(image_mask > 1) or np.any(depth_mask > 1):
        raise ValueError("Masks must contain only 0 or 1")
    image_mask = image_mask.astype(bool)
    depth_mask = depth_mask.astype(bool)
    depth = np.fromfile(depth_path, dtype="<f4").reshape(height, width)
    depth = np.asarray(depth, dtype=np.float32)
    if (not np.isfinite(depth).all() or np.any(depth_mask & ~image_mask) or
            np.any(depth[depth_mask] <= 0) or np.any(depth[~depth_mask] != 0)):
        raise ValueError("Depth and validity masks are inconsistent")
    result = {
        "keyframe_id": keyframe_id,
        "calibration_id": calibration_id,
        "pose_version": pose_version,
        "frame_count": frame_count,
        "optimized_frame_count": optimized_count,
        "parent_frame": "odom",
        "source_manifest_sha256": source_sha,
        "split": split,
        "rgb": rgb.astype(np.float32) / 255.0,
        "image_mask": image_mask,
        "depth_mask": depth_mask,
        "depth_m": depth,
        "K": intrinsic,
        "camera_from_parent": pose,
        **times,
    }
    if "parent_from_body" in data:
        result["parent_from_body"] = _pose(data["parent_from_body"])
    return result


def load_bundle(path):
    """Read a completed bundle; no input arrays alias its files."""
    root = Path(path).resolve(strict=True)

    def reject_constant(value):
        raise ValueError("Nonfinite JSON value: " + value)

    document = json.loads((root / "bundle.json").read_text(encoding="utf-8"),
                          parse_constant=reject_constant)
    if (not isinstance(document, dict) or type(document.get("schema")) is not int
            or document["schema"] != 1):
        raise ValueError("Unsupported bundle schema")
    source_sha = document.get("source_manifest_sha256")
    if (not isinstance(source_sha, str) or
            not re.fullmatch(r"[0-9a-f]{64}", source_sha)):
        raise ValueError("Invalid source manifest SHA-256")
    entries = document.get("views")
    if not isinstance(entries, list) or not entries:
        raise ValueError("Bundle has no views")
    views = [_view(root, entry, source_sha) for entry in entries]
    ids = [view["keyframe_id"] for view in views]
    if len(set(ids)) != len(ids) or {view["split"] for view in views} != {
            "train", "validation"}:
        raise ValueError("View IDs or splits are invalid")
    return views
