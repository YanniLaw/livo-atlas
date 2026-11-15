"""Bounded messages for the experimental online Gaussian worker."""

import json
import math
import re
import select
import struct
import time

import numpy as np

_MAX_HEADER = 64 * 1024
_MAX_BODY = 16 * 1024 * 1024
_MAX_PIXELS = 1_000_000
_PREFIX = struct.Struct("<IQ")
_SESSION = re.compile(r"[0-9a-f]{32}")
_UINT = re.compile(r"0|[1-9][0-9]*")
_TYPES = {"hello", "hello_ok", "reset", "reset_ok", "view", "candidate",
          "decision", "commit_ok", "discard_ok", "error", "stop"}


def _unsigned(value, name):
    if not isinstance(value, str) or len(value) > 20 or not _UINT.fullmatch(value):
        raise ValueError("Invalid " + name)
    result = int(value)
    if result > 2**64 - 1:
        raise ValueError("Invalid " + name)
    return result


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError("Invalid " + name)
    return value


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


def _pose(value, name):
    if not isinstance(value, list) or len(value) != 16:
        raise ValueError("Invalid " + name)
    pose = np.array([_number(element, name) for element in value],
                    dtype=np.float64).reshape(4, 4)
    rotation = pose[:3, :3]
    if (not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-9, rtol=0) or
            not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0) or
            abs(np.linalg.det(rotation) - 1) > 1e-6):
        raise ValueError("Invalid " + name)
    return pose


def _dimensions(header):
    width = _integer(header.get("width"), "width", 1, _MAX_PIXELS)
    height = _integer(header.get("height"), "height", 1, _MAX_PIXELS)
    pixels = width * height
    if pixels > _MAX_PIXELS:
        raise ValueError("Image exceeds the pixel limit")
    return width, height, pixels


def _validate_view(header, body):
    width, height, pixels = _dimensions(header)
    if len(body) != 9 * pixels:
        raise ValueError("Incorrect view body size")
    frame_id = _unsigned(header.get("keyframe_id"), "keyframe ID")
    frame_count = _unsigned(header.get("frame_count"), "frame count")
    optimized = _unsigned(header.get("optimized_frame_count"), "optimized frame count")
    calibration = _unsigned(header.get("calibration_id"), "calibration ID")
    version = _unsigned(header.get("pose_version"), "pose version")
    if (frame_id >= frame_count or optimized > frame_count or calibration != 0 or
            header.get("parent_frame") != ("odom" if version == 0 else "map")):
        raise ValueError("Invalid view identity or pose frontier")
    for name in ("state_timestamp", "cloud_timestamp", "image_timestamp",
                 "inverse_exposure_time", "fx", "fy", "cx", "cy"):
        value = _number(header.get(name), name)
        if name in ("fx", "fy") and value <= 0:
            raise ValueError("Focal lengths must be positive")
    _pose(header.get("camera_from_submap"), "camera pose")
    _pose(header.get("parent_from_submap"), "submap pose")
    image_mask = np.frombuffer(body, dtype=np.uint8, count=pixels, offset=3 * pixels)
    depth_mask = np.frombuffer(body, dtype=np.uint8, count=pixels, offset=4 * pixels)
    depth = np.frombuffer(body, dtype="<f4", count=pixels, offset=5 * pixels)
    if (np.any(image_mask > 1) or np.any(depth_mask > 1) or
            np.any(depth_mask > image_mask) or not np.isfinite(depth).all() or
            np.any(depth[depth_mask == 1] <= 0) or
            np.any(depth[depth_mask == 0] != 0)):
        raise ValueError("Invalid view masks or depth")
    return width, height, pixels


def _validate_message(header, body):
    if not isinstance(header, dict) or type(header.get("schema")) is not int:
        raise ValueError("Invalid message header")
    kind = header.get("type")
    if header["schema"] != 1 or not isinstance(kind, str) or kind not in _TYPES:
        raise ValueError("Unsupported message type or schema")
    session = header.get("session_id")
    if not isinstance(session, str) or not _SESSION.fullmatch(session):
        raise ValueError("Invalid session ID")
    if not isinstance(body, bytes) or len(body) > _MAX_BODY:
        raise ValueError("Invalid message body")
    if kind in ("reset", "reset_ok"):
        _unsigned(header.get("generation"), "generation")
        _unsigned(header.get("pose_version"), "pose version")
        if "submap_id" in header:
            _unsigned(header["submap_id"], "submap ID")
    if kind in ("view", "candidate", "decision", "commit_ok", "discard_ok"):
        for name in ("job_id", "generation", "submap_id", "pose_version"):
            _unsigned(header.get(name), name)
    if kind in ("view", "candidate"):
        for name in ("membership_revision", "frame_count", "optimized_frame_count"):
            _unsigned(header.get(name), name)
    if kind == "view":
        _validate_view(header, body)
    elif kind == "candidate":
        _, _, pixels = _dimensions(header)
        if len(body) != 3 * pixels:
            raise ValueError("Incorrect preview body size")
        frame_ids = header.get("frame_ids")
        if not isinstance(frame_ids, list) or not 1 <= len(frame_ids) <= 10:
            raise ValueError("Invalid candidate frame IDs")
        ids = [_unsigned(value, "frame ID") for value in frame_ids]
        if ids != sorted(set(ids)):
            raise ValueError("Candidate frame IDs must be sorted and unique")
        if (_unsigned(header["optimized_frame_count"], "optimized frame count") >
                _unsigned(header["frame_count"], "frame count") or
                ids[-1] >= _unsigned(header["frame_count"], "frame count")):
            raise ValueError("Invalid candidate pose frontier")
        _integer(header.get("steps"), "steps", 20, 20)
        _integer(header.get("gaussian_count"), "Gaussian count", 1, 10_000)
        _integer(header.get("supervised_depth_pixels"), "depth count", 0,
                 20 * _MAX_PIXELS)
        if any(_number(header.get(name), name) < 0
               for name in ("rgb_l1", "depth_l1_m")):
            raise ValueError("Losses must be nonnegative")
        for name in ("nonzero_gradient", "parameter_updated"):
            if name in header:
                _integer(header[name], name, 0, 1)
        for name in ("gpu_peak_allocated_bytes", "gpu_peak_reserved_bytes",
                     "worker_peak_rss_bytes"):
            if name in header:
                _unsigned(header[name], name)
    elif body:
        raise ValueError("Unexpected body")
    if kind == "decision" and header.get("action") not in ("accept", "discard"):
        raise ValueError("Invalid candidate decision")
    if kind == "error" and header.get("code") not in (
            "gpu_unavailable", "extension_unavailable", "protocol_error", "worker_error"):
        raise ValueError("Invalid worker error")
    if kind == "hello_ok":
        for name in ("python", "torch", "gsplat", "cuda", "device"):
            if not isinstance(header.get(name), str) or not header[name]:
                raise ValueError("Invalid worker software metadata")
        free = _unsigned(header.get("gpu_free_bytes"), "free GPU bytes")
        total = _unsigned(header.get("gpu_total_bytes"), "total GPU bytes")
        if not total or free > total:
            raise ValueError("Invalid GPU memory metadata")


def _reject_unicode_escapes(text):
    quoted = False
    escaped = False
    for value in text:
        if quoted:
            if escaped:
                if value == "u":
                    raise ValueError("Unicode escapes are not supported by OpenCV 4.2")
                escaped = False
            elif value == "\\":
                escaped = True
            elif value == '"':
                quoted = False
        elif value == '"':
            quoted = True


def encode_message(header, body):
    """Validate and encode one complete message."""
    _validate_message(header, body)
    text = json.dumps(header, separators=(",", ":"), allow_nan=False,
                      ensure_ascii=False)
    _reject_unicode_escapes(text)
    encoded = text.encode("utf-8")
    if not encoded or len(encoded) > _MAX_HEADER:
        raise ValueError("Header exceeds the limit")
    return _PREFIX.pack(len(encoded), len(body)) + encoded + body


def _deadline(timeout):
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
        raise ValueError("Timeout must be positive")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("Timeout must be positive")
    return time.monotonic() + timeout


def _wait(sock, event, deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError("Message transfer timed out")
    poller = select.poll()
    poller.register(sock.fileno(), event)
    if not poller.poll(max(1, math.ceil(remaining * 1000))):
        raise TimeoutError("Message transfer timed out")


def _read_exact(sock, size, deadline):
    result = bytearray()
    while len(result) < size:
        if time.monotonic() >= deadline:
            raise TimeoutError("Message transfer timed out")
        try:
            part = sock.recv(size - len(result))
        except (BlockingIOError, InterruptedError):
            _wait(sock, select.POLLIN, deadline)
            continue
        if not part:
            raise OSError("Socket closed during message")
        result.extend(part)
    return bytes(result)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError("Nonfinite JSON value: " + value)


def read_message(sock, timeout):
    """Read one frame with a single monotonic transfer deadline."""
    deadline = _deadline(timeout)
    previous = sock.gettimeout()
    sock.setblocking(False)
    try:
        prefix = _read_exact(sock, _PREFIX.size, deadline)
        header_size, body_size = _PREFIX.unpack(prefix)
        if not 1 <= header_size <= _MAX_HEADER or body_size > _MAX_BODY:
            raise ValueError("Message size exceeds the limit")
        encoded = _read_exact(sock, header_size, deadline)
        body = _read_exact(sock, body_size, deadline)
    finally:
        sock.settimeout(previous)
    try:
        text = encoded.decode("utf-8")
        _reject_unicode_escapes(text)
        header = json.loads(text, object_pairs_hook=_unique_object,
                            parse_constant=_reject_constant)
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ValueError("Invalid JSON header") from error
    _validate_message(header, body)
    return header, body


def write_message(sock, header, body, timeout):
    """Write one complete frame without blocking beyond its deadline."""
    deadline = _deadline(timeout)
    encoded = memoryview(encode_message(header, body))
    previous = sock.gettimeout()
    sock.setblocking(False)
    try:
        while encoded:
            if time.monotonic() >= deadline:
                raise TimeoutError("Message transfer timed out")
            try:
                count = sock.send(encoded)
            except (BlockingIOError, InterruptedError):
                _wait(sock, select.POLLOUT, deadline)
                continue
            if count == 0:
                raise OSError("Socket closed during message")
            encoded = encoded[count:]
    finally:
        sock.settimeout(previous)


def decode_view(header, body):
    """Convert a validated BGR observation into owned P6-03b-style arrays."""
    _validate_message(header, body)
    if header["type"] != "view":
        raise ValueError("Expected a view")
    width, height, pixels = _dimensions(header)
    bgr = np.frombuffer(body, dtype=np.uint8, count=3 * pixels).reshape(height, width, 3)
    rgb = bgr[..., ::-1].astype(np.float32)
    rgb /= 255.0
    image_mask = np.frombuffer(body, dtype=np.uint8, count=pixels,
                               offset=3 * pixels).reshape(height, width).astype(bool)
    depth_mask = np.frombuffer(body, dtype=np.uint8, count=pixels,
                               offset=4 * pixels).reshape(height, width).astype(bool)
    depth = np.frombuffer(body, dtype="<f4", count=pixels,
                          offset=5 * pixels).reshape(height, width).astype(np.float32)
    result = dict(header)
    result.update({
        "keyframe_id": int(header["keyframe_id"]),
        "calibration_id": int(header["calibration_id"]),
        "split": "train", "rgb": rgb, "image_mask": image_mask,
        "depth_mask": depth_mask, "depth_m": depth,
        "K": np.array([[header["fx"], 0, header["cx"]],
                       [0, header["fy"], header["cy"]], [0, 0, 1]], dtype=np.float64),
        "camera_from_parent": _pose(header["camera_from_submap"], "camera pose"),
    })
    return result
