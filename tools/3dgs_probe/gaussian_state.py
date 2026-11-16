"""CPU-only, bounded storage for a confirmed Gaussian working set."""

from contextlib import ExitStack
import copy
from dataclasses import dataclass
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import stat
import struct
import zipfile

import numpy as np


@dataclass
class GaussianState:
    metadata: dict
    parameters: dict[str, np.ndarray]
    origin_ids: np.ndarray
    views: list[dict]


class CommitUncertainError(OSError):
    """The manifest is visible, but directory durability was not confirmed."""


_LIMITS = {"manifest.json": 512 * 1024, "gaussians.npz": 1024 * 1024,
           "observations.npz": 192 * 1024 * 1024}
_IDS = {"submap_id", "anchor_keyframe_id", "membership_revision", "generation",
        "pose_version", "frame_count", "optimized_frame_count"}
_METADATA = _IDS | {"schema", "kind", "representation", "optimizer", "capture_kind",
                    "origin_session_id", "parent_frame", "submap_state", "primary",
                    "parent_from_submap", "software"}
_HASHES = {"source_manifest_sha256", "bundle_manifest_sha256"}
_TIMES = {"state_timestamp", "cloud_timestamp", "image_timestamp", "inverse_exposure_time"}
_VIEW_FIELDS = _TIMES | {"keyframe_id", "calibration_id", "width", "height", "fx", "fy",
                         "cx", "cy", "camera_from_body", "camera_from_submap"}
_MEMORY_VIEW = _TIMES | {"keyframe_id", "calibration_id", "K", "camera_from_body",
                         "camera_from_parent", "rgb", "image_mask", "depth_mask", "depth_m"}
_PARAMETERS = {"means": 3, "quats": 4, "log_scales": 3,
               "color_logits": 3, "opacity_logits": None}


def _keys(value, required, optional=frozenset()):
    if not isinstance(value, dict) or not required <= value.keys() <= required | optional:
        raise ValueError("Missing or unknown state fields")


def _unsigned(value):
    if (not isinstance(value, str) or len(value) > 20 or
            not re.fullmatch(r"0|[1-9][0-9]*", value) or int(value) > 2**64 - 1):
        raise ValueError("Expected a canonical uint64 string")
    return int(value)


def _integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError("Integer is outside the state budget")
    return value


def _number(value):
    if (isinstance(value, (bool, np.bool_)) or
            not isinstance(value, (int, float, np.integer, np.floating))):
        raise ValueError("Expected a finite number")
    try:
        result = float(value)
    except OverflowError as error:
        raise ValueError("Expected a finite number") from error
    if not math.isfinite(result):
        raise ValueError("Expected a finite number")
    return result


def _pose(value):
    try:
        values = np.asarray(value, dtype=object).reshape(16)
        pose = np.array([_number(x) for x in values], dtype="<f8").reshape(4, 4).copy()
    except (TypeError, OverflowError) as error:
        raise ValueError("Invalid rigid transform") from error
    rotation = pose[:3, :3]
    if (not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-9, rtol=0) or
            not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0) or
            abs(np.linalg.det(rotation) - 1) > 1e-6):
        raise ValueError("Invalid rigid transform")
    return pose


def _metadata(data):
    _keys(data, _METADATA, _HASHES)
    if (type(data["schema"]) is not int or data["schema"] != 1 or
            data["kind"] != "livo_atlas.gaussian_state" or
            data["representation"] != "gsplat-sh0-window-v1" or
            data["optimizer"] != "reset_on_restore" or
            data["capture_kind"] not in ("offline_probe", "online_confirmed") or
            not isinstance(data["origin_session_id"], str) or
            not re.fullmatch(r"[0-9a-f]{32}", data["origin_session_id"])):
        raise ValueError("Unsupported state identity or representation")
    ids = {key: _unsigned(data[key]) for key in _IDS}
    if (not 1 <= ids["frame_count"] <= 512 or
            ids["optimized_frame_count"] > ids["frame_count"] or
            (ids["pose_version"] == 0 and ids["optimized_frame_count"] != 0) or
            data["parent_frame"] != ("map" if ids["pose_version"] else "odom")):
        raise ValueError("Invalid state pose frontier")
    _pose(data["parent_from_submap"])
    primary = data["primary"]
    if not isinstance(primary, list) or not 1 <= len(primary) <= 512:
        raise ValueError("Invalid primary membership")
    body_poses = {}
    for offset, entry in enumerate(primary):
        _keys(entry, {"keyframe_id", "submap_from_body"})
        key = _unsigned(entry["keyframe_id"])
        pose = _pose(entry["submap_from_body"])
        if key != ids["anchor_keyframe_id"] + offset or key >= ids["frame_count"]:
            raise ValueError("Primary membership is not a covered contiguous range")
        if offset == 0 and not np.allclose(pose, np.eye(4), atol=1e-8, rtol=0):
            raise ValueError("The anchor body must define the local origin")
        body_poses[key] = pose
    if (data["submap_state"] not in ("active", "sealed") or
            ids["membership_revision"] != len(primary) + (data["submap_state"] == "sealed")):
        raise ValueError("Membership revision disagrees with submap state")
    _keys(data["software"], {"python", "numpy", "torch", "gsplat", "cuda"})
    if any(not isinstance(x, str) or not 1 <= len(x) <= 128
           for x in data["software"].values()):
        raise ValueError("Invalid software metadata")
    for key in _HASHES & data.keys():
        if not isinstance(data[key], str) or not re.fullmatch(r"[0-9a-f]{64}", data[key]):
            raise ValueError("Invalid source digest")
    return body_poses


def _view_description(view):
    _keys(view, _MEMORY_VIEW)
    rgb = view["rgb"]
    if not isinstance(rgb, np.ndarray) or rgb.ndim != 3 or rgb.shape[-1] != 3:
        raise ValueError("Invalid RGB view shape")
    height, width = rgb.shape[:2]
    intrinsic = view["K"]
    _array(intrinsic, "<f8", (3, 3))
    if not np.array_equal(intrinsic[2], [0, 0, 1]) or intrinsic[0, 1] or intrinsic[1, 0]:
        raise ValueError("Intrinsics must use the fixed pinhole layout")
    return {"keyframe_id": str(_integer(view["keyframe_id"], 0, 511)),
            "calibration_id": str(_integer(view["calibration_id"], 0, 0)),
            "width": width, "height": height,
            "fx": float(intrinsic[0, 0]), "fy": float(intrinsic[1, 1]),
            "cx": float(intrinsic[0, 2]), "cy": float(intrinsic[1, 2]),
            "camera_from_body": _pose(view["camera_from_body"]).ravel().tolist(),
            "camera_from_submap": _pose(view["camera_from_parent"]).ravel().tolist(),
            **{name: _number(view[name]) for name in _TIMES}}


def _view_specs(views, body_poses):
    if not isinstance(views, list) or not 1 <= len(views) <= 10:
        raise ValueError("State needs one to ten retained views")
    specs, keys = {}, []
    for index, view in enumerate(views):
        _keys(view, _VIEW_FIELDS)
        key = _unsigned(view["keyframe_id"])
        width = _integer(view["width"], 1, 1_000_000)
        height = _integer(view["height"], 1, 1_000_000)
        if width * height > 1_000_000 or key not in body_poses or _unsigned(view["calibration_id"]):
            raise ValueError("View exceeds its membership or pixel budget")
        keys.append(key)
        for name in _TIMES | {"fx", "fy", "cx", "cy"}:
            value = _number(view[name])
            if name in ("fx", "fy", "inverse_exposure_time") and value <= 0:
                raise ValueError("Focal length and exposure must be positive")
        camera_body = _pose(view["camera_from_body"])
        camera_local = _pose(view["camera_from_submap"])
        if not np.allclose(camera_local, camera_body @ np.linalg.inv(body_poses[key]),
                           atol=1e-8, rtol=0):
            raise ValueError("Camera/body relation disagrees with the local pose")
        for name, dtype, shape in (("rgb", "<f4", (height, width, 3)),
                                   ("image_mask", "|u1", (height, width)),
                                   ("depth_mask", "|u1", (height, width)),
                                   ("depth_m", "<f4", (height, width))):
            specs[f"v{index}_{name}"] = (np.dtype(dtype), shape)
    if keys != sorted(set(keys)):
        raise ValueError("Retained view IDs must be ordered and unique")
    return specs, keys


def _array(value, dtype, shape):
    if (not isinstance(value, np.ndarray) or value.dtype != np.dtype(dtype) or
            value.shape != shape or not value.flags.c_contiguous):
        raise ValueError("Unexpected array dtype, shape or layout")
    if value.dtype.kind == "f" and not np.isfinite(value).all():
        raise ValueError("State arrays must be finite")


def _describe(state):
    body_poses = _metadata(state.metadata)
    if not isinstance(state.views, list):
        raise ValueError("Expected a list of retained views")
    descriptions = [_view_description(view) for view in state.views]
    _, keys = _view_specs(descriptions, body_poses)
    _keys(state.parameters, set(_PARAMETERS))
    n = _integer(int(state.origin_ids.size), 1, 10_000)
    _array(state.origin_ids, "<u8", (n,))
    for name, columns in _PARAMETERS.items():
        _array(state.parameters[name], "<f4", (n, columns) if columns else (n,))
    counts = dict(zip(*np.unique(state.origin_ids, return_counts=True)))
    if set(counts) != set(keys) or any(count > 1000 for count in counts.values()):
        raise ValueError("Gaussian origins disagree with retained views")
    if not np.allclose(np.linalg.norm(state.parameters["quats"], axis=1), 1, atol=1e-5, rtol=0):
        raise ValueError("Gaussian quaternions must be normalized")
    scales = state.parameters["log_scales"]
    if np.any(scales < math.log(.001) - 1e-6) or np.any(scales > math.log(.2) + 1e-6):
        raise ValueError("Gaussian scale is outside the model bounds")
    observations = {}
    for index, view in enumerate(state.views):
        height, width = view["rgb"].shape[:2]
        for name in ("rgb", "depth_m"):
            _array(view[name], "<f4", (height, width, 3) if name == "rgb" else (height, width))
        if np.any(view["rgb"] < 0) or np.any(view["rgb"] > 1):
            raise ValueError("RGB is outside [0, 1]")
        for name in ("image_mask", "depth_mask"):
            mask = view[name]
            if (not isinstance(mask, np.ndarray) or mask.dtype not in (np.dtype(bool), np.dtype("u1")) or
                    mask.shape != (height, width) or not mask.flags.c_contiguous or np.any(mask > 1)):
                raise ValueError("Invalid observation mask")
        image_mask = view["image_mask"].astype(bool, copy=False)
        depth_mask = view["depth_mask"].astype(bool, copy=False)
        depth = view["depth_m"]
        if (not depth_mask.any() or np.any(depth_mask & ~image_mask) or
                np.any(depth[depth_mask] <= 0) or np.any(depth[~depth_mask] != 0)):
            raise ValueError("Depth and masks disagree")
        for name in ("rgb", "image_mask", "depth_mask", "depth_m"):
            value = view[name]
            observations[f"v{index}_{name}"] = (value.astype("u1", copy=False)
                                                if name.endswith("mask") else value)
    manifest = copy.deepcopy(state.metadata)
    manifest.update(views=descriptions, gaussian_count=n)
    # Check serializability and metadata size before creating any output.
    _json_bytes(manifest)
    return manifest, dict(state.parameters, origin_ids=state.origin_ids), observations


def validate_state(state: GaussianState) -> None:
    """Validate owned CPU data without accessing files or GPU libraries."""
    _describe(state)


def _json_bytes(value):
    data = json.dumps(value, allow_nan=False, ensure_ascii=False,
                      sort_keys=True, separators=(",", ":")).encode("utf-8")
    if len(data) > _LIMITS["manifest.json"]:
        raise ValueError("State manifest exceeds its budget")
    return data


def _digest(stream):
    stream.seek(0)
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(block)
    stream.seek(0)
    return digest.hexdigest()


def _write_npz(path, arrays):
    with path.open("xb") as stream:
        np.savez(stream, **arrays)
        stream.flush()
        os.fsync(stream.fileno())


def _sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def write_state(path: Path, state: GaussianState) -> None:
    """Publish a new asset; a post-commit sync failure is explicitly uncertain."""
    manifest, parameters, observations = _describe(state)
    path = Path(path)
    path.mkdir()
    manifest["files"] = {}
    for name, arrays in (("gaussians.npz", parameters), ("observations.npz", observations)):
        file = path / name
        _write_npz(file, arrays)
        size = file.stat().st_size
        if size > _LIMITS[name]:
            raise ValueError("State array file exceeds its budget")
        with file.open("rb") as stream:
            manifest["files"][name] = {"bytes": size, "sha256": _digest(stream)}
    pending = path / "manifest.json.pending"
    with pending.open("xb") as stream:
        stream.write(_json_bytes(manifest))
        stream.flush()
        os.fsync(stream.fileno())
    os.rename(pending, path / "manifest.json")
    try:
        _sync_directory(path)
        _sync_directory(path.parent)
    except OSError as error:
        # Once visible, an asset may already be in use. Never roll it back.
        raise CommitUncertainError("State is visible; directory sync failed: " + str(path)) from error


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate manifest field")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError("Nonfinite JSON value: " + value)


def _open_file(path, maximum):
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or not 1 <= info.st_size <= maximum:
            raise ValueError("State file is not a bounded regular file")
        return os.fdopen(fd, "rb")
    except BaseException:
        os.close(fd)
        raise


def _inspect_npz(stream, specs):
    # Bound the directory before ZipFile constructs one ZipInfo per entry.
    size = os.fstat(stream.fileno()).st_size
    if size < 22:
        raise ValueError("Truncated ZIP directory")
    stream.seek(-22, os.SEEK_END)
    signature, disk, start_disk, count, total, directory_size, offset, comment = struct.unpack(
        "<4s4H2IH", stream.read(22))
    if (signature != b"PK\x05\x06" or disk or start_disk or comment or
            count != len(specs) or total != count or directory_size > 64 * 1024 or
            offset + directory_size != size - 22):
        raise ValueError("Invalid or oversized ZIP directory")
    stream.seek(0)
    with zipfile.ZipFile(stream) as archive:
        entries = archive.infolist()
        if len(entries) != len(specs) or {entry.filename for entry in entries} != {
                name + ".npy" for name in specs}:
            raise ValueError("Unexpected or duplicate NPZ members")
        for entry in entries:
            if entry.compress_type != zipfile.ZIP_STORED or entry.flag_bits & 1:
                raise ValueError("State NPZ members must be uncompressed and unencrypted")
            dtype, shape = specs[entry.filename[:-4]]
            with archive.open(entry) as member:
                magic = member.read(8)
                if magic not in (b"\x93NUMPY\x01\x00", b"\x93NUMPY\x02\x00"):
                    raise ValueError("Unsupported NPY version")
                length_size = 2 if magic[-2] == 1 else 4
                length_bytes = member.read(length_size)
                if len(length_bytes) != length_size:
                    raise ValueError("Truncated NPY header")
                length = int.from_bytes(length_bytes, "little")
                if not 1 <= length <= 4096:
                    raise ValueError("NPY header exceeds its budget")
                header = member.read(length)
                if len(header) != length:
                    raise ValueError("Truncated NPY header")
                reader = (np.lib.format.read_array_header_1_0 if length_size == 2 else
                          np.lib.format.read_array_header_2_0)
                actual_shape, fortran, actual_dtype = reader(
                    io.BytesIO(length_bytes + header), max_header_size=4096)
                if fortran or actual_dtype != dtype or actual_shape != shape:
                    raise ValueError("NPY dtype or shape disagrees with manifest")
                if entry.file_size != 8 + length_size + length + math.prod(shape) * dtype.itemsize:
                    raise ValueError("NPY payload length disagrees with its header")
    stream.seek(0)


def read_state(path: Path) -> GaussianState:
    """Load owned arrays only after checking all bounded headers and digests."""
    path = Path(path)
    if path.is_symlink() or not path.is_dir() or {p.name for p in path.iterdir()} != set(_LIMITS):
        raise ValueError("State directory is incomplete or contains unknown files")
    with _open_file(path / "manifest.json", _LIMITS["manifest.json"]) as stream:
        manifest = json.loads(stream.read(), object_pairs_hook=_unique_object,
                              parse_constant=_reject_constant)
    _keys(manifest, _METADATA | {"files", "views", "gaussian_count"}, _HASHES)
    metadata = {key: value for key, value in manifest.items()
                if key not in ("files", "views", "gaussian_count")}
    body_poses = _metadata(metadata)
    n = _integer(manifest["gaussian_count"], 1, 10_000)
    observation_specs, _ = _view_specs(manifest["views"], body_poses)
    parameter_specs = {name: (np.dtype("<f4"), (n, columns) if columns else (n,))
                       for name, columns in _PARAMETERS.items()}
    parameter_specs["origin_ids"] = (np.dtype("<u8"), (n,))
    _keys(manifest["files"], {"gaussians.npz", "observations.npz"})
    with ExitStack() as stack:
        streams = {}
        # Validate every member header before NumPy can allocate any payload.
        for name, specs in (("gaussians.npz", parameter_specs),
                            ("observations.npz", observation_specs)):
            entry = manifest["files"][name]
            _keys(entry, {"bytes", "sha256"})
            size = _integer(entry["bytes"], 1, _LIMITS[name])
            if not isinstance(entry["sha256"], str) or not re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]):
                raise ValueError("Invalid array file digest")
            stream = stack.enter_context(_open_file(path / name, _LIMITS[name]))
            if os.fstat(stream.fileno()).st_size != size or _digest(stream) != entry["sha256"]:
                raise ValueError("Array file size or digest mismatch")
            _inspect_npz(stream, specs)
            streams[name] = stream
        loaded = []
        for name in ("gaussians.npz", "observations.npz"):
            with np.load(streams[name], allow_pickle=False, max_header_size=4096) as arrays:
                loaded.append({key: arrays[key].copy(order="C") for key in arrays.files})
    parameters, observations = loaded
    views = []
    for index, description in enumerate(manifest["views"]):
        view = {name: description[name] for name in _TIMES}
        view.update(keyframe_id=_unsigned(description["keyframe_id"]), calibration_id=0,
                    K=np.array([[description["fx"], 0, description["cx"]],
                                [0, description["fy"], description["cy"]], [0, 0, 1]], dtype="<f8"),
                    camera_from_body=_pose(description["camera_from_body"]),
                    camera_from_parent=_pose(description["camera_from_submap"]))
        for name in ("rgb", "image_mask", "depth_mask", "depth_m"):
            view[name] = observations[f"v{index}_{name}"]
        views.append(view)
    origin_ids = parameters.pop("origin_ids")
    state = GaussianState(metadata, parameters, origin_ids, views)
    validate_state(state)
    # Preserve raw uint8 mask values until their 0/1 domain has been checked.
    for view in views:
        for name in ("image_mask", "depth_mask"):
            view[name] = view[name].astype(bool)
    return state
