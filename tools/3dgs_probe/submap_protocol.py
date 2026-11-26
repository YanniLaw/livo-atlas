"""Bounded schema 2 payloads; schema 1 framing stays in online_protocol."""

from pathlib import PurePosixPath
import re

import numpy as np

from online_protocol import (_MAX_BODY, _MAX_PIXELS, _SESSION, _dimensions,
                             _integer, _number, _pose, _unsigned,
                             _validate_message, _validate_view)


_BASE_FIELDS = {"schema", "type", "session_id"}
_JOB_FIELDS = {"job_id", "generation", "pose_version"}
_TARGET_FIELDS = {"submap_id", "membership_revision", "frame_count", "optimized_frame_count",
                  "anchor_keyframe_id", "primary_count", "submap_state", "parent_frame",
                  "parent_from_submap"}
_VIEW_FIELDS_V2 = {"keyframe_id", "calibration_id", "state_timestamp", "cloud_timestamp",
                   "image_timestamp", "inverse_exposure_time", "fx", "fy", "cx", "cy",
                   "width", "height", "camera_from_submap", "camera_from_body"}
_MEMORY_FIELDS = {"gpu_peak_allocated_bytes", "gpu_peak_reserved_bytes", "worker_peak_rss_bytes"}
_CONTENT_FIELDS = {"source_generation", "content_job_id"}


def _fields(header, required, optional=frozenset()):
    if not required <= header.keys() <= required | optional:
        raise ValueError("Missing or unknown schema 2 fields")


def _same_pose(left, right):
    """Match the C++ pose_math translation-norm and rotation-angle comparison."""
    rotation = left[:3, :3].T @ right[:3, :3]
    skew = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0],
                     rotation[1, 0] - rotation[0, 1]])
    angle = np.arctan2(0.5 * np.linalg.norm(skew), 0.5 * (np.trace(rotation) - 1))
    return np.linalg.norm(left[:3, 3] - right[:3, 3]) <= 1e-9 and abs(angle) <= 1e-9


def _primary_context(header, body, offset):
    count = _integer(header.get("primary_count"), "primary count", 1, 512)
    anchor = _unsigned(header.get("anchor_keyframe_id"), "anchor ID")
    frontier = _unsigned(header.get("frame_count"), "frame count")
    optimized = _unsigned(header.get("optimized_frame_count"), "optimized count")
    version = _unsigned(header.get("pose_version"), "pose version")
    sealed = header.get("submap_state") == "sealed"
    if (header.get("submap_state") not in ("active", "sealed") or
            not anchor < frontier <= 512 or count > frontier - anchor or optimized > frontier or
            (version == 0 and optimized != 0) or (version != 0 and optimized == 0) or
            header.get("parent_frame") != ("map" if version else "odom") or
            _unsigned(header.get("membership_revision"), "membership revision") != count + int(sealed)):
        raise ValueError("Invalid primary membership or pose frontier")
    if type(offset) is not int or offset < 0 or len(body) != offset + 128 * count:
        raise ValueError("Incorrect primary body size")
    parent = _pose(header.get("parent_from_submap"), "submap pose")
    raw = np.frombuffer(body, dtype="<f8", count=16 * count, offset=offset).reshape(count, 16)
    poses = [_pose(values.tolist(), "primary body pose") for values in raw]
    if (not _same_pose(poses[0], np.eye(4)) or
            np.max(np.abs(poses[0][:3, :3] - np.eye(3))) > 1e-9):
        raise ValueError("The first primary body must define the origin")
    return {
        "capture_kind": "online_confirmed", "origin_session_id": header["session_id"],
        **{key: header[key] for key in ("submap_id", "generation", "pose_version", "frame_count",
           "optimized_frame_count", "membership_revision", "anchor_keyframe_id", "submap_state",
           "parent_frame")},
        "parent_from_submap": parent.ravel().tolist(),
        "primary": [{"keyframe_id": str(anchor + i), "submap_from_body": pose.ravel().tolist()}
                    for i, pose in enumerate(poses)],
    }


def decode_target(header, body, offset=0):
    """Decode an owned capture context; no generated fields or source-camera guesses."""
    _validate_message(header, body)
    if header["schema"] != 2 or header["type"] not in ("activate", "view"):
        raise ValueError("Expected a schema 2 target")
    return _primary_context(header, body, offset)


def _frame_ids(header):
    raw = header.get("frame_ids")
    if not isinstance(raw, list) or not 1 <= len(raw) <= 10:
        raise ValueError("Invalid retained frame IDs")
    ids = [_unsigned(value, "frame ID") for value in raw]
    if ids != sorted(set(ids)) or ids[-1] >= 512:
        raise ValueError("Retained frame IDs must be covered, ordered and unique")
    count = _integer(header.get("gaussian_count"), "Gaussian count", len(ids), 10_000)
    if count > 1000 * len(ids):
        raise ValueError("Gaussian count exceeds its source budget")
    return ids


def validate_message(header, body):
    from holdout_split import CAPABILITY, MAX_SPLIT_BYTES

    if header.get('type') in ('joint_render', 'joint_render_ok', 'joint_render_error'):
        from joint_render_protocol import validate_render_message
        return validate_render_message(header, body)
    if 'capabilities' in header:
        if (header.get('type') not in ('hello', 'hello_ok') or
                header['capabilities'] not in (['joint_render_v1'],
                    ['joint_render_v1', 'joint_render_spatial_v1'], [CAPABILITY],
                    ['joint_render_v1', CAPABILITY],
                    ['joint_render_v1', 'joint_render_spatial_v1', CAPABILITY])):
            raise ValueError('Invalid requested render capabilities')
    holdout = CAPABILITY in header.get('capabilities', [])
    split_fields = {'split_id'} if holdout else set()
    if holdout and (not isinstance(header.get('split_id'), str) or
                    not re.fullmatch(r'[0-9a-f]{64}', header['split_id'])):
        raise ValueError('Invalid Gaussian split digest')
    session = header.get("session_id")
    if not isinstance(session, str) or not _SESSION.fullmatch(session):
        raise ValueError("Invalid session ID")
    if not isinstance(body, bytes) or len(body) > _MAX_BODY:
        raise ValueError("Invalid message body")
    kind = header.get("type")
    base = _BASE_FIELDS | _JOB_FIELDS
    if kind not in ("hello", "hello_ok", "error"):
        for name in _JOB_FIELDS:
            _unsigned(header.get(name), name)
    if kind == "hello":
        _fields(header, _BASE_FIELDS | {"state_directory"} | split_fields, {"capabilities"})
        path = header["state_directory"]
        if (not isinstance(path, str) or "\0" in path or not path.startswith("/") or
                str(PurePosixPath(path)) != path or PurePosixPath(path).name != "states" or
                ".." in PurePosixPath(path).parts):
            raise ValueError("Expected a normalized absolute states directory")
        if holdout:
            if not 0 < len(body) <= MAX_SPLIT_BYTES:
                raise ValueError('Split exceeds the hello body budget')
            return
    elif kind == "hello_ok":
        _fields(header, _BASE_FIELDS | {"mode", "state_schema", "python", "torch", "gsplat",
                                       "cuda", "device", "gpu_free_bytes", "gpu_total_bytes"} | split_fields, {"capabilities"})
        if header["mode"] != "persistent_submaps" or type(header["state_schema"]) is not int or header["state_schema"] != (2 if holdout else 1):
            raise ValueError("Worker does not support persistent submaps")
        for name in ("python", "torch", "gsplat", "cuda", "device"):
            if not isinstance(header[name], str) or not 1 <= len(header[name]) <= 128:
                raise ValueError("Invalid worker metadata")
        free = _unsigned(header["gpu_free_bytes"], "free GPU bytes")
        total = _unsigned(header["gpu_total_bytes"], "GPU total bytes")
        if not 0 <= free <= total or not total:
            raise ValueError("Invalid GPU memory counters")
    elif kind in ("activate", "view"):
        required = base | _TARGET_FIELDS | ({"action"} if kind == "activate" else _VIEW_FIELDS_V2)
        optional = _CONTENT_FIELDS | {"asset_id", "manifest_sha256"} if kind == "activate" else set()
        _fields(header, required, optional)
        _unsigned(header["submap_id"], "submap ID")
        offset = 0 if kind == "activate" else 9 * _dimensions(header)[2]
        context = _primary_context(header, body, offset)
        if kind == "activate":
            action = header["action"]
            if action not in ("empty", "reuse", "rebuild"):
                raise ValueError("Invalid activation action")
            if action == "empty":
                if optional & header.keys():
                    raise ValueError("Empty activation cannot reference old content")
            else:
                for name in _CONTENT_FIELDS:
                    _unsigned(header.get(name), name)
                if not _unsigned(header["content_job_id"], "content job"):
                    raise ValueError("Content job must be nonzero")
                if ("asset_id" in header) != ("manifest_sha256" in header):
                    raise ValueError("Asset ID and digest must be supplied together")
                if "asset_id" in header:
                    if not _unsigned(header["asset_id"], "asset ID"):
                        raise ValueError("Asset ID must be nonzero")
                    if not isinstance(header["manifest_sha256"], str) or not re.fullmatch(r"[0-9a-f]{64}", header["manifest_sha256"]):
                        raise ValueError("Invalid manifest digest")
        else:
            # Keep the v1 pixel validator, including mask/depth checks, unchanged.
            _validate_view(header, body[:offset])
            frame = _unsigned(header["keyframe_id"], "frame ID")
            anchor = int(context["anchor_keyframe_id"])
            if not anchor <= frame < anchor + len(context["primary"]):
                raise ValueError("View is outside primary membership")
            camera_body = _pose(header["camera_from_body"], "body camera pose")
            body_pose = np.array(context["primary"][frame-anchor]["submap_from_body"]).reshape(4, 4)
            if not np.allclose(_pose(header["camera_from_submap"], "local camera pose"),
                               camera_body @ np.linalg.inv(body_pose), atol=1e-8, rtol=0):
                raise ValueError("Independent camera/body relation disagrees")
        return
    elif kind == "activated":
        _fields(header, base | {"submap_id"})
        _unsigned(header["submap_id"], "submap ID")
    elif kind == "candidate":
        _fields(header, base | {"submap_id", "operation", "membership_revision", "frame_count",
                                "optimized_frame_count", "frame_ids", "width", "height", "steps",
                                "gaussian_count", "supervised_depth_pixels", "rgb_l1", "depth_l1_m",
                                "nonzero_gradient", "parameter_updated", "capture_time"},
                _MEMORY_FIELDS | {'render_metadata'})
        _unsigned(header["submap_id"], "submap ID")
        frames = _frame_ids(header)
        frontier = _unsigned(header["frame_count"], "frame count")
        optimized = _unsigned(header["optimized_frame_count"], "optimized count")
        revision = _unsigned(header["membership_revision"], "membership revision")
        if not frames[-1] < frontier <= 512 or optimized > frontier or not 1 <= revision <= 513:
            raise ValueError("Invalid candidate frontier")
        operation = header["operation"]
        if not isinstance(operation, str):
            raise ValueError("Invalid operation")
        steps = {"view": 20, "reuse": 0, "rebuild": 20 * len(frames)}.get(operation)
        if steps is None or type(header["steps"]) is not int or header["steps"] != steps:
            raise ValueError("Step count differs from the operation")
        _integer(header["supervised_depth_pixels"], "depth count", 0, steps * _MAX_PIXELS)
        for name in ("rgb_l1", "depth_l1_m"):
            if _number(header[name], name) < 0:
                raise ValueError("Losses must be nonnegative")
        _number(header["capture_time"], "capture time")
        for name in ("nonzero_gradient", "parameter_updated"):
            _integer(header[name], name, 0, 1)
        for name in _MEMORY_FIELDS & header.keys():
            _unsigned(header[name], name)
        if len(body) != 3 * _dimensions(header)[2]:
            raise ValueError("Incorrect candidate image size")
        if 'render_metadata' in header:
            from spatial_metadata import validate_metadata
            metadata = header['render_metadata']
            source = metadata.get('content') if isinstance(metadata, dict) else None
            if not isinstance(source, dict):
                raise ValueError('Missing spatial candidate content')
            expected = dict(session_id=session, submap_id=header['submap_id'],
                source_generation=source.get('source_generation') if operation == 'reuse' else header['generation'],
                content_job_id=source.get('content_job_id') if operation == 'reuse' else header['job_id'])
            validate_metadata(metadata, expected, frames)
            camera = metadata['camera']
            if any(camera[key] != header[key] for key in ('width', 'height')) or camera['capture_time'] != header['capture_time']:
                raise ValueError('Spatial query differs from candidate preview')
        return
    elif kind in ("decision", "commit_ok", "discard_ok"):
        required = base | {"submap_id", "operation"}
        if kind == "decision":
            required |= {"action"}
        if kind == "commit_ok":
            required |= _CONTENT_FIELDS | {"frame_ids", "gaussian_count"}
        _fields(header, required)
        _unsigned(header["submap_id"], "submap ID")
        if header["operation"] not in ("view", "reuse", "rebuild"):
            raise ValueError("Invalid operation")
        if kind == "decision" and header["action"] not in ("accept", "discard"):
            raise ValueError("Invalid decision")
        if kind == "commit_ok":
            _frame_ids(header)
            for name in _CONTENT_FIELDS:
                _unsigned(header[name], name)
    elif kind in ("checkpoint", "checkpoint_ok"):
        required = base | {"submap_id", "asset_id"} | _CONTENT_FIELDS
        if kind == "checkpoint_ok":
            required |= {"manifest_sha256", "manifest_bytes", "gaussian_bytes", "observation_bytes",
                         "frame_ids", "gaussian_count"}
        _fields(header, required)
        for name in ("submap_id", "asset_id", "source_generation", "content_job_id"):
            value = _unsigned(header[name], name)
            if name in ("asset_id", "content_job_id") and not value:
                raise ValueError("Asset/content IDs must be nonzero")
        if kind == "checkpoint_ok":
            _frame_ids(header)
            if not isinstance(header["manifest_sha256"], str) or not re.fullmatch(r"[0-9a-f]{64}", header["manifest_sha256"]):
                raise ValueError("Invalid manifest digest")
            for name, limit in (("manifest_bytes", 512 * 1024), ("gaussian_bytes", 1024 * 1024),
                                ("observation_bytes", 192 * 1024 * 1024)):
                if not 1 <= _unsigned(header[name], name) <= limit:
                    raise ValueError("Asset file exceeds budget")
    elif kind in ("map_committed", "map_committed_ok"):
        _fields(header, base | {"map_sequence", "asset_ids"})
        if not _unsigned(header["map_sequence"], "map sequence"):
            raise ValueError("Map sequence must be nonzero")
        ids = header["asset_ids"]
        if not isinstance(ids, list) or len(ids) > 32:
            raise ValueError("Too many map asset references")
        parsed = [_unsigned(value, "asset ID") for value in ids]
        if parsed != sorted(set(parsed)) or (parsed and parsed[0] == 0):
            raise ValueError("Asset references must be nonzero, ordered and unique")
    elif kind in ("prune", "prune_ok"):
        _fields(header, base | {"map_sequence", "asset_id"})
        if not _unsigned(header["map_sequence"], "map sequence") or not _unsigned(header["asset_id"], "asset ID"):
            raise ValueError("Map/asset IDs must be nonzero")
    elif kind in ("stop", "stopped"):
        _fields(header, base | {"map_sequence"}, {"incomplete"})
        _unsigned(header["map_sequence"], "map sequence")
        if "incomplete" in header:
            _integer(header["incomplete"], "incomplete flag", 0, 1)
    elif kind == "error":
        _fields(header, _BASE_FIELDS | {"code"}, _JOB_FIELDS | {"submap_id", "message"})
        if header["code"] not in ("gpu_unavailable", "extension_unavailable", "protocol_error",
                                   "worker_error", "storage_error", "commit_uncertain", "resource_limit"):
            raise ValueError("Invalid worker error")
        for name in (_JOB_FIELDS | {"submap_id"}) & header.keys():
            _unsigned(header[name], name)
        if "message" in header and (not isinstance(header["message"], str) or len(header["message"]) > 2048):
            raise ValueError("Invalid worker error detail")
    else:
        raise ValueError("Unsupported schema 2 message")
    if body:
        raise ValueError("Unexpected message body")
