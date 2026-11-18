"""CPU-only inspection of a bounded online Gaussian map and its durable assets."""

import copy
import hashlib
import json
import math
from pathlib import Path
import re
import stat

import numpy as np
import yaml

from gaussian_state import (_LIMITS, _open_file, _reject_constant,
                            _unique_object, _unsigned, read_state)
from online_protocol import _pose
from submap_protocol import _same_pose


_TARGET_IDS = ("generation", "submap_id", "pose_version", "membership_revision",
               "frame_count", "optimized_frame_count", "anchor_keyframe_id")
_CONTENT_IDS = ("submap_id", "source_generation", "content_job_id")


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _integer(value, low, high):
    _require(type(value) is int and low <= value <= high, "Invalid map integer")
    return value


def _flag(value):
    return _integer(value, 0, 1)


def _json(path):
    with _open_file(path, 512 * 1024) as stream:
        return json.loads(stream.read().decode("utf-8"), object_pairs_hook=_unique_object,
                          parse_constant=_reject_constant)


def _yaml(path):
    with _open_file(path, 512 * 1024) as stream:
        text = stream.read().decode("utf-8")
    if text.startswith("%YAML:1.0\n"):
        text = text.split("\n", 1)[1]
    return yaml.safe_load(text)


def _transform(value):
    result = np.eye(4)
    result[:3, :3] = np.asarray(value["rotation"], dtype=np.float64).reshape(3, 3)
    result[:3, 3] = np.asarray(value["translation"], dtype=np.float64)
    return _pose(result.ravel().tolist(), "recorded body pose")


def _project(pose):
    result = pose.copy()
    left, _, right = np.linalg.svd(pose[:3, :3])
    direction = np.eye(3)
    direction[2, 2] = -1 if np.linalg.det(left @ right) < 0 else 1
    result[:3, :3] = left @ direction @ right
    return result


def _latest_poses(keyframes, version, frame_count, optimized):
    raw = []
    for i in range(frame_count):
        data = _yaml(keyframes / "frames" / str(i) / "frame.yml")
        _require(_unsigned(data["id"]) == i, "Raw pose ID differs from its path")
        raw.append(_transform(data["odom_from_body"]))
    if not version:
        _require(optimized == 0, "Version zero claims optimized frames")
        return raw
    data = _yaml(keyframes / "online_submaps/poses" / (str(version) + ".yml"))
    count = _unsigned(data["frame_count"])
    _require(_unsigned(data["version"]) == version and
             _unsigned(data["base_version"]) + 1 == version and
             data["parent_frame"] == "map" and
             0 < _unsigned(data["optimized_frame_count"]) == optimized <= count <= frame_count and
             len(data["poses"]) == count and _unsigned(data["tail_anchor_keyframe_id"]) == optimized-1,
             "Stored absolute pose frontier differs from the map")
    result = []
    for i, row in enumerate(data["poses"]):
        _require(_unsigned(row["id"]) == i, "Absolute poses must be an ordered complete prefix")
        result.append(_transform(row["map_from_body"]))
    projected = data["tail_policy"] == "last_optimized_anchor_projected"
    _require(data["tail_policy"] in ("last_optimized_anchor", "last_optimized_anchor_projected"),
             "Unknown pose tail policy")
    anchor = optimized - 1
    raw_anchor = _project(raw[anchor]) if projected else raw[anchor]
    correction = result[anchor] @ np.linalg.inv(raw_anchor)
    result.extend(correction @ (_project(pose) if projected else pose) for pose in raw[count:])
    return result


def _target(value, primary):
    ids = {key: _unsigned(value[key]) for key in _TARGET_IDS}
    count = _integer(value["primary_count"], 1, 512)
    anchor, frontier = ids["anchor_keyframe_id"], ids["frame_count"]
    _require(anchor + count <= frontier <= 512 and ids["optimized_frame_count"] <= frontier and
             value["submap_state"] in ("active", "sealed") and
             ids["membership_revision"] == count + (value["submap_state"] == "sealed") and
             ((not ids["pose_version"] and value["parent_frame"] == "odom" and
               ids["optimized_frame_count"] == 0) or
              (ids["pose_version"] > 0 and value["parent_frame"] == "map" and
               ids["optimized_frame_count"] > 0)), "Invalid source or binding frontier")
    _pose(value["parent_from_submap"], "submap binding")
    if primary:
        _require(isinstance(value["primary"], list) and len(value["primary"]) == count,
                 "Source omits complete canonical primary membership")
        for i, row in enumerate(value["primary"]):
            _require(_unsigned(row["keyframe_id"]) == anchor + i, "Invalid canonical primary ID")
            pose = _pose(row["submap_from_body"], "canonical primary pose")
            if i == 0:
                _require(np.max(np.abs(pose - np.eye(4))) <= 1e-9 and
                         _same_pose(pose, np.eye(4)), "Source primary origin must be identity")
    return ids


def _content(value, session, submap):
    result = tuple(_unsigned(value[key]) for key in _CONTENT_IDS)
    _require(value["session_id"] == session and result[0] == submap and result[2] > 0,
             "Content belongs to a different session or submap")
    return result


def _coverage(ids, count, source):
    _require(isinstance(ids, list) and 1 <= len(ids) <= 10, "Invalid saved view budget")
    frames = [_unsigned(value) for value in ids]
    _require(frames == sorted(set(frames)) and
             _unsigned(source["anchor_keyframe_id"]) <= frames[0] and
             frames[-1] < _unsigned(source["anchor_keyframe_id"]) + source["primary_count"] and
             len(frames) <= _integer(count, 1, 10000) <= 1000 * len(frames),
             "Saved views or parameter count differ from source coverage")
    return frames


def _unchanged(source, poses):
    anchor = _unsigned(source["anchor_keyframe_id"])
    inverse = np.linalg.inv(poses[anchor])
    return all(_same_pose(np.asarray(row["submap_from_body"]).reshape(4, 4),
                          inverse @ poses[_unsigned(row["keyframe_id"])])
               for row in source["primary"])


def _digest(path):
    digest = hashlib.sha256()
    with _open_file(path, _LIMITS[path.name]) as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _check_groups(keyframes, rows):
    """Compare logical membership with the latest immutable grouping records."""
    records = keyframes / "online_submaps/records"
    if not records.exists():
        return
    groups = list(records.iterdir())
    _require(len(groups) == len(rows) and
             sorted(_unsigned(path.name) for path in groups) == list(range(len(rows))),
             "Map omits authoritative logical groups")
    for submap, row in enumerate(rows):
        revisions = list((records / str(submap)).iterdir())
        _require(0 < len(revisions) <= 513 and
                 all(path.suffix == ".yml" for path in revisions), "Invalid group revisions")
        revision = max(_unsigned(path.stem) for path in revisions)
        saved = _yaml(records / str(submap) / (str(revision) + ".yml"))
        first = _unsigned(row["anchor_keyframe_id"])
        _require(saved["schema_version"] == 1 and _unsigned(saved["id"]) == submap and
                 _unsigned(saved["membership_revision"]) == revision ==
                 _unsigned(row["membership_revision"]) and
                 saved["state"] == row["submap_state"] and
                 _unsigned(saved["anchor_keyframe_id"]) == first and
                 [_unsigned(value) for value in saved["keyframe_ids"]] ==
                 list(range(first, first + row["primary_count"])),
                 "Logical membership differs from authoritative grouping")


def _inspect_gaussian_map(directory, keyframes, expected_version, expected_frames,
                          query_id=None):
    """Verify one state at a time; never retain a map-wide parameter cache."""
    directory, keyframes = Path(directory), Path(keyframes)
    data = _json(directory / "map.json")
    _require(type(data["schema"]) is int and data["schema"] == 1 and data["kind"] == "livo_atlas.gaussian_map",
             "Unknown Gaussian map schema")
    rows = data["entries"]
    _require(isinstance(rows, list) and len(rows) <= 512 and
             sum("content" in row for row in rows) <= 32, "Gaussian content budget exceeded")
    states = directory / "states"
    _require(stat.S_ISDIR(states.lstat().st_mode), "State root must be a real directory")
    assets = list(states.iterdir())
    _require(len(assets) <= 33, "Gaussian asset directory budget exceeded")
    asset_sizes = {}
    for asset in assets:
        number = _unsigned(asset.name)
        _require(number > 0 and stat.S_ISDIR(asset.lstat().st_mode), "Invalid asset directory")
        paths = list(asset.iterdir())
        _require({path.name for path in paths} == set(_LIMITS), "Unexpected asset files")
        sizes = {}
        for path in paths:
            info = path.lstat()
            _require(stat.S_ISREG(info.st_mode) and 0 < info.st_size <= _LIMITS[path.name],
                     "Gaussian asset byte budget exceeded")
            sizes[path.name] = info.st_size
        asset_sizes[number] = sizes
    total = sum(sum(value.values()) for value in asset_sizes.values())
    _require(total <= 33 * sum(_LIMITS.values()), "Total Gaussian byte budget exceeded")
    status = _json(directory / "status.json")
    session = data["session_id"]
    _require(isinstance(session, str) and re.fullmatch(r"[0-9a-f]{32}", session) and
             session == status["session_id"], "Map session differs from the online session")
    version, frames, optimized = (_unsigned(data[key]) for key in
                                  ("pose_version", "frame_count", "optimized_frame_count"))
    _require(version == expected_version and frames == expected_frames <= 512 and
             ((version == 0 and data["parent_frame"] == "odom" and optimized == 0) or
              (version > 0 and data["parent_frame"] == "map" and 0 < optimized <= frames)) and
             _unsigned(data["map_sequence"]) > 0 and
             math.isfinite(data["written_at_unix_seconds"]), "Map differs from the final SLAM frontier")
    poses = _latest_poses(keyframes, version, frames, optimized)
    report = {"valid": True, "map_sequence": _unsigned(data["map_sequence"]),
              "pose_version": version, "frame_count": frames, "indexed": len(rows),
              "verified_assets": 0, "current": 0, "needs_rebuild": 0, "missing": 0,
              "volatile": 0, "asset_bytes": total, "referenced_bytes": 0,
              "asset_directories": len(assets)}
    referenced = set()
    query = None
    next_primary = residents = 0
    for submap, row in enumerate(rows):
        binding = row["binding"]
        target = _target(binding, False)
        count = _integer(row["primary_count"], 1, 512)
        _require(_unsigned(row["submap_id"]) == submap == target["submap_id"] and
                 _unsigned(row["anchor_keyframe_id"]) == next_primary == target["anchor_keyframe_id"] and
                 count == binding["primary_count"] and
                 row["membership_revision"] == binding["membership_revision"] and
                 row["submap_state"] == binding["submap_state"] and
                 target["pose_version"] == version and target["frame_count"] == frames and
                 target["optimized_frame_count"] == optimized and binding["parent_frame"] == data["parent_frame"] and
                 _same_pose(_pose(binding["parent_from_submap"], "binding"), poses[next_primary]),
                 "Logical group or binding differs from the latest absolute poses")
        next_primary += count
        residents += _flag(row["resident"])
        available = row["availability"]
        if "content" not in row:
            _require(available == "missing" and "asset" not in row and not row["resident"] and
                     not row.get("volatile", 0), "Missing group claims confirmed content")
            report["missing"] += 1
            continue
        content = _content(row["content"], session, submap)
        source = row["content_source"]
        original = _target(source, True)
        _require(original["submap_id"] == submap and original["generation"] == content[1] and
                 original["anchor_keyframe_id"] == target["anchor_keyframe_id"] and
                 source["primary_count"] <= count and original["frame_count"] <= frames and
                 original["pose_version"] <= version and
                 target["generation"] >= content[1] and _unsigned(row["binding_job_id"]) >= content[2],
                 "Confirmed source identity or frontier differs")
        _coverage(row["trained_frame_ids"], row["gaussian_count"], source)
        current = _unchanged(source, poses)
        _require(available == ("current" if current else "needs_rebuild"),
                 "Currentness differs from the complete canonical primary poses")
        report["current" if current else "needs_rebuild"] += 1
        dirty = _flag(row["volatile"])
        report["volatile"] += dirty
        if "asset" not in row:
            _require(dirty == 1, "Confirmation has no durable asset but claims persistence")
            continue
        description = row["asset"]
        asset_content = _content(description, session, submap)
        _require(dirty == int(asset_content != content), "Volatile content is confused with its asset")
        asset_id = _unsigned(description["asset_id"])
        _require(asset_id in asset_sizes and asset_id not in referenced, "Missing or repeated durable asset")
        referenced.add(asset_id)
        path = states / str(asset_id)
        sizes = asset_sizes[asset_id]
        for key, name in (("manifest_bytes", "manifest.json"), ("gaussian_bytes", "gaussians.npz"),
                          ("observation_bytes", "observations.npz")):
            _require(_unsigned(description[key]) == sizes[name], "Asset size differs from the checkpoint")
        _require(description["manifest_sha256"] == _digest(path / "manifest.json"),
                 "Asset manifest differs from its checkpoint digest")
        saved_source = row["asset_source"]
        saved = _target(saved_source, True)
        _require(saved["submap_id"] == submap and saved["generation"] == asset_content[1] and
                 saved["anchor_keyframe_id"] == target["anchor_keyframe_id"] and
                 saved_source["primary_count"] <= count and saved["frame_count"] <= frames and
                 saved["pose_version"] <= version and
                 _flag(row["asset_current"]) == int(_unchanged(saved_source, poses)),
                 "Asset source identity or currentness differs")
        ids = _coverage(row["asset_trained_frame_ids"], row["asset_gaussian_count"], saved_source)
        state = read_state(path)
        metadata = state.metadata
        _require(metadata["origin_session_id"] == session and metadata["capture_kind"] == "online_confirmed" and
                 all(metadata[key] == saved_source[key] for key in _TARGET_IDS) and
                 all(metadata[key] == saved_source[key] for key in ("parent_frame", "submap_state", "primary")) and
                 np.allclose(metadata["parent_from_submap"], saved_source["parent_from_submap"], atol=1e-12, rtol=0) and
                 [view["keyframe_id"] for view in state.views] == ids and
                 state.origin_ids.size == row["asset_gaussian_count"], "Stored state differs from the map source or coverage")
        if not dirty:
            _require(source == saved_source and row["trained_frame_ids"] == row["asset_trained_frame_ids"] and
                     row["gaussian_count"] == row["asset_gaussian_count"], "Durable and confirmed coverage differ")
        report["verified_assets"] += 1
        report["referenced_bytes"] += sum(sizes.values())
        if query_id is not None:
            for view in state.views:
                if view["keyframe_id"] == query_id:
                    _require(query is None, "Query frame belongs to multiple assets")
                    query = (submap, copy.deepcopy(view))
            del view
        del state
    _require(next_primary == frames and residents <= 1 and len(referenced) <= 32,
             "Logical frontier is incomplete or resident budget exceeded")
    _check_groups(keyframes, rows)
    report["unreferenced_assets"] = len(assets) - len(referenced)
    report["durable"] = report["volatile"] == 0 and report["unreferenced_assets"] == 0
    return report, data, poses, query


def inspect_gaussian_map(directory, keyframes, expected_version, expected_frames):
    return _inspect_gaussian_map(directory, keyframes, expected_version,
                                 expected_frames)[0]


def inspect_gaussian_finalization(directory, map_report):
    """Reconcile a final report with a previously verified map and asset set.

    This checks current files, not whether the publishing process fsynced them.
    Replay acceptance must also check that process's log and exit evidence.
    """
    directory = Path(directory)
    with _open_file(directory / 'finalization.json', 16 * 1024) as stream:
        result = json.loads(stream.read().decode('utf-8'), object_pairs_hook=_unique_object,
                            parse_constant=_reject_constant)
    data = _json(directory / 'map.json')
    _require(type(result['schema']) is int and result['schema'] == 1 and
             result['kind'] == 'livo_atlas.gaussian_finalization', 'Unknown finalization schema')
    _require(result['status'] == 'succeeded' and isinstance(result['reason'], str) and
             len(result['reason'].encode('utf-8')) <= 1024 and '\0' not in result['reason'],
             'Finalization did not succeed')
    _require(map_report['valid'] and map_report['durable'] and
             not map_report['needs_rebuild'] and not map_report['volatile'] and
             not map_report['unreferenced_assets'], 'Finalization map is not current and durable')
    for key in ('target_valid', 'eligible_set_frozen', 'durable', 'worker_stopped'):
        _require(_flag(result[key]) == 1, 'Finalization is incomplete: ' + key)
    _require(result['session_id'] == data['session_id'] and
             result['parent_frame'] == data['parent_frame'], 'Finalization session or parent differs')
    for key in ('pose_version', 'frame_count', 'optimized_frame_count', 'map_sequence'):
        _require(_unsigned(result[key]) == _unsigned(data[key]), 'Finalization frontier differs: ' + key)
    for key in ('current', 'needs_rebuild', 'missing'):
        _require(_integer(result[key], 0, 512) == map_report[key], 'Finalization count differs: ' + key)
    _require(_integer(result['dirty'], 0, 32) == map_report['volatile'], 'Finalization dirty count differs')
    rows = data['entries']
    _require(all(row['submap_state'] == 'sealed' for row in rows), 'Finalization groups are not sealed')
    eligible = [_unsigned(row['submap_id']) for row in rows if 'content' in row]
    assets = sorted(_unsigned(row['asset']['asset_id']) for row in rows if 'content' in row)
    for key, expected in (('eligible_submap_ids', eligible), ('asset_ids', assets)):
        values = result[key]
        _require(isinstance(values, list) and len(values) <= 32, 'Finalization ID budget exceeded')
        ids = [_unsigned(value) for value in values]
        _require(ids == expected and ids == sorted(set(ids)), 'Finalization identity set differs: ' + key)
    _require(_integer(result['eligible'], 0, 32) == len(eligible) == result['current'] ==
             map_report['verified_assets'], 'Finalization eligible set differs from confirmed content')
    for row in rows:
        if 'content' in row:
            _require(row['availability'] == 'current' and _flag(row['asset_current']) == 1 and
                     all(row['asset'][key] == row['content'][key] for key in
                         ('session_id', *_CONTENT_IDS)), 'Finalization has historical or dirty assets')
    counters = ('accepted', 'effective_training', 'rebuilds', 'checkpoints')
    before, after = result['before'], result['after']
    for key in counters:
        _require(_unsigned(before[key]) <= _unsigned(after[key]), 'Finalization counters went backwards')
    for counts in (before, after):
        _require(_unsigned(counts['effective_training']) <= _unsigned(counts['accepted']) and
                 _unsigned(counts['rebuilds']) <= _unsigned(counts['accepted']), 'Invalid training counters')
    _require(_unsigned(after['accepted']) >= len(eligible), 'Finalization has unconfirmed content')
    _unsigned(result['repair_attempts'])
    for key in ('elapsed_seconds', 'repair_seconds', 'shutdown_seconds'):
        value = result[key]
        _require(type(value) in (int, float) and math.isfinite(value) and value >= 0,
                 'Invalid finalization duration')
    _require(math.isclose(result['elapsed_seconds'], result['repair_seconds'] +
                         result['shutdown_seconds'], rel_tol=1e-8, abs_tol=1e-8),
             'Finalization stage durations differ')
    return result
