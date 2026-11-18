#!/usr/bin/env python3
"""Strictly compare paired Noetic replays and inspect real Gaussian previews."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys

import numpy as np
from PIL import Image
import yaml


sys.path.insert(0, str(Path(__file__).resolve().parent / "3dgs_probe"))
from gaussian_map import inspect_gaussian_finalization, inspect_gaussian_map  # noqa: E402


_ARRAYS = ("trajectory.tum", "mat_pre.txt", "mat_out.txt", "imu.txt")
_PARAMETER_GROUPS = ("common", "extrin_calib", "time_offset", "preprocess", "vio",
                     "imu", "lio", "local_map", "uav", "publish", "evo", "pcd_save",
                     "image_save", "debug", "laserMapping", "use_sim_time", "keyframes",
                     "submaps", "loop_closure", "global_output")


def _digest(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _raw_keyframes(directory):
    root = directory / "keyframes"
    if not (root / "dataset.yml").is_file() or not (root / "frames").is_dir():
        raise ValueError("Missing raw keyframe dataset in " + str(directory))
    paths = [root / "dataset.yml"] + sorted((root / "frames").rglob("*"))
    return {str(path.relative_to(root)): _digest(path) for path in paths if path.is_file()}


def _parameters(directory):
    tree = yaml.safe_load((directory / "parameters.yaml").read_text()) or {}
    result = {key: tree[key] for key in _PARAMETER_GROUPS if key in tree}
    if isinstance(result.get("evo"), dict):
        result["evo"] = dict(result["evo"])
        result["evo"].pop("seq_name", None)
    return result


def _stats(directory, marker):
    lines = (directory / "mapping.log").read_text(errors="replace").splitlines()
    matches = [line for line in lines if marker in line]
    if not matches:
        return {}
    return {key: float(value) if any(c in value for c in ".eE") else int(value)
            for key, value in re.findall(r"\b([a-z_][a-z0-9_]*)=([-+0-9.eE]+)", matches[-1])}


def _finalization(directory, previews, map_report, summary, stats, persistent):
    report = inspect_gaussian_finalization(previews, map_report)
    lines = (directory / 'mapping.log').read_text(errors='replace').splitlines()
    lines = [line for line in lines if 'livo_atlas gaussian finalization:' in line]
    if len(lines) != 1:
        raise ValueError('Missing or duplicate finalization shutdown log')
    fields = dict(re.findall(r'\b([a-z_][a-z0-9_]*)=([^\s]+)', lines[0]))
    for key in ('status', 'session_id', 'pose_version', 'frame_count', 'optimized_frame_count',
                'parent_frame', 'map_sequence', 'target_valid', 'eligible_set_frozen',
                'eligible', 'current', 'needs_rebuild', 'missing', 'dirty', 'durable',
                'worker_stopped', 'repair_attempts'):
        if fields.get(key) != str(report[key]):
            raise ValueError('Finalization log differs from report: ' + key)
    for key in ('eligible_submap_ids', 'asset_ids'):
        if fields.get(key) != (','.join(report[key]) or '-'):
            raise ValueError('Finalization log identities differ: ' + key)
    for stage in ('before', 'after'):
        for key, value in report[stage].items():
            if fields.get(stage + '_' + key) != value:
                raise ValueError('Finalization log counters differ')
    for key in ('elapsed_seconds', 'repair_seconds', 'shutdown_seconds'):
        value = float(fields[key])
        if not math.isfinite(value) or not math.isclose(value, report[key], rel_tol=1e-8, abs_tol=1e-8):
            raise ValueError('Finalization log durations differ')
    total = float(fields['total_seconds'])
    if (not math.isfinite(total) or total < report['elapsed_seconds'] or
            fields.get('report_persisted') != '1' or
            summary.get('forced_termination') is not False or
            summary.get('node_returncode_after_sigint') != 0):
        raise ValueError('Finalization publication or process exit was not successful')
    if (int(report['after']['accepted']) != stats['accepted'] or
            any(int(report['after'][key]) != persistent.get(key) for key in
                ('effective_training', 'rebuilds', 'checkpoints'))):
        raise ValueError('Finalization counters differ from shutdown statistics')
    return report


def _previews(directory):
    images = {path.stem: path for path in directory.glob("preview-*.png")}
    metadata = {path.stem: path for path in directory.glob("preview-*.json")}
    result = {"pairs": len(metadata), "readable": True, "trained": 0, "reused": 0, "rebuilt": 0,
              "gpu_peak_allocated_bytes": 0, "gpu_peak_reserved_bytes": 0,
              "worker_peak_rss_bytes": 0, "latencies_seconds": []}
    if not 1 <= len(metadata) <= 8 or images.keys() != metadata.keys():
        result["readable"] = False
        return result
    try:
        session = json.loads((directory / "status.json").read_text())["session_id"]
        if not isinstance(session, str) or not re.fullmatch(r"[0-9a-f]{32}", session):
            raise ValueError("Invalid preview session")
    except (OSError, ValueError, TypeError, KeyError):
        result["readable"] = False
        return result
    for stem, path in metadata.items():
        try:
            data = json.loads(path.read_text())
            ids = ("job_id", "generation", "submap_id", "pose_version")
            if (data.get("schema") != 1 or data.get("type") != "preview" or
                    data.get("session_id") != session or
                    any(not isinstance(data.get(key), str) or
                        not re.fullmatch(r"0|[1-9][0-9]*", data[key]) for key in ids) or
                    stem != "preview-" + data["job_id"] or data["job_id"] == "0"):
                raise ValueError("Invalid preview identity")
            with Image.open(images[stem]) as image:
                if (image.format != "PNG" or image.mode != "RGB" or
                        image.size != (data["width"], data["height"])):
                    raise ValueError("Preview must be an RGB PNG")
                image.verify()
            values = (float(data["rgb_l1"]), float(data["depth_l1_m"]))
            if not all(math.isfinite(value) and value >= 0 for value in values):
                raise ValueError("Invalid preview losses")
            operation = data.get("operation", "view")
            if operation not in ("view", "reuse", "rebuild"):
                raise ValueError("Unknown Gaussian preview operation")
            expected_steps = (0 if operation == "reuse" else 20 if operation == "view"
                              else 20 * len(data["frame_ids"]))
            if data.get("steps", 20) != expected_steps:
                raise ValueError("Preview steps differ from its operation")
            if operation == "reuse":
                result["reused"] += 1
                if any(data.get(key, 0) for key in ("nonzero_gradient", "parameter_updated", "supervised_depth_pixels")):
                    raise ValueError("Zero-step reuse claims training")
            if operation == "rebuild":
                result["rebuilt"] += 1
            if (operation != "reuse" and data.get("nonzero_gradient") == 1 and data.get("parameter_updated") == 1 and
                    data.get("supervised_depth_pixels", 0) > 0):
                result["trained"] += 1
            for key in ("gpu_peak_allocated_bytes", "gpu_peak_reserved_bytes",
                        "worker_peak_rss_bytes"):
                result[key] = max(result[key], int(data.get(key, "0")))
            if "latency_seconds" in data:
                latency = float(data["latency_seconds"])
                if not math.isfinite(latency) or latency < 0:
                    raise ValueError("Invalid preview latency")
                result["latencies_seconds"].append(latency)
        except (OSError, ValueError, TypeError, KeyError):
            result["readable"] = False
    return result


def compare(off, on, previews, joint_render=False, min_joint_dual=0, min_joint_groups=0,
            publication=False):
    """Return every observed difference; no numeric tolerance is applied."""
    off, on, previews = map(Path, (off, on, previews))
    summaries = [json.loads((run / "summary.json").read_text()) for run in (off, on)]
    differences = {}
    if off.resolve() == on.resolve():
        differences["run_directories"] = "OFF and ON must be different runs"
    configs = []
    for run in (off, on):
        tree = yaml.safe_load((run / "parameters.yaml").read_text()) or {}
        config = tree.get("gaussian", {})
        configs.append(config if isinstance(config, dict) else {})
    if joint_render or publication:
        enabled = [config.get('joint_render_enabled', False) for config in configs]
        if (any(config.get('enabled') is not True or config.get('persist_submaps') is not True or
                config.get('finalize_on_shutdown') is not True for config in configs) or
                enabled != ([True, True] if publication else [False, True]) or
                (publication and [config.get('publish_preview', False) for config in configs] != [False, True])):
            differences['gaussian_configuration'] = 'Joint comparison requires persistent training and Finalize; publication comparison keeps both renderers ON'
        ignored = {'socket_path', 'output_directory'}
        ignored.update({'publish_preview', 'preview_frame_id'} if publication else
                       {'joint_render_enabled', 'joint_render_interval_ms',
                        'joint_render_timeout_ms', 'joint_render_policy'})
        normalized = [{key: value for key, value in config.items() if key not in ignored}
                      for config in configs]
        if normalized[0] != normalized[1]:
            differences['gaussian_parameters'] = 'Non-render Gaussian settings differ'
    elif configs[0].get("enabled", False) is not False or configs[1].get("enabled") is not True:
        differences["gaussian_configuration"] = "OFF must disable Gaussian and ON must enable it"
    output_directory = configs[1].get("output_directory")
    if (not isinstance(output_directory, str) or not output_directory or
            Path(output_directory).expanduser().resolve() != previews.resolve()):
        differences["preview_directory"] = "Previews do not match the ON output directory"
    for index, summary in enumerate(summaries):
        expected = summary.get("expected_input_counts", {})
        observed = summary.get("observed_counts", {})
        if (not summary.get("clean_shutdown") or summary.get("input_count_mismatches") or
                not expected or any(observed.get(topic, 0) != count
                                    for topic, count in expected.items())):
            differences["input_or_shutdown_" + ("off" if index == 0 else "on")] = True
    for key in ("expected_input_counts", "lio_cycles", "vio_cycles", "mapping_cpu_affinity",
                "playback_rate", "image_queue_size", "binary_sha256"):
        if key not in summaries[0] or summaries[0].get(key) != summaries[1].get(key):
            differences[key] = [summaries[0].get(key), summaries[1].get(key)]
    sources = [summary.get("source", {}).get("file_sha256", {}) for summary in summaries]
    if not sources[0] or sources[0] != sources[1]:
        differences["source_snapshot"] = "Recorded source file hashes differ or are missing"
    if _parameters(off) != _parameters(on):
        differences["parameters"] = "Non-Gaussian configuration differs"
    semantic_paths = [run / "semantic-output.json" for run in (off, on)]
    if (not all(path.is_file() for path in semantic_paths) or
            json.loads(semantic_paths[0].read_text()) !=
            json.loads(semantic_paths[1].read_text())):
        differences["published_semantics"] = "Image or point-cloud output differs or is missing"
    arrays = {}
    for name in _ARRAYS:
        before, after = (np.loadtxt(run / name, ndmin=2) for run in (off, on))
        equal = (before.shape == after.shape and np.isfinite(before).all() and
                 np.isfinite(after).all() and np.array_equal(before, after))
        arrays[name] = {"off_shape": list(before.shape), "on_shape": list(after.shape),
                        "exact": bool(equal)}
        if not equal:
            differences[name] = arrays[name]
    raw_off, raw_on = _raw_keyframes(off), _raw_keyframes(on)
    if not raw_off or raw_off != raw_on:
        differences["raw_keyframes"] = {"off_files": len(raw_off), "on_files": len(raw_on),
                                        "different": sorted(set(raw_off) ^ set(raw_on) |
                                            {name for name in raw_off.keys() & raw_on.keys()
                                             if raw_off[name] != raw_on[name]})}
    loops = [_stats(run, "livo_atlas loops:") for run in (off, on)]
    for key in ("frames", "pose_version", "accepted"):
        if key not in loops[0] or loops[0].get(key) != loops[1].get(key):
            differences["loop_backend"] = loops
            break
    stats = _stats(on, "livo_atlas gaussian:")
    required = ("observed", "submitted", "completed", "accepted", "failed",
                "peak_pending", "peak_in_flight")
    if (any(key not in stats for key in required) or stats.get("accepted", 0) < 1 or
            stats.get("failed") != 0 or stats.get("peak_pending", 2) > 1 or
            stats.get("peak_in_flight", 2) > 1):
        differences["gaussian_stats"] = stats
    first = summaries[1].get("first_gaussian_preview_during_playback_s")
    if (not isinstance(first, (int, float)) or not math.isfinite(first) or first < 0 or
            first > summaries[1].get("playback_wall_s", float("inf"))):
        differences["online_preview"] = "No complete preview was observed during playback"
    preview_report = _previews(previews)
    persistence = configs[1].get("persist_submaps", False)
    if type(persistence) is not bool:
        differences["gaussian_configuration"] = "Persistence must be a boolean"
    only_reuse = preview_report["pairs"] == preview_report["reused"]
    if not preview_report["readable"] or (not preview_report["trained"] and
            not (persistence and only_reuse)):
        differences["preview_files"] = preview_report
    map_report = None
    finalization_report = None
    confirmed_training = stats.get("accepted", 0)
    if persistence:
        try:
            map_report = inspect_gaussian_map(previews, on / "keyframes",
                loops[1]["pose_version"], loops[1]["frames"])
            persistent = _stats(on, "livo_atlas gaussian submaps:")
            rebinds = persistent.get("rebinds", -1)
            confirmed_training = (stats.get("accepted", 0) - rebinds
                                  if type(rebinds) is int and rebinds >= 0 else 0)
            effective_training = persistent.get("effective_training", 0)
            if (type(effective_training) is not int or
                    not 0 < effective_training <= confirmed_training or
                    effective_training < preview_report["trained"] or
                    not map_report["durable"] or not map_report["verified_assets"] or
                    persistent.get("incomplete", 1) != 0 or
                    any(persistent.get(key) != map_report[key] for key in
                        ("map_sequence", "indexed", "missing", "needs_rebuild")) or
                    persistent.get("dirty") != map_report["volatile"] or
                    persistent.get("asset_bytes") != map_report["referenced_bytes"]):
                differences["gaussian_map"] = "Map is incomplete or differs from shutdown statistics"
        except (OSError, ValueError, TypeError, KeyError) as error:
            map_report = {"valid": False, "error": str(error)}
            differences["gaussian_map"] = str(error)
    finalize = configs[1].get('finalize_on_shutdown', False)
    if type(finalize) is not bool or (finalize and not persistence):
        differences['gaussian_configuration'] = 'Finalization requires boolean persistence'
    if finalize:
        try:
            finalization_report = _finalization(on, previews, map_report, summaries[1],
                                                stats, _stats(on, 'livo_atlas gaussian submaps:'))
        except (OSError, ValueError, TypeError, KeyError) as error:
            differences['gaussian_finalization'] = str(error)
    joint_report = None
    if joint_render or publication:
        try:
            try:
                from .joint_render_report import inspect_joint_renders
            except ImportError:
                from joint_render_report import inspect_joint_renders
            joint_report = inspect_joint_renders(on, previews, min_joint_dual, min_joint_groups)
            if joint_report['policy'] != configs[1].get('joint_render_policy', 'temporal'):
                raise ValueError('Reported render policy differs from parameters.yaml')
        except (OSError, ValueError, TypeError, KeyError) as error:
            differences['joint_render'] = str(error)
        try:
            off_previews = Path(configs[0]['output_directory'])
            if publication:
                inspect_joint_renders(off, off_previews, min_joint_dual, min_joint_groups)
            elif list(off_previews.glob('joint-render-*')) or 'livo_atlas gaussian joint accepted:' in (off / 'mapping.log').read_text():
                raise ValueError('Disabled replay published a joint render')
            off_map = inspect_gaussian_map(off_previews, off / 'keyframes', loops[0]['pose_version'], loops[0]['frames'])
            if not off_map['durable']:
                raise ValueError('Disabled replay did not persist its training')
            if configs[0].get('finalize_on_shutdown'):
                _finalization(off, off_previews, off_map, summaries[0],
                              _stats(off, 'livo_atlas gaussian:'), _stats(off, 'livo_atlas gaussian submaps:'))
        except (OSError, ValueError, TypeError, KeyError) as error:
            differences['off_gaussian_map'] = str(error)
    publication_report = None
    if publication:
        try:
            try:
                from .gaussian_preview_report import inspect_gaussian_publication
            except ImportError:
                from gaussian_preview_report import inspect_gaussian_publication
            off_publication = inspect_gaussian_publication(off, Path(configs[0]['output_directory']))
            publication_report = inspect_gaussian_publication(on, previews)
            if off_publication['enabled'] or not publication_report['enabled']:
                raise ValueError('Receiver evidence does not match publication OFF/ON')
        except (OSError, ValueError, TypeError, KeyError) as error:
            differences['gaussian_publication'] = str(error)
    return {"passed": not differences, "differences": differences, "arrays": arrays,
            "gaussian_finalization": finalization_report, "joint_render": joint_report,
            "gaussian_publication": publication_report,
            "gaussian_map": map_report, "confirmed_training_operations": confirmed_training,
            "raw_files": len(raw_off), "loop_stats": loops, "gaussian_stats": stats,
            "first_preview_during_playback_s": first, "previews": preview_report,
            "node_peak_rss_mib": [summary.get("peak_sampled_rss_mib") for summary in summaries]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--off", type=Path, required=True)
    parser.add_argument("--on", type=Path, required=True)
    parser.add_argument("--previews", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--joint-render-comparison', action='store_true',
                        help='Compare rendering OFF/ON with persistent training enabled in both')
    mode.add_argument('--preview-publication-comparison', action='store_true',
                      help='Keep persistent joint rendering ON and compare ROS publication OFF/ON')
    parser.add_argument('--min-joint-dual', type=int, default=0)
    parser.add_argument('--min-joint-query-groups', type=int, default=0)
    args = parser.parse_args()
    if args.min_joint_dual < 0 or args.min_joint_query_groups < 0:
        parser.error('Joint render evidence thresholds must be nonnegative')
    try:
        result = compare(args.off, args.on, args.previews, args.joint_render_comparison,
                         args.min_joint_dual, args.min_joint_query_groups,
                         args.preview_publication_comparison)
    except (OSError, ValueError, TypeError, KeyError) as error:
        print(json.dumps({"passed": False, "error": str(error)}, indent=2))
        return 2
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
