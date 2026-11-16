#!/usr/bin/env python3
"""Inspect a CPU Gaussian asset or verify model restoration on a view bundle."""

import argparse
import gc
import hashlib
import json
import os
from pathlib import Path
import resource
import time
import uuid
import weakref

import numpy as np

from gaussian_state import (CommitUncertainError, _LIMITS, _unsigned, read_state,
                            write_state)


def _new_output(source, output):
    source = source.resolve(strict=True)
    output = output.absolute()
    resolved = output.resolve()
    if os.path.lexists(output):
        raise ValueError("Output directory must not exist")
    if resolved.is_relative_to(source) or source.is_relative_to(resolved):
        raise ValueError("Input and output directories must be separate")
    if not output.parent.is_dir():
        raise ValueError("Output parent directory must exist")
    return output


def _load_probe(path, anchor_id, submap_id):
    # The existing bundle reader is eager. Bound the small manifest first.
    manifest = path / "bundle.json"
    if not 1 <= manifest.stat().st_size <= 512 * 1024:
        raise ValueError("Bundle manifest exceeds its budget")
    raw = manifest.read_bytes()
    document = json.loads(raw)
    entries = document.get("views")
    if not isinstance(entries, list) or not 3 <= len(entries) <= 12:
        raise ValueError("Roundtrip needs at most ten train and two validation views")
    splits = [entry.get("split") for entry in entries]
    if (not 2 <= splits.count("train") <= 10 or not 1 <= splits.count("validation") <= 2 or
            set(splits) != {"train", "validation"}):
        raise ValueError("Roundtrip needs two to ten train and one to two validation views")
    for entry in entries:
        width, height = entry.get("width"), entry.get("height")
        if (type(width) is not int or type(height) is not int or width <= 0 or height <= 0 or
                width * height > 1_000_000):
            raise ValueError("Bundle pixel budget exceeded")
        if "parent_from_body" not in entry:
            raise ValueError("Bundle needs body poses; prepare it again with the current exporter")
    ids = sorted(_unsigned(entry.get("keyframe_id")) for entry in entries)
    frontiers = {_unsigned(entry.get("frame_count")) for entry in entries}
    if len(frontiers) != 1 or not ids[-1] < next(iter(frontiers)) <= 512:
        raise ValueError("Probe must have one covered pose frontier of at most 512 frames")
    # Compare only the supplied IDs; never allocate a range from their span.
    if anchor_id != ids[0] or any(key != ids[0] + offset for offset, key in enumerate(ids)):
        raise ValueError("Probe members must be contiguous and anchored at their first body")
    from bundle import load_bundle

    views = sorted(load_bundle(path), key=lambda view: view["keyframe_id"])
    parent_anchor = views[0]["parent_from_body"]
    anchor_inverse = np.linalg.inv(parent_anchor)
    context = {
        "capture_kind": "offline_probe", "origin_session_id": uuid.uuid4().hex,
        "submap_id": str(submap_id), "anchor_keyframe_id": str(anchor_id),
        "membership_revision": str(len(views) + 1), "submap_state": "sealed",
        "generation": "1", "pose_version": "0", "parent_frame": "odom",
        "frame_count": str(views[0]["frame_count"]), "optimized_frame_count": "0",
        "parent_from_submap": parent_anchor.ravel().tolist(),
        "primary": [{"keyframe_id": str(view["keyframe_id"]),
                     "submap_from_body": (anchor_inverse @ view["parent_from_body"]).ravel().tolist()}
                    for view in views],
        "source_manifest_sha256": document["source_manifest_sha256"],
        "bundle_manifest_sha256": hashlib.sha256(raw).hexdigest(),
    }
    prepared, sources = [], {}
    for view in views:
        # Derive the independent camera/body reference before changing frames.
        sources[view["keyframe_id"]] = (
            view["camera_from_parent"] @ view["parent_from_body"]).ravel().tolist()
        local = dict(view)
        local["camera_from_parent"] = view["camera_from_parent"] @ parent_anchor
        local["camera_from_submap"] = local["camera_from_parent"].ravel().tolist()
        local["parent_from_submap"] = context["parent_from_submap"]
        local["session_id"] = context["origin_session_id"]
        for key in ("submap_id", "generation", "pose_version", "frame_count",
                    "optimized_frame_count", "membership_revision", "parent_frame"):
            local[key] = context[key]
        prepared.append(local)
    return prepared, sources, context


def _same_array(before, after):
    return (before.dtype == after.dtype and before.shape == after.shape and
            np.array_equal(before.view("u1"), after.view("u1")))


def _render_cpu(model, view):
    import torch
    from train import _render_view

    with torch.no_grad():
        return [value.cpu().numpy().copy() for value in _render_view(model.parameters, view)]


def _roundtrip(args, report):
    output = _new_output(args.bundle, args.output)
    views, sources, context = _load_probe(args.bundle, args.anchor_id, args.submap_id)
    train = [view for view in views if view["split"] == "train"]
    import torch
    from online_model import OnlineGaussianModel, checked_cuda_device

    device = checked_cuda_device()
    torch.set_num_threads(1)
    torch.cuda.reset_peak_memory_stats()
    model = OnlineGaussianModel(device)
    model.reset(1)
    output.mkdir()
    report.update(capture_kind="offline_probe", origin_session_id=context["origin_session_id"],
                  source_manifest_sha256=context["source_manifest_sha256"],
                  bundle_manifest_sha256=context["bundle_manifest_sha256"],
                  train_ids=[view["keyframe_id"] for view in train],
                  validation_ids=[view["keyframe_id"] for view in views if view["split"] == "validation"])
    for view in train[:-1]:
        model.train_candidate(view)
        model.decide(True)
    context["source_cameras"] = [{"keyframe_id": str(key), "camera_from_body": sources[key]}
                                 for key in model.frame_ids]
    state = model.capture_state(context)
    report["software"] = state.metadata["software"]
    report["gaussian_count"] = int(state.origin_ids.size)
    before_float = _render_cpu(model, model.views[-1])
    before_rgb8 = model.render_preview(model.frame_ids[-1])
    write_state(output / "state", state)
    original = weakref.ref(model)
    allocated_before = torch.cuda.memory_allocated()
    del model
    gc.collect()
    torch.cuda.synchronize()
    allocated_after = torch.cuda.memory_allocated()
    report["cold_restore"] = {"released_original": original() is None,
                               "allocated_before_release_bytes": allocated_before,
                               "allocated_after_release_bytes": allocated_after}
    if original() is not None or allocated_after >= allocated_before:
        raise RuntimeError("Original GPU working set was not released")
    loaded = read_state(output / "state")
    restored = OnlineGaussianModel.from_state(loaded, device)
    report["cold_restore"]["allocated_after_restore_bytes"] = torch.cuda.memory_allocated()
    parameters_equal = (all(_same_array(value, loaded.parameters[name]) and
                            _same_array(value, restored.parameters[name].detach().cpu().numpy())
                            for name, value in state.parameters.items()) and
                        _same_array(state.origin_ids, loaded.origin_ids) and
                        np.array_equal(state.origin_ids, restored.origin_ids.cpu().numpy()))
    view_fields = ("rgb", "image_mask", "depth_mask", "depth_m", "K", "camera_from_body", "camera_from_parent")
    observations_equal = (state.metadata == loaded.metadata and
                           all(_same_array(before[key], after[key])
                               for before, after in zip(state.views, loaded.views) for key in view_fields))
    for before, after in zip(state.views, restored.views):
        for key in ("rgb", "image_mask", "depth_mask", "depth_m"):
            observations_equal &= _same_array(before[key], after[key].cpu().numpy())
    report.update(parameter_bitwise_equal=parameters_equal, observation_bitwise_equal=bool(observations_equal))
    if not parameters_equal or not observations_equal:
        raise RuntimeError("Restored parameters or observations changed")
    after_float = _render_cpu(restored, restored.views[-1])
    after_rgb8 = restored.render_preview(restored.frame_ids[-1])
    differences = [float(np.max(np.abs(before - after))) for before, after in zip(before_float, after_float)]
    rgb8_difference = int(np.abs(before_rgb8.astype(int) - after_rgb8.astype(int)).max())
    report["render_max_abs"] = dict(zip(("rgb", "alpha", "depth_m"), differences))
    report["render_max_abs"]["rgb8"] = rgb8_difference
    for index, (before, after) in enumerate(zip(before_float, after_float)):
        if not np.allclose(before, after, atol=1e-5 if index == 2 else 1e-6,
                           rtol=1e-6 if index == 2 else 0):
            raise RuntimeError("Restored float render exceeded the fixed tolerance")
    if rgb8_difference > 1:
        raise RuntimeError("Restored RGB8 preview changed by more than one level")
    from PIL import Image

    Image.fromarray(before_rgb8, "RGB").save(output / "before.png")
    Image.fromarray(after_rgb8, "RGB").save(output / "restored.png")
    _, metrics = restored.train_candidate(train[-1])
    restored.decide(True)
    report["continued"] = {"keyframe_id": train[-1]["keyframe_id"], **{
        name: metrics[name] for name in ("steps", "nonzero_gradient", "parameter_updated",
                                        "supervised_depth_pixels", "gaussian_count", "frame_ids")}}
    if (metrics["steps"] != 20 or not metrics["nonzero_gradient"] or not metrics["parameter_updated"] or
            metrics["supervised_depth_pixels"] <= 0):
        raise RuntimeError("Restored model did not complete valid continued training")
    Image.fromarray(restored.render_preview(train[-1]["keyframe_id"]), "RGB").save(output / "continued.png")
    torch.cuda.synchronize()
    report.update(status="passed", gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated(),
                  gpu_peak_reserved_bytes=torch.cuda.max_memory_reserved())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    inspect = commands.add_parser("inspect", help="Validate an asset using only the CPU")
    inspect.add_argument("--state", type=Path, required=True)
    run = commands.add_parser("roundtrip", help="Train, save, release, restore and continue on a bounded probe")
    run.add_argument("--bundle", type=Path, required=True)
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--anchor-id", type=_unsigned, required=True)
    run.add_argument("--submap-id", type=_unsigned, required=True)
    args = parser.parse_args()
    report = {"status": "failed"}
    started = time.monotonic()
    code = 0
    try:
        if args.command == "inspect":
            state = read_state(args.state)
            report.update({key: state.metadata[key] for key in (
                "schema", "representation", "capture_kind", "origin_session_id", "submap_id",
                "anchor_keyframe_id", "membership_revision", "generation", "pose_version",
                "parent_frame", "submap_state", "frame_count", "optimized_frame_count",
                "optimizer", "software", "source_manifest_sha256", "bundle_manifest_sha256")
                if key in state.metadata})
            pixels = [int(view["depth_m"].size) for view in state.views]
            budgets = {key: {"used": used, "limit": limit} for key, used, limit in (
                ("primary_frames", len(state.metadata["primary"]), 512),
                ("retained_views", len(state.views), 10),
                ("retained_pixels", sum(pixels), 10_000_000),
                ("max_view_pixels", max(pixels), 1_000_000),
                ("gaussians", int(state.origin_ids.size), 10_000))}
            budgets["files"] = {name: {"used": (args.state / name).stat().st_size, "limit": limit}
                                for name, limit in _LIMITS.items()}
            report.update(status="passed", gaussian_count=int(state.origin_ids.size),
                          primary_ids=[int(entry["keyframe_id"]) for entry in state.metadata["primary"]],
                          retained_ids=[view["keyframe_id"] for view in state.views],
                          budget_checks=budgets)
        else:
            _roundtrip(args, report)
    except Exception as error:
        report.update(status="commit_uncertain" if isinstance(error, CommitUncertainError) else "failed",
                      error_type=type(error).__name__, error=str(error))
        code = 3 if isinstance(error, CommitUncertainError) else 1
    report.update(elapsed_seconds=time.monotonic() - started,
                  peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024)
    if args.command == "roundtrip" and args.output.is_dir() and not args.output.is_symlink():
        # Only a directory created by this invocation may receive a report.
        if "capture_kind" in report:
            (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2, allow_nan=False))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
