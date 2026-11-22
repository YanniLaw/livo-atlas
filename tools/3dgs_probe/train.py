"""Fixed-size offline Gaussian experiment using saved reconstruction views."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import gsplat
import numpy as np
from PIL import Image
import torch

from bundle import load_bundle
from rasterizer import render, to_gsplat_camera


_ALPHA_THRESHOLD = 0.05


def _training_ids(views):
    ids = sorted(view["keyframe_id"] for view in views if view["split"] == "train")
    if not ids or len(ids) != len(set(ids)):
        raise ValueError("A nonempty set of unique training IDs is required")
    return ids


def training_schedule(views, steps):
    """Cycle through training IDs in ascending order, never through held-out IDs."""
    if type(steps) is not int or steps < 0:
        raise ValueError("steps must be a nonnegative integer")
    ids = _training_ids(views)
    return [ids[step % len(ids)] for step in range(steps)]


def _depth_candidates(view):
    return (np.asarray(view["image_mask"], dtype=bool) &
            np.asarray(view["depth_mask"], dtype=bool))


def _reserved_seed_count(candidates, uncovered, limit):
    if candidates > limit and 0 < uncovered < candidates:
        return min(limit // 2, uncovered)
    return 0


def _evenly_spaced(pixels, count):
    return pixels[np.arange(count, dtype=np.int64) * pixels.size // count]


def _seed_pixels(mask, limit, uncovered_mask):
    valid = np.flatnonzero(mask)
    count = min(valid.size, limit)
    if uncovered_mask is not None:
        uncovered = np.asarray(uncovered_mask)
        if uncovered.dtype != np.dtype(bool) or uncovered.ndim != 2 or uncovered.shape != mask.shape:
            raise ValueError("uncovered_mask must be boolean and match the training image")
        holes = valid[uncovered.ravel()[valid]]
        quota = _reserved_seed_count(valid.size, holes.size, limit)
        if quota:
            reserved = _evenly_spaced(holes, quota)
            remaining = np.setdiff1d(valid, reserved, assume_unique=True)
            return np.sort(np.concatenate((reserved, _evenly_spaced(remaining, count - quota))))
    return _evenly_spaced(valid, count) if count else valid


def initialize_gaussians(train_views, max_per_view, *, uncovered_mask=None):
    """Backproject a deterministic subset of training depths into the parent frame."""
    if type(max_per_view) is not int or max_per_view <= 0:
        raise ValueError("max_per_view must be a positive integer")
    order = _training_ids(train_views)
    if uncovered_mask is not None and len(order) != 1:
        raise ValueError("uncovered_mask requires exactly one training view")
    by_id = {view["keyframe_id"]: view for view in train_views
             if view["split"] == "train"}
    means, colors = [], []
    for keyframe_id in order:
        view = by_id[keyframe_id]
        mask = _depth_candidates(view)
        pixels = _seed_pixels(mask, max_per_view, uncovered_mask)
        count = pixels.size
        if count == 0:
            continue
        rows, cols = np.divmod(pixels, mask.shape[1])
        depths = view["depth_m"][rows, cols].astype(np.float64)
        intrinsic = view["K"]
        camera_points = np.column_stack((
            (cols - intrinsic[0, 2]) * depths / intrinsic[0, 0],
            (rows - intrinsic[1, 2]) * depths / intrinsic[1, 1],
            depths,
            np.ones(count),
        ))
        parent_from_camera = np.linalg.inv(view["camera_from_parent"])
        means.append((camera_points @ parent_from_camera.T)[:, :3])
        colors.append(view["rgb"][rows, cols])
    if not means:
        raise ValueError("No training depth is available for initialization")
    means = np.concatenate(means).astype(np.float32)
    colors = np.concatenate(colors).astype(np.float32)
    count = means.shape[0]
    quats = np.zeros((count, 4), dtype=np.float32)
    quats[:, 0] = 1
    return {
        "means": means,
        "quats": quats,
        "scales": np.full((count, 3), 0.03, dtype=np.float32),
        "colors": colors,
        "opacities": np.full(count, 0.5, dtype=np.float32),
    }


def create_parameters(initial, device):
    """Create unconstrained trainable values and fixed unit quaternions."""
    def tensor(value):
        return torch.as_tensor(value, dtype=torch.float32, device=device)

    values = {
        "means": tensor(initial["means"]),
        "quats": tensor(initial["quats"]),
        "log_scales": torch.log(tensor(initial["scales"])),
        "color_logits": torch.logit(tensor(initial["colors"]).clamp(1e-4, 1 - 1e-4)),
        "opacity_logits": torch.logit(tensor(initial["opacities"])),
    }
    if any(not torch.isfinite(value).all().item() for value in values.values()):
        raise ValueError("Initial Gaussian parameters must be finite")
    return {name: value if name == "quats" else torch.nn.Parameter(value)
            for name, value in values.items()}


def masked_losses(rgb, depth, alpha, view):
    """Return image L1, observed-depth L1 and the depth sample count."""
    device = rgb.device
    target = torch.as_tensor(view["rgb"], dtype=rgb.dtype, device=device)
    image_mask = torch.as_tensor(view["image_mask"], dtype=torch.bool, device=device)
    depth_mask = torch.as_tensor(view["depth_mask"], dtype=torch.bool, device=device)
    observed_depth = torch.as_tensor(view["depth_m"], dtype=depth.dtype, device=device)
    if (rgb.shape != target.shape or rgb.ndim != 3 or rgb.shape[-1] != 3 or
            depth.shape != rgb.shape[:2] or alpha.shape != depth.shape or
            image_mask.shape != depth.shape or depth_mask.shape != depth.shape or
            observed_depth.shape != depth.shape):
        raise ValueError("Render and observation dimensions do not match")
    if not image_mask.any().item():
        raise ValueError("Image has no valid pixels")
    image_loss = torch.abs(rgb[image_mask] - target[image_mask]).mean()
    selected = image_mask & depth_mask & (alpha.detach() >= _ALPHA_THRESHOLD)
    count = int(selected.sum().item())
    depth_loss = (torch.abs(depth[selected] - observed_depth[selected]).mean()
                  if count else depth.new_zeros(()))
    return image_loss, depth_loss, count


def _render_view(parameters, view):
    intrinsic, pose = to_gsplat_camera(view["K"], view["camera_from_parent"])
    height, width = view["image_mask"].shape
    return render(
        parameters["means"], parameters["quats"],
        parameters["log_scales"].exp(), parameters["opacity_logits"].sigmoid(),
        parameters["color_logits"].sigmoid(), pose, intrinsic, width, height,
    )


def _save_rgb(path, rgb):
    values = np.clip(np.rint(rgb.detach().cpu().numpy() * 255), 0, 255)
    Image.fromarray(values.astype(np.uint8), mode="RGB").save(path)


def _evaluate(parameters, views, output, stage, first_render):
    metrics = {}
    with torch.no_grad():
        for view in views:
            if not first_render:
                torch.cuda.synchronize()
                start = time.perf_counter()
                start_event = torch.cuda.Event(enable_timing=True)
                end_event = torch.cuda.Event(enable_timing=True)
                start_event.record()
            rgb, alpha, depth = _render_view(parameters, view)
            if not first_render:
                end_event.record()
                torch.cuda.synchronize()
                first_render.update({
                    "wall_seconds": time.perf_counter() - start,
                    "cuda_event_ms": start_event.elapsed_time(end_event),
                })
            image_loss, depth_loss, depth_count = masked_losses(rgb, depth, alpha, view)
            image_mask = view["image_mask"]
            image_count = int(image_mask.sum().item())
            coverage = (alpha[image_mask] >= _ALPHA_THRESHOLD).sum().item() / image_count
            mse = (rgb[image_mask] - view["rgb"][image_mask]).square().mean().item()
            if not all(math.isfinite(value) for value in (
                    image_loss.item(), depth_loss.item(), coverage, mse)):
                raise RuntimeError("Evaluation produced a nonfinite metric")
            key = str(view["keyframe_id"])
            metrics[key] = {
                "split": view["split"],
                "rgb_l1": image_loss.item(),
                "rgb_psnr_db": -10 * math.log10(mse) if mse > 0 else None,
                "alpha_coverage_at_0_05": coverage,
                "image_valid_pixels": image_count,
                "observed_depth_pixels": int(view["depth_mask"].sum().item()),
                "supervised_depth_pixels": depth_count,
                "depth_l1_m": depth_loss.item() if depth_count else None,
            }
            if view["split"] == "validation":
                _save_rgb(output / f"validation_{key}_{stage}.png", rgb)
                if stage == "initial":
                    _save_rgb(output / f"validation_{key}_target.png", view["rgb"])
    return metrics


def _system_gpu_snapshot():
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=index,name,driver_version,memory.used,memory.free",
             "--format=csv,noheader,nounits"],
            check=True, capture_output=True, text=True, timeout=10,
        )
        return result.stdout.strip().splitlines()
    except (OSError, subprocess.SubprocessError):
        return None


def _source_versions():
    root = Path(__file__).resolve().parents[2]
    paths = ("tools/3dgs_probe/train.py", "tools/3dgs_probe/rasterizer.py",
             "tools/3dgs_probe/bundle.py", "tools/3dgs_probe/prepare_views.cc")
    hashes = {path: hashlib.sha256((root / path).read_bytes()).hexdigest()
              for path in paths}
    try:
        commit = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"],
                                check=True, capture_output=True, text=True).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        commit = None
    return {"git_head": commit, "sha256": hashes}


def _gpu_views(views, device):
    result = []
    for view in views:
        copied = dict(view)
        for name in ("rgb", "image_mask", "depth_mask", "depth_m"):
            copied[name] = torch.as_tensor(view[name], device=device)
        result.append(copied)
    return result


def run_experiment(views, output, steps, seed, report):
    train_ids = [0, 1, 2, 4, 5, 6, 8, 9, 10, 11]
    validation_ids = [3, 7]
    if (_training_ids(views) != train_ids or
            sorted(view["keyframe_id"] for view in views
                   if view["split"] == "validation") != validation_ids):
        raise ValueError("The experiment requires the documented 10/2 split")
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required")
    free, total = torch.cuda.mem_get_info()
    report["gpu_before"] = {
        "free_bytes": free, "total_bytes": total, "system": _system_gpu_snapshot()
    }
    if free < 4 * 2**30:
        raise RuntimeError("At least 4 GiB of free GPU memory is required")
    torch.cuda.set_per_process_memory_fraction(0.4)
    torch.cuda.reset_peak_memory_stats()
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    torch.manual_seed(seed)
    np.random.seed(seed)
    device = torch.device("cuda:0")
    start = time.perf_counter()
    initial = initialize_gaussians(views, 1000)
    parameters = create_parameters(initial, device)
    gpu_views = _gpu_views(views, device)
    torch.cuda.synchronize()
    report["initialization_seconds"] = time.perf_counter() - start
    report["gaussian_count"] = len(initial["means"])
    report["software"] = {
        "python": sys.version.split()[0], "torch": torch.__version__,
        "torch_cuda": torch.version.cuda, "gsplat": gsplat.__version__,
        "gpu": torch.cuda.get_device_name(device),
        "compute_capability": list(torch.cuda.get_device_capability(device)),
    }
    report["source"] = _source_versions()
    report["source_manifest_sha256"] = views[0]["source_manifest_sha256"]
    first_render = {}
    report["first_render"] = first_render
    report["initial"] = _evaluate(parameters, gpu_views, output, "initial", first_render)
    original = {name: value.detach().clone() for name, value in parameters.items()
                if name != "quats"}
    optimizer = torch.optim.Adam([
        {"params": [parameters["means"]], "lr": 5e-4},
        {"params": [parameters["log_scales"]], "lr": 2e-3},
        {"params": [parameters["color_logits"]], "lr": 1e-2},
        {"params": [parameters["opacity_logits"]], "lr": 1e-2},
    ])
    by_id = {view["keyframe_id"]: view for view in gpu_views}
    schedule = training_schedule(views, steps)
    wall_times, cuda_times = [], []
    report["actual_steps"] = 0
    report["training_depth_samples"] = 0
    report["nonzero_gradient_seen"] = False
    train_start = time.perf_counter()
    for keyframe_id in schedule:
        if time.perf_counter() - train_start >= 300:
            raise RuntimeError("Training exceeded its 300 second wall-time limit")
        torch.cuda.synchronize()
        step_start = time.perf_counter()
        start_event = torch.cuda.Event(enable_timing=True)
        end_event = torch.cuda.Event(enable_timing=True)
        start_event.record()
        optimizer.zero_grad(set_to_none=True)
        rgb, alpha, depth = _render_view(parameters, by_id[keyframe_id])
        image_loss, depth_loss, depth_count = masked_losses(
            rgb, depth, alpha, by_id[keyframe_id]
        )
        loss = image_loss + 0.05 * depth_loss
        if not torch.isfinite(loss).item():
            raise RuntimeError("Training loss is nonfinite")
        loss.backward()
        for name, parameter in parameters.items():
            if name == "quats":
                continue
            if parameter.grad is None or not torch.isfinite(parameter.grad).all().item():
                raise RuntimeError("Training gradient is missing or nonfinite: " + name)
            report["nonzero_gradient_seen"] |= bool(parameter.grad.any().item())
        optimizer.step()
        with torch.no_grad():
            parameters["log_scales"].clamp_(math.log(0.001), math.log(0.2))
        if any(not torch.isfinite(value).all().item() for value in parameters.values()):
            raise RuntimeError("Updated Gaussian parameter is nonfinite")
        end_event.record()
        torch.cuda.synchronize()
        wall_times.append(time.perf_counter() - step_start)
        cuda_times.append(start_event.elapsed_time(end_event))
        report["actual_steps"] += 1
        report["training_depth_samples"] += depth_count
        report["training_wall_seconds"] = time.perf_counter() - train_start
        if report["training_wall_seconds"] > 300:
            raise RuntimeError("Training exceeded its 300 second wall-time limit")
    report["step_timing"] = {
        "wall_seconds": wall_times,
        "cuda_event_ms": cuda_times,
        "warm_wall_median_seconds": float(np.median(wall_times[1:] or wall_times)),
        "warm_wall_p95_seconds": float(np.percentile(wall_times[1:] or wall_times, 95)),
        "warm_cuda_median_ms": float(np.median(cuda_times[1:] or cuda_times)),
        "warm_cuda_p95_ms": float(np.percentile(cuda_times[1:] or cuda_times, 95)),
    }
    report["parameter_updated"] = any(
        not torch.equal(parameters[name].detach(), before)
        for name, before in original.items()
    )
    report["final"] = _evaluate(parameters, gpu_views, output, "final", first_render)
    report["gpu_after"] = {
        "free_bytes": torch.cuda.mem_get_info()[0],
        "allocated_peak_bytes": torch.cuda.max_memory_allocated(),
        "reserved_peak_bytes": torch.cuda.max_memory_reserved(),
        "system": _system_gpu_snapshot(),
    }
    before = np.mean([report["initial"][str(i)]["rgb_l1"] for i in train_ids])
    after = np.mean([report["final"][str(i)]["rgb_l1"] for i in train_ids])
    report["train_rgb_l1_mean_initial"] = float(before)
    report["train_rgb_l1_mean_final"] = float(after)
    report["relative_rgb_l1_reduction"] = float((before - after) / before) if before else 0
    if (not report["nonzero_gradient_seen"] or not report["parameter_updated"] or
            report["training_depth_samples"] == 0 or
            report["relative_rgb_l1_reduction"] < 0.01):
        raise RuntimeError("The experiment did not meet its acceptance criteria")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--steps", required=True, type=int)
    parser.add_argument("--seed", required=True, type=int)
    args = parser.parse_args()
    if not 1 <= args.steps <= 100 or not 0 <= args.seed < 2**32:
        parser.error("steps must be 1–100 and seed must be a 32-bit unsigned integer")
    bundle = args.bundle.resolve(strict=True)
    output = args.output.resolve()
    if (args.output.exists() or args.output.is_symlink() or
            output == bundle or output.is_relative_to(bundle) or
            bundle.is_relative_to(output)):
        parser.error("output must be new and separate from the bundle")
    output.mkdir()
    report = {
        "status": "failed", "started_utc": datetime.now(timezone.utc).isoformat(),
        "configuration": {"steps": args.steps, "seed": args.seed,
                          "max_per_view": 1000, "memory_fraction": 0.4,
                          "max_train_seconds": 300},
    }
    try:
        views = load_bundle(bundle)
        run_experiment(views, output, args.steps, args.seed, report)
        report["status"] = "passed"
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
        print(report["error"], file=sys.stderr)
    report["finished_utc"] = datetime.now(timezone.utc).isoformat()
    temporary = output / "report.json.pending"
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    temporary.replace(output / "report.json")
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
