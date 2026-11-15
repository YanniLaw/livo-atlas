"""Camera conversion and a single gsplat rendering operation."""

import importlib

import numpy as np
import torch
from gsplat import rasterization


def to_gsplat_camera(K, camera_from_parent):
    """Convert OpenCV pixel centers while retaining the pose direction."""
    intrinsic = np.asarray(K, dtype=np.float64)
    pose = np.asarray(camera_from_parent, dtype=np.float64)
    if intrinsic.shape != (3, 3) or pose.shape != (4, 4):
        raise ValueError("Camera matrices have invalid shapes")
    if not np.isfinite(intrinsic).all() or not np.isfinite(pose).all():
        raise ValueError("Camera matrices must be finite")
    if (intrinsic[0, 0] <= 0 or intrinsic[1, 1] <= 0 or
            not np.allclose(intrinsic[2], [0, 0, 1], atol=1e-12, rtol=0) or
            abs(intrinsic[0, 1]) > 1e-12 or abs(intrinsic[1, 0]) > 1e-12):
        raise ValueError("Invalid pinhole intrinsics")
    rotation = pose[:3, :3]
    if (not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-9, rtol=0) or
            not np.allclose(rotation.T @ rotation, np.eye(3),
                            atol=1e-6, rtol=0) or
            abs(np.linalg.det(rotation) - 1) > 1e-6):
        raise ValueError("Camera pose must be a rigid transform")
    converted = intrinsic.copy()
    converted[0, 2] += 0.5
    converted[1, 2] += 0.5
    return converted, pose.copy()


def render(means, quats, scales, opacities, colors, viewmat, K_gsplat,
           width, height):
    """Render one camera; depth is alpha-normalized camera z in meters."""
    tensors = (means, quats, scales, opacities, colors)
    if not all(isinstance(value, torch.Tensor) for value in tensors):
        raise ValueError("Gaussian parameters must be tensors")
    if means.ndim != 2 or means.shape[1] != 3 or means.shape[0] == 0:
        raise ValueError("Gaussian centers must have shape [N, 3]")
    count = means.shape[0]
    if (quats.shape != (count, 4) or scales.shape != (count, 3) or
            opacities.shape != (count,) or colors.shape != (count, 3)):
        raise ValueError("Gaussian parameter shapes do not match")
    if (means.device.type != "cuda" or means.dtype != torch.float32 or
            any(value.device != means.device or value.dtype != means.dtype
                for value in tensors)):
        raise ValueError("Gaussian parameters must share a CUDA float32 device")
    if (not isinstance(width, int) or isinstance(width, bool) or width <= 0 or
            not isinstance(height, int) or isinstance(height, bool) or
            height <= 0):
        raise ValueError("Image dimensions must be positive integers")
    if any(not torch.isfinite(value).all().item() for value in tensors):
        raise ValueError("Gaussian parameters must be finite")
    if (torch.any(scales <= 0).item() or
            torch.any(quats.square().sum(dim=1) <= 1e-12).item() or
            torch.any((opacities < 0) | (opacities > 1)).item() or
            torch.any((colors < 0) | (colors > 1)).item()):
        raise ValueError("Gaussian parameters are outside their valid ranges")
    pose = torch.as_tensor(viewmat, device=means.device, dtype=means.dtype)
    intrinsic = torch.as_tensor(K_gsplat, device=means.device, dtype=means.dtype)
    if (pose.shape != (4, 4) or intrinsic.shape != (3, 3) or
            not torch.isfinite(pose).all().item() or
            not torch.isfinite(intrinsic).all().item()):
        raise ValueError("Invalid renderer camera matrices")
    # gsplat otherwise falls back to an uncontrolled JIT build on import failure.
    try:
        importlib.import_module("gsplat.csrc")
    except (ImportError, OSError) as error:
        raise RuntimeError("A loadable prebuilt gsplat.csrc is required") from error
    rendered, alphas, _ = rasterization(
        means, quats, scales, opacities, colors, pose.unsqueeze(0),
        intrinsic.unsqueeze(0), width, height, render_mode="RGB+ED"
    )
    if (rendered.shape != (1, height, width, 4) or
            alphas.shape != (1, height, width, 1) or
            not torch.isfinite(rendered).all().item() or
            not torch.isfinite(alphas).all().item()):
        raise RuntimeError("Renderer returned invalid output")
    return rendered[0, ..., :3], alphas[0, ..., 0], rendered[0, ..., 3]
