"""Small, independently specified Gaussian state fixtures; no GPU imports."""

import numpy as np

from gaussian_state import GaussianState


def pose(x=0, y=0, z=0, rotated=False):
    result = np.eye(4, dtype=np.float64)
    if rotated:
        result[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    result[:3, 3] = [x, y, z]
    return result


def metadata(count, parent=None):
    return {
        "schema": 1, "kind": "livo_atlas.gaussian_state",
        "representation": "gsplat-sh0-window-v1", "optimizer": "reset_on_restore",
        "capture_kind": "offline_probe", "origin_session_id": "a" * 32,
        "submap_id": "0", "anchor_keyframe_id": "0", "membership_revision": str(count),
        "generation": "1", "pose_version": "0", "frame_count": str(count),
        "optimized_frame_count": "0", "parent_frame": "odom", "submap_state": "active",
        "primary": [{"keyframe_id": str(i), "submap_from_body": pose(i).ravel().tolist()}
                    for i in range(count)],
        "parent_from_submap": (pose(4, -2, 1, True) if parent is None else parent).ravel().tolist(),
        "software": {key: "test" for key in ("python", "numpy", "torch", "gsplat", "cuda")},
        "source_manifest_sha256": "b" * 64, "bundle_manifest_sha256": "c" * 64,
    }


def make_state(count=2, width=8, height=6, per_view=1):
    views = []
    for i in range(count):
        rgb = np.empty((height, width, 3), dtype="<f4")
        rgb[:] = [0.1, 0.2, 0.3]
        depth = np.zeros((height, width), dtype="<f4")
        depth[height // 2, width // 2] = 4
        views.append({
            "keyframe_id": i, "calibration_id": 0,
            "state_timestamp": 100.0 + i, "cloud_timestamp": 99.99 + i,
            "image_timestamp": 100.01 + i, "inverse_exposure_time": 1.0,
            "rgb": rgb, "image_mask": np.ones((height, width), dtype=bool),
            "depth_mask": depth > 0, "depth_m": depth,
            "K": np.array([[4, 0, 3.5], [0, 4, 2.5], [0, 0, 1]], dtype=np.float64),
            "camera_from_body": pose(1, 2, 3, True),
            "camera_from_parent": pose(1, 2 - i, 3, True),
        })
    n = count * per_view
    return GaussianState(metadata(count), {
        "means": np.column_stack((np.arange(n), np.zeros(n), np.full(n, 4))).astype("<f4"),
        "quats": np.tile(np.array([1, 0, 0, 0], dtype="<f4"), (n, 1)),
        "log_scales": np.full((n, 3), np.log(0.03), dtype="<f4"),
        "color_logits": np.full((n, 3), 0.3, dtype="<f4"),
        "opacity_logits": np.zeros(n, dtype="<f4"),
    }, np.repeat(np.arange(count, dtype="<u8"), per_view), views)


def context_for_model(frame_ids, count=None):
    count = max(frame_ids) + 1 if count is None else count
    result = metadata(count, np.eye(4))
    for key in ("schema", "kind", "representation", "optimizer", "software"):
        result.pop(key)
    # Model fixture cameras are Rz90/t(1,2,3) in S, bodies translate by id.
    result["source_cameras"] = [
        {"keyframe_id": str(i), "camera_from_body": pose(1, 2 + i, 3, True).ravel().tolist()}
        for i in frame_ids
    ]
    return result
