"""CPU-only, hand-specified schema 2 fixtures shared by protocol and worker tests."""

import numpy as np

from test_online_protocol import view_fixture


SESSION = "a" * 32


def target_fixture(kind="activate", count=2, first=0, submap=0, generation=1,
                   version=0, action="empty", job=1):
    header = {
        "schema": 2, "type": kind, "session_id": SESSION, "job_id": str(job),
        "generation": str(generation), "submap_id": str(submap), "pose_version": str(version),
        "frame_count": str(first + count), "optimized_frame_count": str(first + count if version else 0),
        "membership_revision": str(count), "anchor_keyframe_id": str(first),
        "primary_count": count, "submap_state": "active", "parent_frame": "map" if version else "odom",
        "parent_from_submap": [0, -1, 0, 4, 1, 0, 0, -2, 0, 0, 1, 1, 0, 0, 0, 1],
    }
    if kind == "activate":
        header["action"] = action
    poses = np.tile(np.eye(4, dtype="<f8"), (count, 1, 1))
    poses[:, 0, 3] = np.arange(count)
    return header, poses.tobytes()


def view_v2_fixture(frame=0, count=2, first=0, submap=0, generation=1, version=0, job=1):
    header, pixels = view_fixture()
    target, poses = target_fixture("view", count, first, submap, generation, version, job=job)
    header.update(target, keyframe_id=str(frame))
    local = frame - first
    # C_B is independently fixed; C_S = C_B * inverse(S_B), with S_B = tx(local).
    header["camera_from_body"] = [0, -1, 0, 1, 1, 0, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1]
    header["camera_from_submap"] = [0, -1, 0, 1, 1, 0, 0, 2-local, 0, 0, 1, 3, 0, 0, 0, 1]
    return header, pixels + poses


def control_fixture(kind, job=1, generation=1, version=0, submap=None, **fields):
    result = {"schema": 2, "type": kind, "session_id": SESSION, "job_id": str(job),
              "generation": str(generation), "pose_version": str(version), **fields}
    if submap is not None:
        result["submap_id"] = str(submap)
    return result


def candidate_fixture():
    return control_fixture("candidate", submap=0, operation="reuse", membership_revision="2",
                           frame_count="2", optimized_frame_count="0", frame_ids=["0", "1"],
                           width=8, height=6, steps=0, gaussian_count=2, supervised_depth_pixels=0,
                           rgb_l1=0.1, depth_l1_m=0.0, nonzero_gradient=0, parameter_updated=0,
                           capture_time=100.01)
