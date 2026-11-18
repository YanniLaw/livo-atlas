"""Independent small wire records; no CUDA imports."""
import copy
import numpy as np

from state_fixtures import pose
from submap_fixtures import SESSION, target_fixture


ASSET_FIELDS = ('asset_id', 'submap_id', 'source_generation', 'content_job_id',
                'manifest_sha256', 'manifest_bytes', 'gaussian_bytes', 'observation_bytes')
TARGET_FIELDS = ('generation', 'submap_id', 'membership_revision', 'frame_count',
                 'optimized_frame_count', 'anchor_keyframe_id', 'primary_count',
                 'submap_state', 'parent_frame', 'parent_from_submap', 'pose_version')


def render_request_fixture():
    header = dict(schema=2, type='joint_render', session_id=SESSION,
                  job_id='9007199254740993', generation='4', pose_version='0',
                  map_sequence='9', frame_count='6', optimized_frame_count='0',
                  parent_frame='odom', query_keyframe_id='5', query_submap_id='2',
                  neighbor_reason='', blocks=[])
    pieces = [pose(5).astype('<f8').tobytes()]
    for submap, first in ((1, 2), (2, 4)):
        target, body = target_fixture(submap=submap, first=first, generation=submap + 1)
        target.update(frame_count='6', submap_state='sealed', membership_revision='3',
                      parent_from_submap=pose(first).ravel().tolist())
        content = dict(submap_id=str(submap), source_generation=str(submap + 1),
                       content_job_id=str(submap + 10))
        asset = dict(content, asset_id=str(submap + 1), manifest_sha256='b' * 64,
                     manifest_bytes='2048', gaussian_bytes='1024', observation_bytes='4096')
        header['blocks'].append(dict(target={key: target[key] for key in TARGET_FIELDS},
            content=content, resident=0, asset=asset, frame_ids=[str(first), str(first+1)],
            gaussian_count=2))
        pieces.append(body)
    return header, b''.join(pieces)


def wire_metrics_fixture():
    return dict(image_pixels=48, covered_pixels=24, depth_pixels_available=4,
                depth_pixels_used=2, image_valid=1, depth_valid=1, psnr_valid=1,
                rgb_perfect=0, rgb_l1=.2, rgb_mse=.04, rgb_psnr_db=13.979400086720375,
                alpha_coverage=.5, depth_l1_m=.1, depth_rmse_m=.2)


def render_reply_fixture():
    header, _ = render_request_fixture()
    header = copy.deepcopy(header)
    header.update(type='joint_render_ok', training_view=1, width=8, height=6,
        parent_from_body=pose(5).ravel().tolist(),
        calibration_id='0', fx=4.0, fy=4.0, cx=3.5, cy=2.5,
        camera_from_body=pose(1, 2, 3, True).ravel().tolist(),
        camera_from_parent=pose(1, -3, 3, True).ravel().tolist(),
        image_timestamp=100.01, state_timestamp=100.0, cloud_timestamp=99.99,
        inverse_exposure_time=1.0, single=wire_metrics_fixture(), joint=wire_metrics_fixture(),
        changed_pixels=2, single_gaussian_count=2, gaussian_count=4,
        worker_seconds=.1, gpu_peak_allocated_bytes='123456',
        gpu_peak_reserved_bytes='234567', gpu_allocated_bytes='1234',
        gpu_reserved_bytes='2345', worker_peak_rss_bytes='345678')
    return header, bytes([12, 34, 56]) * 48
