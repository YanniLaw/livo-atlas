#!/usr/bin/env python3
"""Render one retained training camera from a fixed, finalized Gaussian map.

This diagnostic renders only the explicitly selected one or two submaps. It
neither resumes training nor supplies a consistent view of a live writer.
"""

import argparse
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import resource
import sys
import time

import numpy as np

from gaussian_map import _require
from gaussian_state import _unsigned
from joint_renderer import load_block, load_snapshot, render_blocks, verify_snapshot

_ALPHA_THRESHOLD = .05


def image_metrics(result: dict, query: dict) -> dict:
    """Measure RGB on valid image pixels and camera z on covered sparse labels."""
    image = query['image_mask']
    covered = image & (result['alpha'] >= _ALPHA_THRESHOLD)
    available_depth = image & query['depth_mask']
    depth = available_depth & covered
    image_count, covered_count, depth_count = map(int, (image.sum(), covered.sum(), depth.sum()))
    error = result['rgb'][image].astype(np.float64) - query['rgb'][image]
    mse = float(np.mean(error**2)) if image_count else None
    z_error = result['depth_m'][depth].astype(np.float64) - query['depth_m'][depth]
    return dict(image_pixels=image_count, covered_pixels=covered_count,
                alpha_threshold=_ALPHA_THRESHOLD,
                alpha_coverage=covered_count / image_count if image_count else None,
                rgb_l1=float(np.mean(np.abs(error))) if image_count else None,
                rgb_mse=mse, rgb_perfect=mse == 0 if mse is not None else False,
                rgb_psnr_db=-10 * math.log10(mse) if mse is not None and mse > 0 else None,
                depth_pixels_available=int(available_depth.sum()), depth_pixels_used=depth_count,
                depth_l1_m=float(np.mean(np.abs(z_error))) if depth_count else None,
                depth_rmse_m=float(np.sqrt(np.mean(z_error**2))) if depth_count else None)


def _new_output(directory, keyframes, output):
    output = Path(output).absolute()
    resolved = output.resolve()
    _require(not os.path.lexists(output), 'Output directory must not exist')
    _require(output.parent.is_dir(), 'Output parent directory must exist')
    for source in (directory, keyframes):
        source = Path(source).resolve(strict=True)
        _require(not resolved.is_relative_to(source) and not source.is_relative_to(resolved),
                 'Input and output directories must be separate')
    return output


def _json_bytes(document, limit):
    raw = (json.dumps(document, allow_nan=False, sort_keys=True, separators=(',', ':')) + '\n').encode()
    _require(len(raw) <= limit, 'JSON report exceeds its byte budget')
    return raw


def _exclusive_write(path, raw):
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(descriptor, 'wb') as stream:
        stream.write(raw)


def _png(path, rgb):
    from PIL import Image

    pixels = np.rint(np.clip(rgb, 0, 1) * 255).astype(np.uint8)
    with Image.fromarray(pixels, 'RGB') as image:
        image.save(path)


def _blocks_report(snapshot):
    blocks = []
    for row in snapshot.document['entries']:
        # All missing IDs are reported separately. Their unused transforms
        # would otherwise make legal sparse directories exceed 64 KiB.
        if 'asset' not in row:
            continue
        item = dict(submap_id=row['submap_id'], availability=row['availability'],
                    selected=int(row['submap_id']) in snapshot.selected_ids,
                    binding=row['binding'], asset=row['asset'],
                    gaussian_count=row['asset_gaussian_count'],
                    retained_ids=row['asset_trained_frame_ids'],
                    capture={key: row['asset_source'][key] for key in (
                        'pose_version', 'frame_count', 'optimized_frame_count', 'parent_frame',
                        'generation', 'membership_revision', 'anchor_keyframe_id', 'primary_count')})
        blocks.append(item)
    return blocks


def _sources_report(snapshot):
    digests = snapshot.fingerprint['digests']
    relative = []
    for name, digest in digests.items():
        path = Path(name)
        root, label = ((snapshot.directory, 'map') if path.is_relative_to(snapshot.directory)
                       else (snapshot.keyframes, 'keyframes'))
        relative.append((label + '/' + path.relative_to(root).as_posix(), digest))
    combined = hashlib.sha256()
    for name, digest in sorted(relative):
        # Paths cannot contain NUL. Stream the small records without making
        # mount-path length part of either the identity or a byte budget.
        combined.update(name.encode('utf-8') + b'\0' + digest.encode('ascii') + b'\n')
    result = dict(map_directory=str(snapshot.directory), keyframes=str(snapshot.keyframes),
                  map_sha256=digests[str(snapshot.directory / 'map.json')],
                  finalization_sha256=digests[str(snapshot.directory / 'finalization.json')],
                  dataset_sha256=digests[str(snapshot.keyframes / 'dataset.yml')],
                  verified_small_files=len(digests),
                  verified_path_identities=len(snapshot.fingerprint['stamps']),
                  verified_small_files_sha256=combined.hexdigest(),
                  verified_small_files_digest_format='relative-path-nul-sha256-lf-v1')
    version = snapshot.document['pose_version']
    result['pose_update_sha256'] = (digests[str(snapshot.keyframes / 'online_submaps/poses' /
                                              (version + '.yml'))] if version != '0' else None)
    return result


def _render_report(result, query):
    return dict(gaussian_count=result['gaussian_count'], gaussian_counts=result['gaussian_counts'],
                metrics=image_metrics(result, query), timings=result['timings'],
                gpu_peak_allocated_bytes=result['gpu_peak_allocated_bytes'],
                gpu_peak_reserved_bytes=result['gpu_peak_reserved_bytes'])


def run(args) -> dict:
    started = time.perf_counter()
    output = _new_output(args.map_directory, args.keyframes, args.output)
    snapshot = load_snapshot(args.map_directory, args.keyframes, args.keyframe_id, args.submap_ids)
    query = snapshot.query
    height, width = query['rgb'].shape[:2]
    _require(width * height <= 1_000_000, 'Query exceeds the pixel budget')
    validated = time.perf_counter()
    output.mkdir(mode=0o700)
    pending = output / 'report.tmp.json'
    try:
        import torch
        from online_model import checked_cuda_device

        device = checked_cuda_device()
        torch.set_num_threads(1)
        initialized = time.perf_counter()
        block = load_block(snapshot, snapshot.query_submap_id)
        single_loaded = time.perf_counter()
        single = render_blocks([block], query['K'], width, height, device)
        del block
        first_frame = time.perf_counter()
        blocks = [load_block(snapshot, submap) for submap in snapshot.selected_ids]
        selected_loaded = time.perf_counter()
        joint = render_blocks(blocks, query['K'], width, height, device)
        del blocks
        verify_snapshot(snapshot)

        _png(output / 'joint.png', joint['rgb'])
        _png(output / 'reference.png', query['rgb'])
        _png(output / 'single-reference.png', single['rgb'])
        with (output / 'render.npz').open('xb') as stream:
            np.savez(stream, rgb=joint['rgb'], alpha=joint['alpha'], depth_m=joint['depth_m'],
                     valid_mask=(joint['alpha'] >= _ALPHA_THRESHOLD).astype(np.uint8))

        document = snapshot.document
        content = [row['submap_id'] for row in document['entries'] if 'asset' in row]
        selected = [str(value) for value in snapshot.selected_ids]
        changed = np.max(np.abs(joint['rgb'] - single['rgb']), axis=2) > 1e-6
        report = dict(schema=1, kind='livo_atlas.gaussian_render', status='passed', training_view=True,
                      **{key: document[key] for key in ('session_id', 'map_sequence', 'pose_version',
                          'frame_count', 'optimized_frame_count', 'parent_frame')},
                      sources=_sources_report(snapshot), map_validation=snapshot.map_report,
                      finalization_status=snapshot.finalization['status'],
                      selected_ids=selected, content_ids=content,
                      omitted_ids=[value for value in content if value not in selected],
                      missing_ids=[row['submap_id'] for row in document['entries'] if 'asset' not in row],
                      selection_covers_all_content=set(selected) == set(content),
                      coverage_scope='selected_submaps', blocks=_blocks_report(snapshot),
                      query=dict(keyframe_id=str(args.keyframe_id),
                                 owner_submap_id=str(snapshot.query_submap_id),
                                 owner_selected=snapshot.query_submap_id in snapshot.selected_ids,
                                 calibration_id=str(query['calibration_id']), width=width, height=height,
                                 K=query['K'].tolist(), K_gsplat=joint['K_gsplat'].tolist(),
                                 camera_from_body=query['camera_from_body'].ravel().tolist(),
                                 parent_from_body=snapshot.poses[args.keyframe_id].ravel().tolist(),
                                 camera_from_parent=snapshot.camera_from_parent.ravel().tolist(),
                                 **{key: float(query[key]) for key in ('state_timestamp', 'image_timestamp',
                                     'cloud_timestamp', 'inverse_exposure_time')}),
                      joint=_render_report(joint, query), single_reference=_render_report(single, query),
                      comparison=dict(rgb_threshold=1e-6,
                                      changed_image_pixels=int((changed & query['image_mask']).sum()),
                                      interpretation='numeric change does not imply quality improvement'),
                      gpu_peak_allocated_bytes=max(single['gpu_peak_allocated_bytes'],
                                                   joint['gpu_peak_allocated_bytes']),
                      gpu_peak_reserved_bytes=max(single['gpu_peak_reserved_bytes'],
                                                  joint['gpu_peak_reserved_bytes']),
                      rss_peak_bytes=int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024,
                      timings=dict(read_validate_seconds=validated - started,
                                   cuda_setup_seconds=initialized - validated,
                                   single_load_seconds=single_loaded - initialized,
                                   first_frame_seconds=first_frame - started,
                                   selected_load_seconds=selected_loaded - first_frame),
                      software=dict(python=sys.version.split()[0], numpy=np.__version__,
                                    torch=str(torch.__version__), cuda=torch.version.cuda,
                                    gsplat=importlib.metadata.version('gsplat'),
                                    gpu=torch.cuda.get_device_name(device)),
                      budgets=dict(max_blocks=2, max_gaussians=20_000, max_pixels=1_000_000),
                      artifacts={})
        for name in ('joint.png', 'reference.png', 'single-reference.png', 'render.npz'):
            path = output / name
            report['artifacts'][name] = dict(bytes=path.stat().st_size,
                                              sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        verify_snapshot(snapshot)
        report['timings'].update(total_seconds_before_report=time.perf_counter() - started,
                                 endpoint='before_report_publication')
        _exclusive_write(pending, _json_bytes(report, 64 * 1024))
        # All artifacts are closed. This rename is a completion marker, not a
        # claim that the offline diagnostic supplied a durable map commit.
        os.rename(pending, output / 'report.json')
        return report
    except Exception as error:
        try:
            pending.unlink(missing_ok=True)
            failure = dict(schema=1, kind='livo_atlas.gaussian_render_failure', status='failed',
                           error_type=type(error).__name__, error=str(error)[:512])
            # JSON escaping can expand one non-BMP character to twelve bytes.
            # Bound the actual encoding while retaining the original text prefix.
            while True:
                try:
                    raw = _json_bytes(failure, 4096)
                    break
                except ValueError:
                    if not failure['error']:
                        raise
                    failure['error'] = failure['error'][:len(failure['error']) // 2]
            _exclusive_write(output / 'failure.json', raw)
        except (OSError, ValueError):
            # Best-effort diagnostics must never replace the rendering exception.
            pass
        raise


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-directory', type=Path, required=True)
    parser.add_argument('--keyframes', type=Path, required=True)
    parser.add_argument('--keyframe-id', type=_unsigned, required=True)
    parser.add_argument('--submap-ids', type=_unsigned, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        run(args)
    except Exception as error:
        print(f'{type(error).__name__}: {error}', file=sys.stderr)
        return 1
    print(f'Render report: {args.output / "report.json"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
