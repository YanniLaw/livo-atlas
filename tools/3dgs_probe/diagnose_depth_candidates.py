"""Compare source depth support with captured seeds without training or rendering."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import sys
from tempfile import TemporaryDirectory
from zipfile import ZipFile

import numpy as np

from bundle import _pose
from gaussian_coverage import FIELDS, RADII, _query_masks, projected_support
from gaussian_map import _require
from gaussian_state import _open_file, _reject_constant, _unique_object
from evaluate_gaussian_sampling import admit_training_view
from evaluate_gaussian_submaps import camera_from_final_parent, prepare_group
from evaluate_heldout_snapshot import load_settings
from heldout_query import canonical_query, load_evaluation_snapshot
from heldout_selection import EvaluationSelector
from prepare_holdout_split import file_sha
from render_gaussian_map import _exclusive_write, _json_bytes, _new_output, _sources_report
from train import _seed_pixels


COHORTS = ('input_all', 'input_retained', 'all_initial', 'retained_initial')
CONTRASTS = dict(sampling_all=('input_all', 'all_initial'),
                 sampling_retained=('input_retained', 'retained_initial'),
                 input_retention=('input_all', 'input_retained'),
                 seed_retention=('all_initial', 'retained_initial'))


def candidate_points(view, camera):
    """Use the initializer's float64 backprojection and float32 storage frame."""
    _require(view['split'] == 'train', 'Candidates require a training view')
    image, depth_mask = (np.asarray(view[k]) for k in ('image_mask', 'depth_mask'))
    depth, k = np.asarray(view['depth_m']), np.asarray(view['K'], dtype='f8')
    _require(image.ndim == 2 and 0 < image.size <= 1_000_000 and depth_mask.shape == image.shape and
             depth.shape == image.shape and depth.dtype.kind == 'f' and np.isfinite(depth).all(),
             'Invalid candidate depth dimensions')
    for mask in (image, depth_mask):
        _require(mask.dtype in (np.dtype(bool), np.dtype('u1')) and np.all(mask <= 1), 'Invalid candidate mask')
    _require(k.shape == (3, 3) and np.isfinite(k).all() and k[0, 0] > 0 and k[1, 1] > 0 and
             np.allclose(k[2], [0, 0, 1], atol=1e-12, rtol=0) and
             abs(k[0, 1]) <= 1e-12 and abs(k[1, 0]) <= 1e-12, 'Invalid candidate intrinsics')
    pixels = np.flatnonzero(image.astype(bool) & depth_mask.astype(bool))
    y, x = np.divmod(pixels, image.shape[1])
    z = depth[y, x].astype('f8')
    _require(np.all(z > 0), 'Candidate depth must be positive')
    xyz = np.column_stack(((x-k[0, 2])*z/k[0, 0], (y-k[1, 2])*z/k[1, 1], z, np.ones(len(z))))
    return (xyz @ np.linalg.inv(_pose(camera)).T)[:, :3].astype('f4')


def accumulate(points, camera, k, query):
    """Keep projection workspace bounded independently of the observation size."""
    image, _, _ = _query_masks(query)
    points = np.asarray(points)
    _require(points.ndim == 2 and points.shape[1] == 3 and len(points) <= 1_000_000 and
             points.dtype.kind == 'f' and np.isfinite(points).all(), 'Invalid candidate points')
    result = {}
    for radius in RADII:
        maps = {key: np.zeros(image.shape, dtype='i4') for key in FIELDS}
        # Also validate cameras/intrinsics for an empty observation.
        for first in range(0, max(1, len(points)), 64000):
            chunk = projected_support(points[first:first+64000], camera, k, query, radius)
            for key in FIELDS:
                maps[key] += chunk[key]
        result[radius] = maps
    return result


def candidate_summary(query, alpha, support):
    image, labels, _ = _query_masks(query)
    alpha = np.asarray(alpha)
    _require(alpha.shape == image.shape and alpha.dtype.kind == 'f' and np.isfinite(alpha).all() and
             np.all(alpha >= 0) and np.all(alpha <= 1+1e-6), 'Invalid diagnostic alpha')
    holes, known = image & (alpha < .05), labels & (alpha < .05)
    n = lambda mask: int(mask.sum())
    result = dict(image_pixels=n(image), reference_pixels=n(labels), holes=n(holes),
                  hole_depth_known=n(known), hole_depth_unknown=n(holes & ~labels), support={}, contrasts={})
    for cohort in COHORTS:
        result['support'][cohort] = {}
        for radius in RADII:
            maps = support[cohort][radius]
            _require(set(maps) == set(FIELDS) and all(v.shape == image.shape and v.dtype.kind in 'iu' and
                     np.all(v >= 0) for v in maps.values()), 'Invalid candidate count maps')
            _require(np.all((maps['consistent']+maps['front']+maps['behind'])[labels] == maps['any'][labels]),
                     'Candidate depth classes do not partition support')
            c, f, b = (maps[key] > 0 for key in ('consistent', 'front', 'behind'))
            classes = dict(consistent=n(known & c), front_only=n(known & ~c & f & ~b),
                           behind_only=n(known & ~c & ~f & b), mixed=n(known & ~c & f & b),
                           none=n(known & (maps['any'] == 0)))
            _require(sum(classes.values()) == n(known), 'Candidate hole classes do not partition')
            result['support'][cohort][str(radius)] = dict(holes_near_centers=n(holes & (maps['any'] > 0)),
                image_pixels_near_centers=n(image & (maps['any'] > 0)), labelled_holes=classes)
    for name, (parent, child) in CONTRASTS.items():
        result['contrasts'][name] = {}
        for radius in RADII:
            a, b = support[parent][radius], support[child][radius]
            _require(all(np.all(a[key] >= b[key]) for key in FIELDS),
                     f'{child} is not a subset of {parent}; check input lineage or projection rounding')
            result['contrasts'][name][str(radius)] = dict(
                holes_losing_neighbors=n(holes & (a['any'] > 0) & (b['any'] == 0)),
                known_holes_losing_consistent_neighbors=n(known & (a['consistent'] > 0) & (b['consistent'] == 0)))
    return result


def _read_report(path):
    with _open_file(path, 2*1024**2) as stream:
        return json.loads(stream.read(), object_pairs_hook=_unique_object, parse_constant=_reject_constant)


def _receipt(path):
    return dict(bytes=path.stat().st_size, sha256=file_sha(path))


def _add_maps(destination, source):
    for radius in RADII:
        for key in FIELDS:
            destination[radius][key] += source[radius][key]


def run(args):
    root = Path(args.diagnostic_directory).resolve(strict=True)
    output = _new_output(args.map_directory, args.keyframes, args.output)
    _require(not output.resolve().is_relative_to(root) and not root.is_relative_to(output.resolve()),
             'Input diagnostic and output must be separate')
    report = _read_report(root/'report.json')
    report_receipt = _receipt(root/'report.json')
    _require(report['schema'] == 1 and report['kind'] == 'livo_atlas.gaussian_coverage_diagnostic' and
             report['status'] == 'succeeded' and report['selection_policy'] == 'spatial' and
             report['seed_policy'] == 'coverage', 'Requires a successful native spatial coverage diagnostic')
    snapshot = load_evaluation_snapshot(args.map_directory, args.keyframes)
    _require(report['sources'] == _sources_report(snapshot) and report['source_sha256'] == snapshot.source_digests and
             report['split_id'] == snapshot.split.split_id, 'Diagnostic source binding differs')
    settings = load_settings(root/'config.yaml')
    _require(settings.policy == 'spatial' and report['training_options'] == asdict(settings.worker),
             'Diagnostic training settings differ')
    keyframe = report['keyframe_id']
    _require(type(keyframe) is int and keyframe in snapshot.split.frames, 'Invalid diagnostic query')
    frame = snapshot.split.frames[keyframe]
    owner = int(frame['submap_id'])
    _require(frame['role'] == 'heldout' and frame['query'] and owner == report['owner_submap_id'],
             'Diagnostic query is not the heldout owner')
    preparer = args.prepare_executable.resolve(strict=True)
    _require(file_sha(preparer) == report['preparer_sha256'], 'Diagnostic preparer differs')
    selector = EvaluationSelector(snapshot, 'spatial', args.selector_executable)
    selection = selector.select(frame)
    if selection['selection'] is not None:
        selection['selection'].pop('selection_seconds', None)
    ids = selection['selected_ids']
    _require(ids == report['selected_ids'] and ids == sorted(set(ids)) and 1 <= len(ids) <= 2 and
             ids[-1] == owner and selection == report['selection'] and
             selector.runtime_sha256 == report['selector_runtime_sha256'], 'Diagnostic native selection differs')
    modes = dict(single=[owner])
    if len(ids) == 2:
        modes['joint'] = ids
    expected = {'samples.npz', 'config.yaml', 'split.json', 'poses.f64le', *[mode+'.png' for mode in modes]}
    _require(set(report['artifacts']) == expected and
             all(type(row['bytes']) is int and 0 < row['bytes'] <= 64*1024**2 for row in report['artifacts'].values()) and
             sum(row['bytes'] for row in report['artifacts'].values()) <= 64*1024**2,
             'Invalid diagnostic artifact budget')

    def verify_inputs():
        selector.verify()
        _require(_receipt(root/'report.json') == report_receipt and file_sha(preparer) == report['preparer_sha256'],
                 'Diagnostic input changed')
        for name, receipt in report['artifacts'].items():
            with _open_file(root/name, receipt['bytes']):
                pass
            _require(_receipt(root/name) == receipt, 'Diagnostic artifact changed: '+name)
        _require(file_sha(root/'config.yaml') == report['config_sha256'] and
                 file_sha(root/'poses.f64le') == report['final_poses_sha256'], 'Diagnostic configuration or poses differ')
        with _open_file(root/'split.json', len(snapshot.split.payload)) as stream:
            _require(stream.read() == snapshot.split.payload, 'Diagnostic split differs')
        raw_poses = np.asarray(snapshot.poses, dtype='<f8').tobytes()
        with _open_file(root/'poses.f64le', len(raw_poses)) as stream:
            _require(stream.read() == raw_poses, 'Diagnostic final poses differ')

    verify_inputs()
    # Bound decompression as well as disk bytes before NumPy materializes arrays.
    with _open_file(root/'samples.npz', 64*1024**2) as stream, ZipFile(stream) as archive:
        _require(sum(row.file_size for row in archive.infolist()) <= 128*1024**2, 'Diagnostic arrays exceed budget')
    with _open_file(root/'samples.npz', 64*1024**2) as stream, np.load(stream, allow_pickle=False) as data:
        arrays = {key: data[key] for key in data.files}
    query = dict(image_mask=arrays['image_mask'], depth_mask=arrays['depth_mask'], depth_m=arrays['reference_depth_m'])
    image, _, _ = _query_masks(query)
    _require(image.shape == (frame['height'], frame['width']), 'Diagnostic query dimensions differ')
    maximum = min(64*1024**2, settings.max_output_bytes)
    reserve = max(1024**3, settings.min_free_disk_bytes)
    # Persist only input maps per block; seed maps are reproducible from the old archive.
    predicted = len(ids)*2*len(RADII)*len(FIELDS)*image.size*4 + 2*1024**2
    _require(predicted <= maximum, 'Candidate output budget is insufficient')
    groups = []
    _require([row['submap_id'] for row in report['groups']] == ids, 'Diagnostic groups differ')
    for evidence in report['groups']:
        key = evidence['submap_id']
        primary = [(k, f) for k, f in snapshot.split.frames.items() if int(f['submap_id']) == key]
        train = [k for k, f in primary if f['role'] == 'train']
        held = [k for k, f in primary if f['role'] == 'heldout'] or [keyframe]
        anchor = int(snapshot.document['entries'][key]['anchor_keyframe_id'])
        _require(0 < len(primary) <= 32 and train and evidence['anchor_id'] == anchor and
                 [row['keyframe_id'] for row in evidence['training']] == train and
                 evidence['retained_frame_ids'] == train[-10:], 'Diagnostic training lineage differs')
        _require(np.array_equal(arrays[f'block_{key}_anchor'], snapshot.poses[anchor]), 'Diagnostic anchor differs')
        groups.append(dict(submap_id=key, anchor_id=anchor, frame_count=len(snapshot.poses), train_ids=train, held_out_ids=held))
    scratch = max(sum(snapshot.split.frames[k]['width']*snapshot.split.frames[k]['height']*32+128*1024
                      for k in group['train_ids']+group['held_out_ids']) for group in groups)
    _require(shutil.disk_usage(output.parent).free >= scratch+maximum+reserve, 'Insufficient candidate preparation reserve')
    output.mkdir(mode=0o700)
    try:
        blocks, saved, evidence_rows = {}, {}, []
        for group, evidence in zip(groups, report['groups']):
            _require(shutil.disk_usage(output).free >= scratch+maximum+reserve, 'Insufficient candidate preparation reserve')
            with TemporaryDirectory(prefix='.prepare-', dir=output) as temporary:
                views = prepare_group(preparer, snapshot.keyframes, Path(temporary)/'bundle', group, snapshot.split.source['camera_sha256'])
            if keyframe in views:
                fresh = canonical_query(snapshot.split, views[keyframe])
                for saved_key, fresh_key in (('reference_rgb', 'rgb'), ('reference_depth_m', 'depth_m'),
                        ('image_mask', 'image_mask'), ('depth_mask', 'depth_mask'), ('K', 'K')):
                    _require(np.array_equal(arrays[saved_key], fresh[fresh_key]), 'Diagnostic query values differ')
                camera = fresh['camera_from_body'] @ np.linalg.inv(snapshot.poses[keyframe])
                _require(np.array_equal(arrays['camera_from_parent'], camera), 'Diagnostic query camera differs')
            key = group['submap_id']
            prefix = f'block_{key}_'
            camera = arrays['camera_from_parent'] @ arrays[prefix+'anchor']
            empty = lambda: {r: {field: np.zeros(image.shape, dtype='i4') for field in FIELDS} for r in RADII}
            support = dict(input_all=empty(), input_retained=empty())
            point_counts, selected_means, origins = [], [], []
            for frame_id, training in zip(group['train_ids'], evidence['training']):
                view = admit_training_view(snapshot.split, views[frame_id])
                local_camera = camera_from_final_parent(view, snapshot.poses[frame_id]) @ snapshot.poses[group['anchor_id']]
                points = candidate_points(view, local_camera)
                _require(len(points) == training['seed_selection']['depth_candidates'], 'Diagnostic candidate count differs')
                mask = view['image_mask'] & view['depth_mask']
                packed = arrays[prefix+f'prepass_{frame_id}']
                _require(packed.dtype == np.dtype('u1') and packed.ndim == 1 and
                         len(packed) in (0, (mask.size+7)//8), 'Diagnostic prepass shape differs')
                uncovered = np.unpackbits(packed, bitorder='little')[:mask.size].reshape(mask.shape).astype(bool) if len(packed) else None
                pixels = _seed_pixels(mask, 1000, uncovered, settings.worker.coverage_uncovered_ratio, settings.worker.coverage_sampling)
                index = np.searchsorted(np.flatnonzero(mask), pixels)
                selected_means.append(points[index])
                origins.append(np.full(len(pixels), frame_id, dtype='i8'))
                counts = accumulate(points, camera, arrays['K'], query)
                _add_maps(support['input_all'], counts)
                retained = frame_id in evidence['retained_frame_ids']
                if retained:
                    _add_maps(support['input_retained'], counts)
                point_counts.append(dict(keyframe_id=frame_id, candidates=len(points), sampled=len(pixels), retained=retained))
            sampled = np.concatenate(selected_means)
            origin = np.concatenate(origins)
            captured = arrays[prefix+'all_initial_means']
            _require(captured.dtype == np.dtype('f4') and captured.shape == sampled.shape and
                     np.allclose(captured, sampled, atol=2e-5, rtol=0) and
                     np.array_equal(arrays[prefix+'all_origin_ids'], origin), 'Diagnostic sampled seed lineage differs')
            keep = np.isin(origin, evidence['retained_frame_ids'])
            _require(np.array_equal(arrays[prefix+'origin_ids'], origin[keep]) and
                     np.array_equal(arrays[prefix+'initial_means'], captured[keep]), 'Diagnostic retained seed lineage differs')
            for cohort, points in (('all_initial', captured), ('retained_initial', captured[keep])):
                support[cohort] = accumulate(points, camera, arrays['K'], query)
            blocks[key] = support
            for cohort in ('input_all', 'input_retained'):
                for radius in RADII:
                    for field in FIELDS:
                        saved[prefix+cohort+f'_r{radius}_'+field] = support[cohort][radius][field]
            evidence_rows.append(dict(submap_id=key, frames=point_counts, candidates=sum(row['candidates'] for row in point_counts),
                retained_candidates=sum(row['candidates'] for row in point_counts if row['retained']), sampled=len(sampled), retained_sampled=int(keep.sum()),
                seed_reconstruction_max_abs_m=float(np.max(np.abs(captured-sampled)))))
        summaries = {}
        for mode, selected in modes.items():
            combined = {cohort: {r: {field: sum((blocks[key][cohort][r][field] for key in selected),
                start=np.zeros(image.shape, dtype='i4')) for field in FIELDS} for r in RADII} for cohort in COHORTS}
            summaries[mode] = candidate_summary(query, arrays[mode+'_trained_alpha'], combined)
        _require(sum(v.nbytes for v in saved.values())+2*1024**2 <= maximum, 'Candidate raw output budget exceeded')
        _require(shutil.disk_usage(output).free >= maximum+reserve, 'Insufficient candidate output reserve')
        np.savez_compressed(output/'counts.npz', **saved)
        artifacts = {'counts.npz': _receipt(output/'counts.npz')}
        result = dict(schema=1, kind='livo_atlas.depth_candidate_diagnostic', status='succeeded',
            diagnostic_directory=str(root), diagnostic_report=report_receipt, diagnostic_artifacts=report['artifacts'],
            keyframe_id=keyframe, owner_submap_id=owner, selected_ids=ids, random_seed=report['random_seed'],
            selection=selection, selector_runtime_sha256=selector.runtime_sha256, preparer_sha256=report['preparer_sha256'],
            sources=report['sources'], source_sha256=report['source_sha256'], split_id=report['split_id'],
            new_training_calls=0, new_render_calls=0, radii=list(RADII), depth_tolerance='max(.05m,.02z)',
            groups=evidence_rows, summary=summaries, artifacts=artifacts)
        payload = _json_bytes(result, 2*1024**2)
        _require(sum(row['bytes'] for row in artifacts.values())+len(payload) <= maximum, 'Candidate output budget exceeded')
        verify_inputs()
        _require(shutil.disk_usage(output).free >= len(payload)+65536+reserve, 'Insufficient candidate report reserve')
        pending = output/'.report.pending.json'
        _exclusive_write(pending, payload)
        with _open_file(pending, len(payload)) as stream:
            _require(stream.read() == payload, 'Pending candidate report changed')
        verify_inputs()
        _require(_receipt(output/'counts.npz') == artifacts['counts.npz'], 'Candidate artifact changed')
        _require(shutil.disk_usage(output).free >= reserve, 'Insufficient candidate reserve after report write')
        pending.replace(output/'report.json')
        return result
    except BaseException as error:
        if not (output/'report.json').exists():
            _exclusive_write(output/'failure.json', _json_bytes(dict(status='failed', error=str(error)), 65536))
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('diagnostic-directory', 'map-directory', 'keyframes', 'prepare-executable', 'selector-executable', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    try:
        run(parser.parse_args(argv))
    except Exception as error:
        print(f'{type(error).__name__}: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
