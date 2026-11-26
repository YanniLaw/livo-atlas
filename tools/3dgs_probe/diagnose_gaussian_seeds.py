"""Trace one retained seed cohort without changing the online trainer."""

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import random
import shutil
import sys
from tempfile import TemporaryDirectory

import numpy as np
import torch

from evaluate_gaussian_sampling import admit_training_view, frozen_groups
from evaluate_gaussian_submaps import camera_from_final_parent, prepare_group
from evaluate_heldout_snapshot import initialize_cuda, load_settings
from gaussian_map import _require
from gaussian_state import _open_file
from heldout_query import load_evaluation_snapshot, verify_evaluation_snapshot
from holdout_split import HoldoutSplit, MAX_SPLIT_BYTES
from online_model import OnlineGaussianModel, _MAX_PER_VIEW, _MAX_VIEWS
from prepare_holdout_split import file_sha
from render_gaussian_map import _exclusive_write, _json_bytes, _new_output
from train import _depth_candidates, _evenly_spaced, _seed_pixels
from worker_config import load_worker_config


_MAX_OUTPUT_BYTES = 64 * 1024**2


def _cpu(value):
    return value.detach().cpu().numpy().copy() if torch.is_tensor(value) else np.asarray(value).copy()


def seed_features(view, pixels, history):
    """Sparse geometric support is a proxy; missing labels remain unobserved."""
    pixels = np.asarray(pixels)
    mask = np.asarray(view['image_mask'], dtype=bool)
    _require(view['split'] == 'train' and all(v['split'] == 'train' for v in history),
             'Seed diagnosis requires only training observations')
    _require(pixels.ndim == 1 and pixels.dtype.kind in 'iu' and 0 < len(pixels) <= 2000
             and np.all(pixels >= 0) and np.all(pixels < mask.size)
             and np.all(np.diff(pixels.astype('i8')) > 0), 'Invalid diagnostic pixels')
    _require(_depth_candidates(view).ravel()[pixels].all(), 'Seeds require valid observed depth')
    y, x = np.divmod(pixels, mask.shape[1])
    z = np.asarray(view['depth_m'])[y, x].astype('f8')
    k = view['K']
    camera = np.column_stack(((x-k[0, 2])*z/k[0, 0], (y-k[1, 2])*z/k[1, 1], z, np.ones(len(z))))
    points = (camera @ np.linalg.inv(view['camera_from_parent']).T)[:, :3]
    rgb = np.asarray(view['rgb'])
    gray = rgb @ np.array([.2126, .7152, .0722])
    gradients, valid = [], mask.copy()
    for axis in (0, 1):
        gradients.append(np.gradient(gray, axis=axis) if gray.shape[axis] > 1 else np.zeros_like(gray))
        if mask.shape[axis] > 1:
            # A masked neighbor must not manufacture a high texture gradient.
            before = np.take(mask, np.maximum(np.arange(mask.shape[axis])-1, 0), axis=axis)
            after = np.take(mask, np.minimum(np.arange(mask.shape[axis])+1, mask.shape[axis]-1), axis=axis)
            valid &= before & after
    result = dict(points=points, gradient=np.hypot(*gradients)[y, x], gradient_valid=valid[y, x])
    for field in ('visible_views', 'depth_checks', 'depth_support', 'occluded', 'in_front'):
        result[field] = np.zeros(len(pixels), dtype='i4')
    result['color_error_sum'] = np.zeros(len(pixels), dtype='f8')
    for other in history:
        height, width = other['image_mask'].shape
        xyz = points @ other['camera_from_parent'][:3, :3].T + other['camera_from_parent'][:3, 3]
        positive = xyz[:, 2] > 1e-6
        safe_z = np.where(positive, xyz[:, 2], 1.)
        intrinsic = other['K']
        u = intrinsic[0, 0]*xyz[:, 0]/safe_z + intrinsic[0, 2]
        v = intrinsic[1, 1]*xyz[:, 1]/safe_z + intrinsic[1, 2]
        ids = np.flatnonzero(positive & np.isfinite(u) & np.isfinite(v)
                            & (u >= 0) & (u <= width-1) & (v >= 0) & (v <= height-1))
        xx, yy = np.rint(u[ids]).astype('i8'), np.rint(v[ids]).astype('i8')
        good = other['image_mask'][yy, xx].astype(bool)
        result['visible_views'][ids[good]] += 1
        good &= other['depth_mask'][yy, xx].astype(bool)
        ids, xx, yy = ids[good], xx[good], yy[good]
        observed = other['depth_m'][yy, xx]
        error = xyz[ids, 2] - observed
        tolerance = np.maximum(.05, .02*observed)
        consistent = np.abs(error) <= tolerance
        result['depth_checks'][ids] += 1
        result['depth_support'][ids[consistent]] += 1
        result['occluded'][ids[error > tolerance]] += 1
        result['in_front'][ids[error < -tolerance]] += 1
        color_error = np.abs(other['rgb'][yy, xx]-rgb[y[ids], x[ids]]).mean(axis=1)
        result['color_error_sum'][ids[consistent]] += color_error[consistent]
    _require(all(np.isfinite(value).all() for value in result.values()), 'Nonfinite diagnostic feature')
    return result


def cohort_summary(features, selected, uniform):
    result = {}
    for name, mask in dict(uniform_kept=uniform & selected, uniform_omitted=uniform & ~selected,
                           added=selected & ~uniform).items():
        gradient = features['gradient'][mask & features['gradient_valid']]
        checks = int(features['depth_checks'][mask].sum())
        support = int(features['depth_support'][mask].sum())
        result[name] = dict(count=int(mask.sum()), gradient_samples=len(gradient),
            gradient_mean=float(gradient.mean()) if len(gradient) else None,
            visible_views=int(features['visible_views'][mask].sum()), depth_checks=checks,
            depth_support=support, depth_support_ratio=support/checks if checks else None,
            occluded=int(features['occluded'][mask].sum()), in_front=int(features['in_front'][mask].sum()),
            photometric_l1=float(features['color_error_sum'][mask].sum()/support) if support else None)
    return result


def _statistics(value):
    _require(value.ndim == 1 and len(value) > 0 and np.isfinite(value).all(), 'Invalid parameter samples')
    return dict(mean=float(value.mean()), p50=float(np.median(value)),
                p95=float(np.quantile(value, .95)), maximum=float(value.max()))


def parameter_changes(initial, current):
    for name, shape in (('means', (3,)), ('log_scales', (3,)), ('color_logits', (3,)), ('opacity_logits', ())):
        a, b = initial[name], current[name]
        _require(a.shape == b.shape and a.ndim == 1+len(shape) and a.shape[1:] == shape
                 and len(a) > 0 and np.isfinite(a).all() and np.isfinite(b).all(), 'Unaligned seed parameters')
    sigmoid = lambda x: 1/(1+np.exp(-x.astype('f8')))
    return dict(position_l2_m=_statistics(np.linalg.norm(current['means']-initial['means'], axis=1)),
        scale_ratio=_statistics(np.exp(current['log_scales']-initial['log_scales']).mean(axis=1)),
        color_l1=_statistics(np.abs(sigmoid(current['color_logits'])-sigmoid(initial['color_logits'])).mean(axis=1)),
        opacity_delta=_statistics(sigmoid(current['opacity_logits'])-sigmoid(initial['opacity_logits'])))


def validate_target(group, tracked_id):
    ids = group['train_ids']
    _require(0 < len(ids)+len(group['held_out_ids']) <= 32 and type(tracked_id) is int
             and tracked_id in ids and tracked_id in ids[-_MAX_VIEWS:],
             'Tracked frame must be a training observation retained in the final window')


class TraceModel(OnlineGaussianModel):
    """Offline observation only: no additional renders, gradients or RNG calls."""

    def __init__(self, device, tracked_id, *, options):
        super().__init__(device, options=options)
        self.tracked_id = tracked_id
        self.uncovered = None
        self.prepass_available = False
        self.selected_pixels = None
        self.pixels = None
        self.features = None
        self.uniform = None
        self.selected = None
        self.stages = []

    def _coverage_prepass(self, view):
        mask, evidence = super()._coverage_prepass(view)
        if view['keyframe_id'] == self.tracked_id:
            self.uncovered = None if mask is None else mask.copy()
            self.prepass_available = mask is not None
        return mask, evidence

    def _append(self, view, initial):
        tracked = view['keyframe_id'] == self.tracked_id
        if tracked:
            mask = _depth_candidates(view)
            uniform = _evenly_spaced(np.flatnonzero(mask), min(int(mask.sum()), _MAX_PER_VIEW))
            selected = _seed_pixels(mask, _MAX_PER_VIEW, self.uncovered,
                                   self.options.coverage_uncovered_ratio, self.options.coverage_sampling)
            self.selected_pixels = selected.copy()
            self.pixels = np.union1d(uniform, selected)
            self.uniform, self.selected = np.isin(self.pixels, uniform), np.isin(self.pixels, selected)
            # Coverage also discards the oldest model before measuring support.
            history = self.views[1:] if len(self.views) == _MAX_VIEWS else self.views
            history = [{key: _cpu(value) if torch.is_tensor(value) else value for key, value in row.items()}
                       for row in history]
            self.features = seed_features(view, self.pixels, history)
            _require(np.allclose(self.features['points'][self.selected], initial['means'], atol=1e-6, rtol=1e-6),
                     'Selected pixels do not match actual initialized seeds')
        super()._append(view, initial)
        if tracked:
            self._capture(-1)

    def _capture(self, frame_id):
        selected = self.origin_ids == self.tracked_id
        _require(int(selected.sum()) == len(self.selected_pixels), 'Tracked seeds evicted or misaligned')
        values = {name: _cpu(value[selected]) for name, value in self.parameters.items() if name != 'quats'}
        _require(len(self.stages) < 11, 'Diagnostic snapshot limit exceeded')
        self.stages.append((frame_id, values))

    def train_candidate(self, view, *, seed_policy):
        result = super().train_candidate(view, seed_policy=seed_policy)
        if self.selected_pixels is not None:
            self._capture(view['keyframe_id'])
        return result


def trace_summary(arrays):
    features = {key: arrays[key] for key in ('points', 'gradient', 'gradient_valid', 'visible_views',
        'depth_checks', 'depth_support', 'occluded', 'in_front', 'color_error_sum')}
    result = dict(cohorts=cohort_summary(features, arrays['selected'], arrays['uniform']), stages=[])
    for index, frame_id in enumerate(arrays['stage_ids'][1:], 1):
        stage = dict(after_keyframe_id=int(frame_id), cohorts={})
        for name, mask in dict(uniform_kept=arrays['uniform'][arrays['selected']],
                               added=~arrays['uniform'][arrays['selected']]).items():
            stage['cohorts'][name] = parameter_changes(
                {key: arrays[key][0, mask] for key in ('means','log_scales','color_logits','opacity_logits')},
                {key: arrays[key][index, mask] for key in ('means','log_scales','color_logits','opacity_logits')}) if mask.any() else None
        result['stages'].append(stage)
    return result


def run(args):
    output = _new_output(args.map_directory, args.keyframes, args.output)
    _require(type(args.random_seed) is int and 0 <= args.random_seed < 2**32, 'Invalid random seed')
    settings = load_settings(args.config)
    raw_config = Path(args.config).read_bytes()
    _require(hashlib.sha256(raw_config).hexdigest() == settings.config_sha256, 'Config changed')
    snapshot = load_evaluation_snapshot(args.map_directory, args.keyframes)
    if settings.split_file:
        with _open_file(Path(settings.split_file), MAX_SPLIT_BYTES) as stream:
            _require(HoldoutSplit(stream.read()).split_id == snapshot.split.split_id, 'Configured split differs')
    groups = frozen_groups(snapshot)
    _require(type(args.submap_id) is int and 0 <= args.submap_id < len(groups), 'Unknown submap')
    group = groups[args.submap_id]
    validate_target(group, args.tracked_id)
    preparer = args.prepare_executable.resolve(strict=True)
    preparer_hash = file_sha(preparer)
    maximum = min(_MAX_OUTPUT_BYTES, settings.max_output_bytes)
    _require(maximum >= 2*1024**2, 'Diagnostic output budget requires at least 2 MiB')
    reserve = max(settings.min_free_disk_bytes, 1024**3)
    scratch = sum(f['width']*f['height']*32+128*1024 for f in snapshot.split.frames.values()
                  if int(f['submap_id']) == args.submap_id)
    _require(shutil.disk_usage(output.parent).free >= scratch+maximum+reserve,
             'Insufficient diagnostic disk reserve')
    output.mkdir(mode=0o700)
    try:
        _exclusive_write(output/'config.yaml', raw_config)
        config = load_worker_config(output/'config.yaml')
        _require(config.options == settings.worker, 'Archived training options differ')
        reference_held = next(key for key, frame in snapshot.split.frames.items() if frame['role']=='heldout')
        prepared = dict(group, held_out_ids=group['held_out_ids'] or [reference_held])
        with TemporaryDirectory(prefix='.prepare-', dir=output) as temporary:
            views = prepare_group(preparer, snapshot.keyframes, Path(temporary)/'bundle', prepared,
                                  snapshot.split.source['camera_sha256'])
        training = [admit_training_view(snapshot.split, views[key]) for key in group['train_ids']]
        random.seed(args.random_seed)
        np.random.seed(args.random_seed)
        device, software = initialize_cuda(settings.worker)
        torch.manual_seed(args.random_seed)
        torch.cuda.manual_seed_all(args.random_seed)
        model = TraceModel(device, args.tracked_id, options=settings.worker)
        model.reset(1)
        anchor = snapshot.poses[group['anchor_id']]
        receipts = []
        for view in training:
            camera = camera_from_final_parent(view, snapshot.poses[view['keyframe_id']])
            local = dict(view, camera_from_parent=camera @ anchor, generation='1')
            _, metrics = model.train_candidate(local, seed_policy=config.seed_policy)
            _require(metrics['steps'] == 20 and len(metrics['training_frame_ids']) == 20
                     and set(metrics['training_frame_ids']) <= set(group['train_ids']), 'Training lineage mismatch')
            receipts.append(dict(keyframe_id=view['keyframe_id'], **metrics))
            model.decide(True)
        _require(model.selected_pixels is not None, 'Tracked frame was not initialized')
        arrays = dict(model.features, pixels=model.pixels, selected=model.selected, uniform=model.uniform,
            selected_pixels=model.selected_pixels, stage_ids=np.array([key for key, _ in model.stages], dtype='i8'))
        arrays['uncovered'] = (model.uncovered.ravel()[model.pixels] if model.prepass_available
                               else np.zeros(len(model.pixels), bool))
        for name in ('means','log_scales','color_logits','opacity_logits'):
            arrays[name] = np.stack([row[name] for _, row in model.stages])
        _require(sum(value.nbytes for value in arrays.values()) < maximum-1024**2, 'Trace exceeds budget')
        np.savez_compressed(output/'trace.npz', **arrays)
        _require((output/'trace.npz').stat().st_size < maximum-1024**2, 'Trace exceeds disk budget')
        report = dict(schema=1, kind='livo_atlas.gaussian_seed_diagnostic', status='succeeded',
            submap_id=args.submap_id, tracked_id=args.tracked_id, random_seed=args.random_seed,
            seed_policy=config.seed_policy, training_options=asdict(settings.worker), software=software,
            split_id=snapshot.split.split_id, reference_map_sha256=file_sha(snapshot.directory/'map.json'),
            final_poses_sha256=hashlib.sha256(np.asarray(snapshot.poses,dtype='<f8').tobytes()).hexdigest(),
            source_sha256=snapshot.source_digests, config_sha256=settings.config_sha256,
            preparer_sha256=preparer_hash, prepass_available=model.prepass_available,
            training=receipts, retained_frame_ids=model.frame_ids, summary=trace_summary(arrays),
            artifacts={'trace.npz':dict(bytes=(output/'trace.npz').stat().st_size,sha256=file_sha(output/'trace.npz'))})
        verify_evaluation_snapshot(snapshot)
        _require(file_sha(args.config)==file_sha(output/'config.yaml')==settings.config_sha256
                 and file_sha(preparer)==preparer_hash,
                 'Configuration or preparer changed')
        payload = _json_bytes(report, 1024**2)
        _require((output/'trace.npz').stat().st_size+len(raw_config)+len(payload) <= maximum,
                 'Diagnostic output budget exceeded')
        _exclusive_write(output/'report.json', payload)
        return report
    except BaseException as error:
        if not (output/'report.json').exists():
            _exclusive_write(output/'failure.json', _json_bytes(dict(status='failed', error=str(error)), 65536))
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('map-directory','keyframes','prepare-executable','config','output'):
        parser.add_argument('--'+name, type=Path, required=True)
    for name in ('submap-id','tracked-id','random-seed'):
        parser.add_argument('--'+name, type=int, required=True)
    try:
        run(parser.parse_args())
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f'{type(error).__name__}: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
