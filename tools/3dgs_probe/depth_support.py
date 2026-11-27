"""LiDAR neighborhood support and paired depth statistics for frozen views."""

import math

import numpy as np

from gaussian_map import _project, _require, _yaml
from gaussian_state import _open_file

NEIGHBOR_OFFSETS = (-2, -1, 1, 2)
RADII = (2, 4)
MINIMUM_VOTES = 2
MINIMUM_TOLERANCE_M = .20
RELATIVE_TOLERANCE = .03
VOTE_FIELDS = ('available', 'near', 'match', 'far')
GROUPS = ('reference', 'competing', 'near_only', 'no_consensus', 'sparse')
_MAX_POINTS = 100_000
_HEADER_BYTES = 8192
_POINT_FIELDS = 'x y z intensity normal_x normal_y normal_z curvature'.split()
_SUM_FIELDS = ('signed_sum_m', 'absolute_sum_m', 'squared_sum_m2')
_CHANGE_FIELDS = ('increase_m2', 'decrease_m2')


def read_body_cloud(path):
    """Read the byte layout written by KeyframeStore's binary PCD exporter."""
    with _open_file(path, _HEADER_BYTES + _MAX_POINTS*32) as stream:
        header = {}
        while stream.tell() < _HEADER_BYTES:
            raw = stream.readline(512)
            _require(raw.endswith(b'\n'), 'Invalid PCD header line')
            line = raw.decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            field, *values = line.split()
            _require(field not in header, 'Duplicate PCD header field')
            header[field] = values
            if field == 'DATA':
                break
        _require(stream.tell() <= _HEADER_BYTES and header.get('DATA') == ['binary'] and header.get('FIELDS') == _POINT_FIELDS
                 and header.get('SIZE') == ['4']*8 and header.get('TYPE') == ['F']*8
                 and header.get('COUNT') == ['1']*8 and header.get('HEIGHT') == ['1'],
                 'Expected a native binary keyframe PCD')
        points = header.get('POINTS', [])
        _require(len(points) == 1 and points[0].isascii() and points[0].isdigit(), 'Invalid PCD point count')
        count = int(points[0])
        _require(0 < count <= _MAX_POINTS and header.get('WIDTH') == points, 'PCD point count exceeds its bound')
        payload = stream.read(count*32+1)
    _require(len(payload) == count*32, 'PCD payload differs from the point count')
    result = np.frombuffer(payload, dtype='<f4').reshape(count, 8)[:, :3].astype('f8')
    _require(np.isfinite(result).all(), 'Nonfinite PCD point')
    return result


def _archived_transform(data):
    pose = np.eye(4)
    pose[:3, :3] = np.asarray(data['rotation'], dtype='f8').reshape(3, 3)
    pose[:3, 3] = data['translation']
    rotation = pose[:3, :3]
    # Match ValidateExtrinsic and ColmapCamera's normalization of recorded poses.
    _require(np.isfinite(pose).all() and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4, rtol=0)
             and abs(np.linalg.det(rotation)-1) <= 1e-4, 'Invalid archived camera pose')
    return _project(pose)


def project_depth(xyz, camera_from_body, intrinsic, valid_image):
    points = xyz @ camera_from_body[:3, :3].T + camera_from_body[:3, 3]
    _require(np.isfinite(points).all(), 'Nonfinite camera point')
    points = points[points[:, 2] > 0]
    z = points[:, 2]
    # The saved intrinsics use integer centers. ColmapCamera floors corner coordinates.
    with np.errstate(over='ignore'):
        xy = points[:, :2]/z[:, None]*np.diag(intrinsic)[:2] + (intrinsic[:2, 2] + .5)
    h, w = valid_image.shape
    inside = np.all((xy >= 0) & (xy < [w, h]), axis=1)
    x, y = xy[inside].astype(np.int32).T
    keep = valid_image[y, x]
    depth = np.full(valid_image.size, np.inf)
    np.minimum.at(depth, y[keep]*w+x[keep], z[inside][keep])
    depth = np.where(np.isfinite(depth), depth, 0).reshape(h, w)
    _require(np.all(depth <= np.finfo('f4').max), 'Projected depth exceeds float32')
    return depth.astype('f4')


def support_votes(buffers, reference, y, x):
    totals = {}
    depths = reference[y, x].astype('f8')
    limit = np.maximum(MINIMUM_TOLERANCE_M, RELATIVE_TOLERANCE*depths)
    low, high = (depths-limit)[:, None], (depths+limit)[:, None]
    h, w = reference.shape
    for radius in RADII:
        dy, dx = np.mgrid[-radius:radius+1, -radius:radius+1]
        disk = dy**2 + dx**2 <= radius**2
        yy, xx = y[:, None]+dy[disk], x[:, None]+dx[disk]
        inside = (yy >= 0) & (yy < h) & (xx >= 0) & (xx < w)
        yy, xx = np.clip(yy, 0, h-1), np.clip(xx, 0, w-1)
        votes = np.zeros((len(x), len(VOTE_FIELDS)), 'u1')
        for depth in buffers:
            z = depth[yy, xx]
            available = inside & (z > 0)
            votes[:, 0] += available.any(axis=1)
            votes[:, 1] += (available & (z < low)).any(axis=1)
            votes[:, 2] += (available & (z >= low) & (z <= high)).any(axis=1)
            votes[:, 3] += (available & (z > high)).any(axis=1)
        totals[radius] = votes
    return totals


def project_support(snapshot, key, camera, reference, image, labels):
    def frame_file(frame, name):
        return snapshot.keyframes / 'frames' / str(frame) / name

    record = _yaml(frame_file(key, 'frame.yml'))
    original = _archived_transform(record['camera_from_odom']) @ _archived_transform(record['odom_from_body'])
    _require(np.allclose(original, np.asarray(camera['camera_from_body']).reshape(4, 4), atol=6e-9, rtol=0),
             'Raw camera differs from the frozen observation')
    intrinsic = np.asarray(camera['K'])
    calibration = _yaml(snapshot.keyframes / 'dataset.yml')['camera']
    scale = float(calibration['scale'])
    width, height = (int(float(calibration[field])*scale) for field in ('width', 'height'))
    fx, fy = (float(calibration[field])*scale for field in ('fx', 'fy'))
    _require((height, width) == image.shape and math.isfinite(fx) and math.isfinite(fy) and min(fx, fy) > 0,
             'Archived native camera differs from the scoring image')
    native_k = np.array([[fx, 0, width/2-.5], [0, fy, height/2-.5], [0, 0, 1]])
    center = project_depth(read_body_cloud(frame_file(key, 'cloud.pcd')), original, native_k, image)
    _require(np.array_equal(center > 0, labels) and np.array_equal(center, reference),
             'Native PCD projection differs from the frozen depth labels')
    neighbors = [key+offset for offset in NEIGHBOR_OFFSETS if key+offset in snapshot.split.frames]
    times = {snapshot.split.frames[source]['cloud_timestamp'] for source in [key, *neighbors]}
    _require(len(times) == 1+len(neighbors), 'Neighbor clouds must have distinct capture times')
    target = np.asarray(camera['camera_from_parent']).reshape(4, 4)
    buffers = [project_depth(read_body_cloud(frame_file(source, 'cloud.pcd')), target @ snapshot.poses[source],
                             intrinsic, image) for source in neighbors]
    y, x = np.nonzero(labels)
    return neighbors, support_votes(buffers, reference, y, x)


def support_masks(votes):
    available, near, reference, _ = votes.T >= MINIMUM_VOTES
    return dict(reference=reference & ~near, competing=reference & near, near_only=near & ~reference,
                no_consensus=available & ~near & ~reference, sparse=~available)


def _measure(rows, pairs, mask):
    errors = {name: row[mask] for name, row in rows.items()}
    squared = {name: np.square(row) for name, row in errors.items()}
    arms = {name: dict(zip(_SUM_FIELDS, map(float, (error.sum(), np.abs(error).sum(), squared[name].sum()))))
            for name, error in errors.items()}
    changes = {}
    for label, (baseline, candidate) in pairs.items():
        delta = squared[candidate]-squared[baseline]
        changes[label] = dict(increase_m2=float(delta[delta > 0].sum()), decrease_m2=float(-delta[delta < 0].sum()))
    return dict(pixels=int(mask.sum()), arms=arms, changes=changes)


def _pool(rows):
    def sums(field, columns):
        return {name: {key: sum(row[field][name][key] for row in rows) for key in columns} for name in rows[0][field]}

    return dict(pixels=sum(row['pixels'] for row in rows), arms=sums('arms', _SUM_FIELDS),
                changes=sums('changes', _CHANGE_FIELDS))


def _check_partition(reference, parts):
    pooled = _pool(parts)
    _require(reference['pixels'] == pooled['pixels'], 'Support groups changed the depth denominator')
    for field in ('arms', 'changes'):
        for name, values in reference[field].items():
            _require(all(math.isclose(value, pooled[field][name][key], abs_tol=1e-8, rel_tol=1e-12)
                         for key, value in values.items()), 'Support groups changed the depth error sums')


def _describe(row, pairs):
    count = row['pixels']
    if not count:
        return dict(pixels=0, sums=row, metrics={}, deltas={})
    metrics = {name: dict(bias_m=stats['signed_sum_m']/count, l1_m=stats['absolute_sum_m']/count,
                          rmse_m=math.sqrt(stats['squared_sum_m2']/count)) for name, stats in row['arms'].items()}
    deltas = {}
    for label, (baseline, candidate) in pairs.items():
        raw = row['changes'][label]
        signed = raw['increase_m2']-raw['decrease_m2']
        first = row['arms'][baseline]['squared_sum_m2']
        second = row['arms'][candidate]['squared_sum_m2']
        # Subtraction of large independently rounded totals can lose the small difference.
        tolerance = max(1e-8, 16*np.finfo('f8').eps*(first+second))
        _require(math.isclose(signed, second-first, abs_tol=tolerance, rel_tol=1e-12),
                 'Squared error deltas differ from totals')
        deltas[label] = {key: metrics[candidate][key]-value for key, value in metrics[baseline].items()}
        deltas[label].update(squared_m2=signed, **raw)
    return dict(pixels=count, sums=row, metrics=metrics, deltas=deltas)


def compare_view(errors, pairs, keep, votes):
    all_depth = _measure(errors, pairs, keep)
    radii = {}
    for radius, vote in votes.items():
        groups = {name: _measure(errors, pairs, keep & mask) for name, mask in support_masks(vote).items()}
        _check_partition(all_depth, list(groups.values()))
        radii[str(radius)] = {name: _describe(row, pairs) for name, row in groups.items()}
    return dict(all=_describe(all_depth, pairs), radii=radii)


def summarize_mode(views, pairs):
    def summarize(group):
        summary = _describe(_pool([row['sums'] for row in group]), pairs)
        directions = {}
        for label in pairs:
            deltas = [row['deltas'][label] for row in group if row['pixels']]
            directions[label] = dict(views=len(deltas), bias_nearer=sum(row['bias_m'] < 0 for row in deltas),
                                     rmse_increased=sum(row['rmse_m'] > 0 for row in deltas),
                                     both=sum(row['bias_m'] < 0 and row['rmse_m'] > 0 for row in deltas))
        return dict(summary, directions=directions)

    all_rows = summarize([view['all'] for view in views])
    radii = {}
    for radius in map(str, RADII):
        groups = {name: summarize([view['radii'][radius][name] for view in views]) for name in GROUPS}
        _check_partition(all_rows['sums'], [row['sums'] for row in groups.values()])
        radii[radius] = groups
    return dict(all=all_rows, radii=radii)
