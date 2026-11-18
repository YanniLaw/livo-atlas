"""Inspect bounded historical render pairs against owner and EOF evidence."""
import json
import math
from pathlib import Path
import re
import sys

from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent / '3dgs_probe'))
from joint_render_protocol import validate_render_message


def _require(condition, detail):
    if not condition:
        raise ValueError(detail)


def _finite(value):
    _require(type(value) in (int, float) and math.isfinite(value), 'Expected finite timing')
    return value


def _json(path):
    _require(path.is_file() and not path.is_symlink() and path.stat().st_size <= 65536,
             'Render JSON exceeds its budget or is not a regular file')
    def unique(pairs):
        result = {}
        for key, value in pairs:
            _require(key not in result, 'Duplicate render JSON key')
            result[key] = value
        return result
    def constant(value):
        raise ValueError('Nonfinite JSON value: ' + value)
    return json.loads(path.read_text(), object_pairs_hook=unique, parse_constant=constant)


def _records(text, marker):
    result = []
    for line in text.splitlines():
        if marker not in line:
            continue
        fields = dict(re.findall(r'([a-z_]+)=([^\s]+)', line.split(marker, 1)[1]))
        converted = {}
        for key, value in fields.items():
            if key == 'session':
                converted[key] = value
            elif key.endswith('_seconds'):
                converted[key] = _finite(float(value))
            else:
                _require(re.fullmatch('0|[1-9][0-9]*', value), 'Invalid owner counter')
                converted[key] = int(value)
        result.append(converted)
    return result


def _wire_metrics(value):
    keys = {'image_pixels', 'covered_pixels', 'depth_pixels_available', 'depth_pixels_used',
            'rgb_l1', 'rgb_mse', 'rgb_psnr_db', 'rgb_perfect', 'alpha_coverage',
            'depth_l1_m', 'depth_rmse_m'}
    _require(isinstance(value, dict) and value.keys() == keys and type(value['rgb_perfect']) is bool,
             'Invalid stored render metrics')
    image, depth = value['image_pixels'], value['depth_pixels_used']
    for valid, fields in ((image > 0, ('rgb_l1', 'rgb_mse', 'alpha_coverage')),
                          (depth > 0, ('depth_l1_m', 'depth_rmse_m'))):
        _require(all((value[key] is not None) == valid for key in fields),
                 'Stored optional render metrics differ from their valid samples')
    result = {key: (0 if item is None else item) for key, item in value.items()}
    result.update(rgb_perfect=int(value['rgb_perfect']), image_valid=int(image > 0),
                  depth_valid=int(depth > 0), psnr_valid=int(value['rgb_psnr_db'] is not None))
    return result


def _pair(path):
    data = _json(path)
    _require(data.get('schema') == 1 and data.get('kind') == 'livo_atlas.joint_render' and
             data.get('training_view') is True, 'Expected a retained training-view render')
    header = dict(data['as_of'], type='joint_render_ok')
    _require(path.stem == 'joint-render-' + header['job_id'], 'Render filename and job differ')
    image = path.with_suffix('.png')
    _require(image.is_file() and not image.is_symlink() and image.stat().st_size <= 4 * 1024 * 1024,
             'Missing or oversized render PNG')
    width, height = data['width'], data['height']
    _require(type(width) is int and type(height) is int and width > 0 and height > 0 and
             width * height <= 1_000_000, 'Stored render camera exceeds its budget')
    with Image.open(image) as pixels:
        _require(pixels.mode == 'RGB' and pixels.size == (width, height), 'Render PNG shape differs')
        rgb = pixels.tobytes()
    K = data['K']
    _require(isinstance(K, list) and len(K) == 9 and K[1] == K[3] == K[6] == K[7] == 0 and K[8] == 1,
             'Invalid stored camera matrix')
    header.update(training_view=1, width=width, height=height,
        calibration_id=data['calibration_id'], fx=K[0], fy=K[4], cx=K[2], cy=K[5],
        parent_from_body=data['parent_from_body'], camera_from_body=data['camera_from_body'],
        camera_from_parent=data['camera_from_parent'], image_timestamp=data['capture_time'],
        **{key: data[key] for key in ('state_timestamp', 'cloud_timestamp', 'inverse_exposure_time',
            'changed_pixels', 'worker_seconds', 'gpu_allocated_bytes', 'gpu_reserved_bytes',
            'gpu_peak_allocated_bytes', 'gpu_peak_reserved_bytes', 'worker_peak_rss_bytes')})
    header.update(single=_wire_metrics(data['single']), joint=_wire_metrics(data['joint']),
        single_gaussian_count=header['blocks'][-1]['gaussian_count'],
        gaussian_count=sum(block['gaussian_count'] for block in header['blocks']))
    validate_render_message(header, rgb)
    for key in ('accepted_at_steady_seconds', 'accepted_at_unix_seconds', 'latency_seconds'):
        _require(_finite(data[key]) >= 0, 'Negative render acceptance timing')
    return data


def inspect_joint_renders(run, previews, min_dual=0, min_groups=0):
    run, previews = Path(run), Path(previews)
    summary = json.loads((run / 'summary.json').read_text())
    _require(summary.get('joint_render_enabled') is True, 'Run did not enable joint rendering')
    start = _finite(summary.get('playback_started_steady_seconds'))
    eof = _finite(summary.get('playback_eof_steady_seconds'))
    cutoff = _finite(summary.get('playback_input_cutoff_steady_seconds'))
    source = summary.get('playback_input_cutoff_source')
    _require(source in ('fixed_rate_lower_bound', 'source_publish_begin'), 'Missing input publication proof')
    if source == 'fixed_rate_lower_bound':
        try:
            from .run_baseline import fixed_rate_input_cutoff
        except ImportError:
            from run_baseline import fixed_rate_input_cutoff
        _require(summary.get('playback_wall_clock_stable') is True, 'Input timeline clock changed')
        stamps = [summary.get(key) for key in
                  ('playback_first_input_timestamp_ns', 'playback_last_input_timestamp_ns')]
        _require(all(type(value) is str and re.fullmatch('0|[1-9][0-9]*', value) for value in stamps),
                 'Invalid input timeline timestamps')
        bound = fixed_rate_input_cutoff(start, int(stamps[0]), int(stamps[1]), summary['playback_rate'])
        _require(abs(bound - cutoff) < 1e-7, 'Input publication lower bound differs')
    _require(0 <= start <= cutoff <= eof and start < eof, 'Missing or inconsistent input EOF interval')
    log = run / 'mapping.log'
    if not log.is_file():
        log = run / 'node.log'
    text = log.read_text(errors='replace')
    events = _records(text, 'livo_atlas gaussian joint accepted:')
    counters = _records(text, 'livo_atlas gaussian joint:')
    _require(len(counters) == 1 and bool(events), 'Missing owner render acceptance evidence')
    stats = counters[0]
    _require(stats.get('accepted') == len(events) and stats.get('failed') == 0 and
             stats.get('disabled') == 0 and stats.get('peak_pending', 2) <= 1 and
             stats.get('peak_in_flight', 2) <= 1, 'Failed or unbounded owner render session')
    jobs = {}
    for event in events:
        _require(event['job'] not in jobs and 0 < event['job'] < 2**64 and
                 1 <= event['blocks'] <= 2 and event['query'] < event['frame_count'] <= 512 and
                 event['changed_pixels'] >= 0, 'Invalid owner render event')
        jobs[event['job']] = event
    online = [event for event in events if start <= event['accepted_at_steady_seconds'] <= cutoff]
    _require(bool(online), 'No render was accepted before input EOF')
    dual = [event for event in online if event['blocks'] == 2 and event['changed_pixels'] > 0]
    groups = sorted({event['owner'] for event in dual})
    _require(len(dual) >= min_dual and len(groups) >= min_groups,
             'Insufficient joint contributions accepted before input EOF')
    files = sorted(previews.glob('joint-render-*.json'))
    images = sorted(previews.glob('joint-render-*.png'))
    saved = stats.get('saved', -1)
    _require(type(saved) is int and 1 <= saved <= stats['accepted'] and
             len(files) == min(4, saved) and
             {path.stem for path in files} == {path.stem for path in images} and
             not any('.tmp.' in path.name for path in files + images), 'Invalid retained render pairs')
    retained = []
    for path in files:
        data = _pair(path)
        q = data['as_of']
        event = jobs.get(int(q['job_id']))
        _require(event is not None, 'Stored render has no owner acceptance')
        for field, key in (('job_id', 'job'), ('map_sequence', 'map_sequence'), ('pose_version', 'pose_version'),
                           ('frame_count', 'frame_count'), ('optimized_frame_count', 'optimized_frame_count'),
                           ('generation', 'generation'), ('query_keyframe_id', 'query'), ('query_submap_id', 'owner')):
            _require(int(q[field]) == event[key], 'Stored render differs from accepted as_of identity')
        _require(q['session_id'] == event['session'] and len(q['blocks']) == event['blocks'] and
                 data['changed_pixels'] == event['changed_pixels'] and
                 abs(data['accepted_at_steady_seconds'] - event['accepted_at_steady_seconds']) < 1e-7,
                 'Stored render differs from owner acceptance')
        retained.append(dict(file=path.name, as_of=q,
                             accepted_at_steady_seconds=data['accepted_at_steady_seconds']))
    return dict(pairs=len(files), accepted_before_eof=len(online),
        acceptance_cutoff_steady_seconds=cutoff, observed_eof_steady_seconds=eof,
        dual_contributions_before_eof=len(dual), query_groups_before_eof=groups,
        stats=stats, events=events, retained=retained)
