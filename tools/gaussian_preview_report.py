"""Record received Gaussian frames and verify as-of source/camera/pixel evidence."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import time


_EMPTY_SHA = hashlib.sha256(b'').hexdigest()
_INTS = ('revision', 'generation', 'map_sequence', 'pose_version', 'frame_count',
         'optimized_frame_count', 'status', 'job_id', 'query_keyframe_id',
         'query_submap_id', 'calibration_id', 'changed_pixels', 'candidate_count',
         'candidate_pixels')
_SOURCE_INTS = ('submap_id', 'binding_generation', 'source_generation',
                'content_job_id', 'membership_revision', 'asset_id', 'gaussian_count')
_REASONS = {0: {''}, 1: {'awaiting_render'}, 2: {'source_changed'},
            3: {'render_disabled'}, 4: {'mapper_failed', 'invalid_snapshot', 'publication_failed'},
            5: {'stopping'}, 6: {'stopped', 'mapper_failed', 'render_disabled'}}


def _require(condition, detail):
    if not condition:
        raise ValueError(detail)


def _frame_name(value, limit):
    return (type(value) is str and 0 < len(value) <= limit and
            re.fullmatch(r'[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*', value) is not None)


def _stamp(capture):
    _require(type(capture) in (int, float) and math.isfinite(capture) and
             0 <= capture < 2147483648., 'Invalid Gaussian capture time')
    seconds = math.floor(capture)
    result = seconds * 1000000000 + math.floor((capture - seconds) * 1000000000 + .5)
    _require(result < 2147483648 * 1000000000, 'Rounded Gaussian capture time overflows')
    return result


def _image(value):
    for key in ('width', 'height', 'step', 'bytes', 'stamp_ns'):
        _require(type(value[key]) is int and value[key] >= 0, 'Invalid image scalar')
    _require(_frame_name(value['frame_id'], 128) and
             re.fullmatch('[0-9a-f]{64}', value['sha256']) is not None,
             'Invalid image frame or digest')
    if value['width'] == 0:
        _require(value['height'] == value['step'] == value['bytes'] == value['stamp_ns'] == 0
                 and value['sha256'] == _EMPTY_SHA, 'Nonready image retains payload')
    else:
        pixels = value['width'] * value['height']
        _require(0 < pixels <= 1000000 and value['encoding'] == 'rgb8' and
                 not value['is_bigendian'] and value['step'] == 3 * value['width'] and
                 value['bytes'] == 3 * pixels, 'Invalid Gaussian RGB pixels')


def gaussian_image_record(image, stamp_ns):
    _require(len(image.data) <= 3000000, 'Gaussian image exceeds byte budget')
    result = {key: int(getattr(image, key)) for key in ('width', 'height', 'step')}
    result.update(frame_id=image.header.frame_id, stamp_ns=int(stamp_ns(image.header.stamp)),
        encoding=image.encoding, is_bigendian=bool(image.is_bigendian), bytes=len(image.data),
        sha256=hashlib.sha256(bytes(image.data)).hexdigest(),
        received_at_steady_seconds=time.monotonic())
    _image(result)
    return result


def _validate(value):
    for key in _INTS:
        _require(type(value[key]) is int and 0 <= value[key] < 2**64, 'Invalid frame counter: ' + key)
    _require(value['revision'] > 0 and value['status'] in _REASONS and
             value['reason'] in _REASONS[value['status']], 'Invalid Gaussian status or revision')
    for key in ('producer_id', 'session_id'):
        _require(type(value[key]) is str and re.fullmatch('[0-9a-f]{32}', value[key]),
                 'Invalid Gaussian identity: ' + key)
    _require(_frame_name(value['frame_id'], 128) and _frame_name(value['parent_frame'], 256)
             and value['frame_id'] != value['parent_frame'] and
             value['policy'] in ('temporal', 'spatial') and value['frame_count'] <= 512 and
             value['optimized_frame_count'] <= value['frame_count'], 'Invalid Gaussian context')
    _require((value['pose_version'] == 0 and value['optimized_frame_count'] == 0 and
              value['parent_frame'] == 'camera_init') or
             (value['pose_version'] > 0 and value['optimized_frame_count'] > 0 and
              value['parent_frame'] != 'camera_init'), 'Invalid Gaussian pose frontier')
    _image(value['image'])
    _require(value['image']['frame_id'] == value['frame_id'] and
             value['image']['stamp_ns'] == value['stamp_ns'], 'Composite/image headers differ')
    pose = value['parent_from_camera']
    translation, quaternion = pose['translation'], pose['quaternion_xyzw']
    _require(len(translation) == 3 and len(quaternion) == 4 and len(value['k']) == 9 and
             all(type(v) in (int, float) and math.isfinite(v)
                 for v in translation + quaternion + value['k']), 'Invalid Gaussian camera')
    _require(abs(sum(v * v for v in quaternion) - 1) <= 1e-6, 'Nonunit display quaternion')
    if value['status'] != 0:
        _require(value['stamp_ns'] == value['capture_time'] == value['job_id'] ==
                 value['query_keyframe_id'] == value['query_submap_id'] ==
                 value['changed_pixels'] == value['candidate_count'] ==
                 value['candidate_pixels'] == value['calibration_id'] == 0 and
                 not value['projection_uncertain'] and not value['neighbor_reason'] and
                 not value['sources'] and value['k'] == [0] * 9 and
                 translation == [0, 0, 0] and quaternion == [0, 0, 0, 1] and
                 value['image']['width'] == 0, 'Nonready frame retains ready payload')
        return
    _require(value['stamp_ns'] == _stamp(value['capture_time']) and value['image']['width'] > 0
             and value['job_id'] > 0 and value['generation'] > 0 and value['map_sequence'] > 0
             and 1 <= len(value['sources']) <= 2, 'Invalid ready Gaussian identity or time')
    K = value['k']
    _require(K[0] > 0 and K[4] > 0 and K[1] == K[3] == K[6] == K[7] == 0 and K[8] == 1,
             'Invalid Gaussian intrinsics')
    for source in value['sources']:
        for key in _SOURCE_INTS:
            _require(type(source[key]) is int and 0 <= source[key] < 2**64,
                     'Invalid Gaussian source counter')
        _require(1 <= len(source['frame_ids']) <= 10 and
                 source['frame_ids'] == sorted(set(source['frame_ids'])) and
                 all(type(v) is int and 0 <= v < value['frame_count'] for v in source['frame_ids']),
                 'Invalid Gaussian source coverage')
    _require(0 <= value['changed_pixels'] <= value['image']['width'] * value['image']['height'],
             'Invalid changed pixel count')


def gaussian_render_record(message, stamp_ns):
    result = {key: int(getattr(message, key)) for key in _INTS}
    result.update({key: getattr(message, key) for key in
        ('producer_id', 'session_id', 'reason', 'parent_frame', 'policy', 'neighbor_reason')})
    result.update(frame_id=message.header.frame_id, stamp_ns=int(stamp_ns(message.header.stamp)),
        capture_time=float(message.capture_time), k=list(message.k),
        projection_uncertain=bool(message.projection_uncertain), sources=[],
        received_at_steady_seconds=time.monotonic())
    _require(len(message.sources) <= 2, 'Too many Gaussian sources')
    for source in message.sources:
        item = {key: int(getattr(source, key)) for key in _SOURCE_INTS}
        _require(len(source.frame_ids) <= 10, 'Too many source frame IDs')
        item.update(resident=bool(source.resident), manifest_sha256=source.manifest_sha256,
                    frame_ids=list(source.frame_ids))
        result['sources'].append(item)
    pose = message.parent_from_camera
    result['parent_from_camera'] = dict(
        translation=[pose.position.x, pose.position.y, pose.position.z],
        quaternion_xyzw=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    result['image'] = gaussian_image_record(message.image, stamp_ns)
    _validate(result)
    return result


def gaussian_reception_errors(observed):
    errors = list(observed['errors'])
    if observed['enabled']:
        if not observed['samples']:
            errors.append('No Gaussian composite messages received')
        if not observed['image_samples']:
            errors.append('No Gaussian image mirror messages received')
    return errors


def _image_signature(image):
    return tuple(image[key] for key in ('frame_id', 'stamp_ns', 'width', 'height',
                                       'step', 'encoding', 'is_bigendian', 'sha256'))


def _sources(header):
    result = []
    for block in header['blocks']:
        asset, target, content = block.get('asset', {}), block['target'], block['content']
        result.append(dict(submap_id=int(content['submap_id']),
            binding_generation=int(target['generation']), source_generation=int(content['source_generation']),
            content_job_id=int(content['content_job_id']), membership_revision=int(target['membership_revision']),
            resident=bool(block['resident']), asset_id=int(asset.get('asset_id', 0)),
            manifest_sha256=asset.get('manifest_sha256', ''),
            frame_ids=[int(v) for v in block['frame_ids']], gaussian_count=block['gaussian_count']))
    return result


def _same_source(value, header, map_frame):
    for key in ('generation', 'map_sequence', 'pose_version', 'frame_count',
                'optimized_frame_count', 'job_id', 'query_keyframe_id', 'query_submap_id'):
        _require(value[key] == int(header[key]), 'Received version differs from owner: ' + key)
    selection = header.get('selection', {})
    _require(value['session_id'] == header['session_id'] and value['sources'] == _sources(header)
             and value['policy'] == header.get('policy', 'temporal') and
             value['neighbor_reason'] == header['neighbor_reason'] and
             value['parent_frame'] == ('camera_init' if header['parent_frame'] == 'odom' else map_frame),
             'Received source identity differs from owner')
    for key in ('candidate_count', 'candidate_pixels', 'projection_uncertain'):
        _require(value[key] == selection.get(key, 0), 'Received selection differs from owner')


def _camera(value, metadata):
    _require(value['capture_time'] == metadata['capture_time'] and value['k'] == metadata['K'] and
             value['calibration_id'] == int(metadata['calibration_id']) and
             value['changed_pixels'] == metadata['changed_pixels'] and
             value['image']['width'] == metadata['width'] and
             value['image']['height'] == metadata['height'], 'Received camera differs from PNG metadata')
    matrix = metadata['camera_from_parent']
    expected_t = [-sum(matrix[4 * j + i] * matrix[4 * j + 3] for j in range(3)) for i in range(3)]
    x, y, z, w = value['parent_from_camera']['quaternion_xyzw']
    rotation = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    _require(all(abs(rotation[i][j] - matrix[4*j+i]) <= 1e-6 for i in range(3) for j in range(3))
             and all(abs(a-b) <= 1e-8 for a, b in zip(
                 value['parent_from_camera']['translation'], expected_t)), 'Received camera pose differs')


def inspect_gaussian_publication(run, previews):
    # Offline-only evidence dependencies; recording in either ROS runner needs
    # neither Torch nor the worker package on its Python import path.
    try:
        from .joint_render_report import inspect_joint_renders, _records, _source_identity, _pair
    except ImportError:
        from joint_render_report import inspect_joint_renders, _records, _source_identity, _pair
    from PIL import Image
    run, previews = Path(run), Path(previews)
    observed = json.loads((run / 'gaussian-publication.json').read_text())
    log_path = run / 'mapping.log'
    if not log_path.is_file():
        log_path = run / 'node.log'
    log = log_path.read_text(errors='replace')
    facts = _records(log, 'livo_atlas gaussian publication:')
    samples, mirrors = observed['samples'], observed['image_samples']
    _require(not observed['errors'], 'Gaussian receiver reported errors')
    if not observed['enabled']:
        _require(not samples and not mirrors and not facts, 'Disabled publication contains received data/work')
        return dict(enabled=False, received=0, terminal_state_received=False)
    _require(len(facts) == 1, 'Missing or duplicate publication shutdown facts')
    stats = facts[0]
    _require(stats['finished'] == 1 and stats['failed'] == stats['pending'] == stats['in_flight'] == 0
             and 0 < stats['peak_pending'] <= 1 and stats['peak_in_flight'] <= 1
             and stats['published'] + stats['unavailable'] >= len(samples), 'Invalid publication resource/call facts')
    # Retain the stronger existing owner, source identity, PNG and input-cutoff
    # checks; native call counts cannot replace them.
    inspect_joint_renders(run, previews)
    summary = json.loads((run / 'summary.json').read_text())
    cutoff, start = summary['playback_input_cutoff_steady_seconds'], summary['playback_started_steady_seconds']
    events = _records(log, 'livo_atlas gaussian joint accepted:')
    accepted = {(event['session'], event['job']): (event, _source_identity(event)) for event in events}
    acceptance_order = {(event['session'], event['job']): i for i, event in enumerate(events)}
    retained = {}
    for path in previews.glob('joint-render-*.json'):
        metadata = _pair(path)
        with Image.open(path.with_suffix('.png')) as image:
            digest = hashlib.sha256(image.tobytes()).hexdigest()
        retained[(metadata['as_of']['session_id'], int(metadata['as_of']['job_id']))] = (metadata, digest)
    last, producer, session, terminal = 0, None, None, False
    context = None
    displayed = None
    displayed_order = -1
    withdrawn = set()
    ready, png_matches = 0, 0
    ready_images = set()
    for value in samples:
        _validate(value)
        _require(not terminal and value['revision'] > last and value['revision'] <= stats['last_revision'],
                 'Received revision is stale, duplicate or follows terminal state')
        producer = producer or value['producer_id']
        session = session or value['session_id']
        _require(value['producer_id'] == producer and value['session_id'] == session,
                 'Foreign producer/session requires explicit rebind')
        _require(value['frame_id'] == observed['optical_frame'], 'Unexpected virtual optical frame')
        current = tuple(value[key] for key in ('generation', 'map_sequence', 'pose_version',
                                                'frame_count', 'optimized_frame_count'))
        _require(context is None or all(a >= b for a, b in zip(current, context)),
                 'Received context rolled back after a newer known state')
        context = current
        last = value['revision']
        terminal = value['status'] == 6
        if value['status'] != 0:
            if displayed is not None:
                withdrawn.add(displayed)
            continue
        key = (session, value['job_id'])
        _require(key in accepted, 'Received READY was never accepted by owner')
        _require(key not in withdrawn and acceptance_order[key] >= displayed_order,
                 'Received READY resurrects a withdrawn or older displayed job')
        displayed = key
        displayed_order = acceptance_order[key]
        event, header = accepted[key]
        _same_source(value, header, observed['map_frame'])
        _require(value['changed_pixels'] == event['changed_pixels'], 'Contribution differs from owner')
        received = value['received_at_steady_seconds']
        _require(type(received) in (int, float) and math.isfinite(received) and received >= 0,
                 'Invalid actual receiver time')
        live = start <= event['accepted_at_steady_seconds'] <= received <= cutoff
        ready += int(live)
        image = value['image']
        ready_images.add(_image_signature(image))
        if key in retained:
            metadata, digest = retained[key]
            _camera(value, metadata)
            _require(image['sha256'] == digest, 'Received RGB differs from retained PNG')
            png_matches += int(live)
    mirror_matches = 0
    for image in mirrors:
        _image(image)
        mirror_matches += int(_image_signature(image) in ready_images)
    _require(ready >= 1 and png_matches >= 1 and mirror_matches > 0,
             'Missing actual READY/PNG/mirror reception before input cutoff')
    return dict(enabled=True, received=len(samples), producer_id=producer, session_id=session,
        ready_before_input_cutoff=ready, png_matches_before_input_cutoff=png_matches,
        mirror_matches=mirror_matches, terminal_state_received=terminal,
        native_calls_completed=stats['published'], unavailable_attempts=stats['unavailable'],
        peak_pending=stats['peak_pending'], peak_in_flight=stats['peak_in_flight'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--previews', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = inspect_gaussian_publication(args.run, args.previews)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(json.dumps(dict(passed=False, error=str(error)), indent=2))
        return 2
    print(json.dumps(dict(passed=True, **result), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
