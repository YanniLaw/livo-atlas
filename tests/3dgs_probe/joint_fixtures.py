"""Real, small final-snapshot files for joint rendering boundary tests."""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from gaussian_state import write_state
from state_fixtures import make_state, pose


def transform(matrix):
    return {'rotation': matrix[:3, :3].ravel().tolist(),
            'translation': matrix[:3, 3].tolist()}


def make_snapshot(root, blocks=3, retained=2):
    root = Path(root)
    directory, keyframes = root / 'map', root / 'keyframes'
    directory.mkdir()
    states = directory / 'states'
    states.mkdir()
    keyframes.mkdir()
    (keyframes / 'dataset.yml').write_text(yaml.safe_dump({'fixture': True}))
    total = 2 * blocks
    rows = []
    for submap in range(blocks):
        first = 2 * submap
        state = make_state()
        state.metadata.update(capture_kind='online_confirmed', submap_id=str(submap),
            anchor_keyframe_id=str(first), frame_count=str(total),
            membership_revision='3', submap_state='sealed',
            parent_from_submap=pose(first).ravel().tolist())
        state.metadata.pop('source_manifest_sha256')
        state.metadata.pop('bundle_manifest_sha256')
        for i, primary in enumerate(state.metadata['primary']):
            primary['keyframe_id'] = str(first + i)
        state.origin_ids += first
        for view in state.views:
            view['keyframe_id'] += first
        if retained == 1:
            state.views = state.views[1:]
            state.parameters = {k: v[1:].copy() for k, v in state.parameters.items()}
            state.origin_ids = state.origin_ids[1:].copy()
        asset_path = states / str(submap + 1)
        write_state(asset_path, state)
        source = {k: copy.deepcopy(state.metadata[k]) for k in (
            'submap_id', 'anchor_keyframe_id', 'membership_revision', 'generation',
            'pose_version', 'frame_count', 'optimized_frame_count', 'parent_frame',
            'submap_state', 'parent_from_submap', 'primary')}
        source['primary_count'] = 2
        content = dict(session_id='a'*32, submap_id=str(submap),
                       source_generation='1', content_job_id=str(submap + 1))
        asset = dict(content, asset_id=str(submap + 1), manifest_sha256=hashlib.sha256(
            (asset_path / 'manifest.json').read_bytes()).hexdigest())
        for key, name in (('manifest_bytes','manifest.json'), ('gaussian_bytes','gaussians.npz'),
                          ('observation_bytes','observations.npz')):
            asset[key] = str((asset_path / name).stat().st_size)
        frame_ids = [str(view['keyframe_id']) for view in state.views]
        rows.append(dict(submap_id=str(submap), anchor_keyframe_id=str(first), primary_count=2,
            membership_revision='3', submap_state='sealed', availability='current', reason='',
            resident=int(submap == blocks-1), binding={k:v for k,v in source.items() if k!='primary'},
            binding_job_id=content['content_job_id'], content=content, volatile=0,
            content_source=copy.deepcopy(source), trained_frame_ids=frame_ids,
            gaussian_count=len(state.origin_ids), asset=asset, asset_source=copy.deepcopy(source),
            asset_trained_frame_ids=frame_ids, asset_gaussian_count=len(state.origin_ids), asset_current=1))
        record = keyframes / 'online_submaps/records' / str(submap)
        record.mkdir(parents=True)
        (record / '3.yml').write_text(yaml.safe_dump(dict(schema_version=1, id=str(submap),
            membership_revision='3', state='sealed', anchor_keyframe_id=str(first),
            keyframe_ids=[str(first), str(first+1)], support_keyframe_ids=[])))
        for i in range(first,first+2):
            folder = keyframes / 'frames' / str(i)
            folder.mkdir(parents=True)
            (folder / 'frame.yml').write_text(yaml.safe_dump(dict(id=str(i), odom_from_body=transform(pose(i)))))
    document = dict(schema=1, kind='livo_atlas.gaussian_map', session_id='a'*32, map_sequence='1',
        pose_version='0', frame_count=str(total), optimized_frame_count='0', parent_frame='odom',
        written_at_unix_seconds=1.0, entries=rows)
    counts = dict(accepted=str(total), effective_training=str(total), rebuilds='0', checkpoints='0')
    report = dict(schema=1, kind='livo_atlas.gaussian_finalization', session_id='a'*32,
        status='succeeded', reason='', pose_version='0', frame_count=str(total), optimized_frame_count='0',
        parent_frame='odom', map_sequence='1', eligible_submap_ids=[str(i) for i in range(blocks)],
        asset_ids=[str(i+1) for i in range(blocks)], eligible=blocks, target_valid=1,
        eligible_set_frozen=1, durable=1, worker_stopped=1, current=blocks, needs_rebuild=0,
        missing=0, dirty=0, elapsed_seconds=.2, repair_seconds=.1, shutdown_seconds=.1,
        repair_attempts='0', before=counts, after=dict(counts, checkpoints=str(blocks)))
    (directory / 'map.json').write_text(json.dumps(document))
    (directory / 'finalization.json').write_text(json.dumps(report))
    (directory / 'status.json').write_text(json.dumps({'session_id':'a'*32}))
    return directory, keyframes


def rigid_update(directory, keyframes, optimized=4):
    document = json.loads((directory / 'map.json').read_text())
    document.update(pose_version='1', optimized_frame_count=str(optimized), parent_frame='map')
    for row in document['entries']:
        binding = row['binding']
        binding.update(pose_version='1', optimized_frame_count=str(optimized), parent_frame='map')
        matrix = np.asarray(binding['parent_from_submap']).reshape(4,4).copy()
        matrix[1,3] += 10
        binding['parent_from_submap'] = matrix.ravel().tolist()
    (directory / 'map.json').write_text(json.dumps(document))
    report = json.loads((directory / 'finalization.json').read_text())
    report.update(pose_version='1', optimized_frame_count=str(optimized), parent_frame='map')
    (directory / 'finalization.json').write_text(json.dumps(report))
    folder = keyframes / 'online_submaps/poses'
    folder.mkdir()
    data = dict(schema_version=2, version='1', base_version='0', parent_frame='map',
        frame_count=str(optimized), optimized_frame_count=str(optimized),
        tail_policy='last_optimized_anchor_projected', tail_anchor_keyframe_id=str(optimized-1),
        poses=[dict(id=str(i), map_from_body=transform(pose(i,10))) for i in range(optimized)])
    (folder / '1.yml').write_text(yaml.safe_dump(data))
