"""Small proven online assets with an independent heldout camera."""

import copy
import hashlib
import json
import shutil

from gaussian_state import read_state, write_state
from holdout_split import HoldoutSplit, build_manifest, canonical_json, observation_record
from joint_fixtures import make_snapshot


def replace_asset(directory, row, state):
    path = directory / 'states' / row['asset']['asset_id']
    shutil.rmtree(path)
    write_state(path, state)
    row['asset']['manifest_sha256'] = hashlib.sha256((path / 'manifest.json').read_bytes()).hexdigest()
    for key, file in (('manifest_bytes', 'manifest.json'), ('gaussian_bytes', 'gaussians.npz'),
                      ('observation_bytes', 'observations.npz')):
        row['asset'][key] = str((path / file).stat().st_size)


def remove_models(directory, submaps):
    path = directory / 'map.json'
    document = json.loads(path.read_text())
    for row in document['entries']:
        if int(row['submap_id']) not in submaps:
            continue
        shutil.rmtree(directory / 'states' / row['asset']['asset_id'])
        for key in list(row):
            if key in ('content', 'content_source', 'trained_frame_ids', 'gaussian_count') or key.startswith('asset'):
                row.pop(key)
        row.update(availability='missing', resident=0, volatile=0)
    path.write_text(json.dumps(document))
    current = [row for row in document['entries'] if 'asset' in row]
    path = directory / 'finalization.json'
    report = json.loads(path.read_text())
    report.update(current=len(current), missing=len(document['entries'])-len(current), eligible=len(current),
                  eligible_submap_ids=[row['submap_id'] for row in current],
                  asset_ids=sorted([row['asset']['asset_id'] for row in current], key=int))
    path.write_text(json.dumps(report))


def make_heldout_snapshot(root, blocks=3, *, write_source=None):
    directory, keyframes = make_snapshot(root, blocks=blocks)
    dataset = keyframes / 'online_submaps/dataset.yml'
    dataset.write_text('fixture: true\n')
    document = json.loads((directory / 'map.json').read_text())
    frames, assets, views, states = [], [], {}, []
    for row in document['entries']:
        state = read_state(directory / 'states' / row['asset']['asset_id'])
        for view in state.views:
            key = view['keyframe_id']
            for field in ('state_timestamp', 'cloud_timestamp', 'image_timestamp'):
                view[field] += int(row['anchor_keyframe_id'])
            view['rgb'][:] = (key + 1) / 255
            source = keyframes / 'frames' / str(key)
            (source / 'image.png').write_bytes(('image' + str(key)).encode())
            if write_source is None:
                (source / 'cloud.pcd').write_bytes(('cloud' + str(key)).encode())
            else:
                write_source(source, view)
            views[key] = copy.deepcopy(view)
            frames.append(observation_record(view, row['submap_id'],
                                             'heldout' if key % 8 == 1 else 'train'))
            assets.append(dict(keyframe_id=str(key), image_sha256=hashlib.sha256(
                (source / 'image.png').read_bytes()).hexdigest(), cloud_sha256=hashlib.sha256(
                (source / 'cloud.pcd').read_bytes()).hexdigest(), record_sha256=hashlib.sha256(
                (source / 'frame.yml').read_bytes()).hexdigest()))
        states.append(state)
    source = dict(dataset_sha256=hashlib.sha256(dataset.read_bytes()).hexdigest(),
                  camera_sha256=hashlib.sha256((keyframes / 'dataset.yml').read_bytes()).hexdigest(),
                  input_sha256='a'*64, preparer_sha256='b'*64, frontend_profile='unit fixture', assets=assets)
    split = HoldoutSplit(canonical_json(build_manifest(source, frames)))
    (directory / 'split.json').write_bytes(split.payload)
    for row, state in zip(document['entries'], states):
        selected = [i for i, view in enumerate(state.views) if view['keyframe_id'] % 8 != 1]
        state.views = [state.views[i] for i in selected]
        state.parameters = {key: value[selected].copy() for key, value in state.parameters.items()}
        state.origin_ids = state.origin_ids[selected].copy()
        ids = [view['keyframe_id'] for view in state.views]
        state.metadata.update(schema=2, holdout=split.proof(ids))
        path = directory / 'states' / row['asset']['asset_id']
        shutil.rmtree(path)
        write_state(path, state)
        row.update(trained_frame_ids=list(map(str, ids)), asset_trained_frame_ids=list(map(str, ids)),
                   gaussian_count=len(ids), asset_gaussian_count=len(ids))
        row['asset']['manifest_sha256'] = hashlib.sha256((path / 'manifest.json').read_bytes()).hexdigest()
        for key, file in (('manifest_bytes', 'manifest.json'), ('gaussian_bytes', 'gaussians.npz'),
                          ('observation_bytes', 'observations.npz')):
            row['asset'][key] = str((path / file).stat().st_size)
    (directory / 'map.json').write_text(json.dumps(document))
    return directory, keyframes, split, views
