"""Freeze a Gaussian split from an existing frontend dataset before live training."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

from bundle import load_bundle
from gaussian_map import _project, _yaml
from holdout_split import build_manifest, canonical_json, observation_record


def file_sha(path):
    if path.is_symlink() or not path.is_file():
        raise ValueError('Expected a regular source file: ' + str(path))
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def prepare_observation(view, submap_id):
    # Match the native reconstruction/online path, which projects accepted
    # rounded rotations onto SO(3) before composing an independent C_B.
    view = dict(view)
    view['camera_from_body'] = view['camera_from_parent'] @ _project(view['parent_from_body'])
    return observation_record(view, submap_id,
                              'heldout' if view['keyframe_id'] % 8 == 1 else 'train')


def prepare(dataset, preparer, input_file, frontend_profile, frame_count, output):
    dataset, preparer = dataset.resolve(strict=True), preparer.resolve(strict=True)
    if not 2 <= frame_count <= 512 or not frontend_profile:
        raise ValueError('A 2..512 frame prefix and frontend profile are required')
    if output.exists():
        raise ValueError('Split output must be new')
    # Read the final membership records, then freeze only the declared prefix.
    membership = {}
    for group in sorted((dataset / 'online_submaps/records').iterdir(), key=lambda p: int(p.name)):
        record = max(group.glob('*.yml'), key=lambda p: int(p.stem))
        for key in _yaml(record)['keyframe_ids']:
            if int(key) in membership:
                raise ValueError('Overlapping primary groups')
            membership[int(key)] = group.name
    frames, assets = [], []
    camera_hash = file_sha(dataset / 'dataset.yml')
    # Each temporary bundle is bounded; this tool never copies the source bag.
    for begin in range(0, frame_count, 16):
        ids = list(range(begin, min(begin + 16, frame_count)))
        # The existing preparer requires both split names. Those temporary names
        # do not decide online roles; only the frozen global ID rule below does.
        if len(ids) == 1:
            ids.insert(0, begin - 1)
        with TemporaryDirectory(prefix='livo-holdout-prepare-') as temporary:
            bundle = Path(temporary) / 'views'
            subprocess.run([str(preparer), '--dataset', str(dataset), '--output', str(bundle),
                            '--train-ids', ','.join(map(str, ids[:-1])),
                            '--validation-ids', str(ids[-1]), '--source-sha256', camera_hash,
                            '--max-pixels', '1000000', '--max-cloud-points', '100000'], check=True)
            views = sorted(load_bundle(bundle), key=lambda v: v['keyframe_id'])
            for view in views:
                key = view['keyframe_id']
                if key < begin:
                    continue
                frames.append(prepare_observation(view, membership[key]))
                directory = dataset / 'frames' / str(key)
                assets.append(dict(keyframe_id=str(key), image_sha256=file_sha(directory / 'image.png'),
                                   cloud_sha256=file_sha(directory / 'cloud.pcd'),
                                   record_sha256=file_sha(directory / 'frame.yml')))
    source = dict(dataset_sha256=file_sha(dataset / 'online_submaps/dataset.yml'),
                  camera_sha256=camera_hash, input_sha256=file_sha(input_file),
                  preparer_sha256=file_sha(preparer), frontend_profile=frontend_profile, assets=assets)
    manifest = build_manifest(source, frames)
    payload = canonical_json(manifest)
    with output.open('xb') as target:
        target.write(payload)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--preparer', type=Path, required=True)
    parser.add_argument('--input-file', type=Path, required=True)
    parser.add_argument('--frontend-profile', required=True,
                        help='Record ROS, dependency versions, frontend revision and configuration')
    parser.add_argument('--frame-count', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest = prepare(args.dataset, args.preparer, args.input_file, args.frontend_profile,
                       args.frame_count, args.output)
    print(json.dumps(dict(split_id=manifest['split_id'], frames=len(manifest['frames']),
                         heldout=sum(f['role'] == 'heldout' for f in manifest['frames']))))


if __name__ == '__main__':
    main()
