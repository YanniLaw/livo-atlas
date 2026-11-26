"""Small controlled reports with independently chosen render/depth errors."""

import copy
import hashlib
import json

import numpy as np

from evaluate_heldout_snapshot import _save_samples
from evaluation_metrics import aggregate_metrics, measure
from heldout_fixtures import make_heldout_snapshot
from holdout_split import canonical_json


def make_sampling_pair(root):
    _, _, split, views = make_heldout_snapshot(root, blocks=6)
    groups, training = [], []
    for owner in range(6):
        ids = [key for key in (2*owner, 2*owner+1) if key not in (1, 9)]
        held = [key for key in (2*owner, 2*owner+1) if key in (1, 9)]
        groups.append(dict(submap_id=owner, anchor_id=2*owner, frame_count=12,
                           train_ids=ids, held_out_ids=held))
        operations = []
        for index, key in enumerate(ids):
            operations.append(dict(keyframe_id=key, steps=20, frame_ids=ids[:index+1],
                training_frame_ids=[key]*10 + [ids[0] if index else key]*10,
                gaussian_count=index+1))
        training.append(dict(submap_id=owner, anchor_id=2*owner, parent_from_submap=np.eye(4).tolist(),
            retained_frame_ids=ids, gaussian_origins={str(k): 1 for k in ids}, training=operations))
    plan = [dict(keyframe_id=1, selected_ids=[0]), dict(keyframe_id=9, selected_ids=[3, 4])]
    for policy, color, z in (('uniform', .2, 3.), ('coverage', .3, 3.5)):
        output = root / policy
        output.mkdir()
        (output / 'split.json').write_bytes(split.payload)
        report = dict(schema=1, kind='livo_atlas.gaussian_sampling_evaluation', status='succeeded',
            seed_policy=policy, random_seed=0, split_id=split.split_id, selection_policy='spatial',
            selection_semantics='frozen_reference_online_models', selector_runtime_sha256='a'*64,
            selection_plan=plan, selection_plan_sha256=hashlib.sha256(canonical_json(plan)).hexdigest(),
            sources={'map_sha256': 'a'*64}, source_sha256={'source': 'b'*64}, final_poses_sha256='c'*64,
            config_sha256='d'*64, preparer_sha256='e'*64, software={'torch': 'fixture'},
            training_options={'depth_loss_weight': .05}, groups=groups, submaps=copy.deepcopy(training),
            training_calls=10, training_steps=200, queries=[], artifacts={})
        if policy == 'coverage':
            for group in report['submaps']:
                for operation in group['training']:
                    operation['seed_selection'] = dict(depth_candidates=1, prepass_seconds=None,
                        uncovered_candidates=None, reserved_uncovered=0)
        for selection in plan:
            key = selection['keyframe_id']
            query = views[key]
            shape = query['depth_m'].shape
            def rendered(rgb):
                value = dict(rgb=np.full((*shape, 3), rgb, dtype='f4'),
                             alpha=np.ones(shape, dtype='f4'), depth_m=np.full(shape, z, dtype='f4'))
                value['alpha'][0, 0 if policy == 'uniform' else 1] = 0
                return value
            single = rendered(color)
            joint = rendered(color+.05) if key == 9 else None
            row = dict(keyframe_id=key, owner_submap_id=key//2, selected_ids=selection['selected_ids'],
                       query=dict(width=8, height=6, K=query['K'].tolist(),
                                  camera_from_parent=np.eye(4).ravel().tolist()),
                       single=measure(single, query), joint=measure(joint, query) if joint else None)
            report['queries'].append(row)
            report['artifacts'].update(_save_samples(output, dict(query=query, single=single, joint=joint)))
        report['summary'] = {mode: aggregate_metrics([row[mode] for row in report['queries']
                                                    if row[mode] is not None]) for mode in ('single', 'joint')}
        (output / 'report.json').write_text(json.dumps(report))
    return root / 'uniform', root / 'coverage'
