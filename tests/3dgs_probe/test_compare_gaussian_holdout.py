import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from compare_gaussian_holdout import compare


def write_json(path, document):
    raw = json.dumps(document, sort_keys=True).encode()
    path.write_bytes(raw)
    return hashlib.sha256(raw).hexdigest()


def write_samples(root, view, samples):
    path = root / view['depth_samples']
    path.parent.mkdir(exist_ok=True, parents=True)
    np.save(path, samples, allow_pickle=False)
    view['depth_samples_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()


def evaluation(root, policy):
    root.mkdir()
    frontier = dict(session_id='a' * 32, pose_version=2, parent_frame='map', frame_count=9,
                    optimized_frame_count=8, map_sequence=25)
    groups = [dict(submap_id=0, anchor_id=0, train_ids=[0, 2], held_out_ids=[1]),
              dict(submap_id=1, anchor_id=3, train_ids=[3, 5, 7, 8], held_out_ids=[4, 6])]
    split = dict(schema=1, groups=groups,
                 observations=[dict(keyframe_id=key, capture_time=100. + key,
                                    split='validation' if key in (1, 4, 6) else 'train',
                                    image_sha256=f'{key:064x}') for key in range(9)],
                 **frontier)
    split_sha256 = write_json(root / 'split.json', split)
    submaps, views = [], []
    for group in groups:
        group_id, train_ids = group['submap_id'], group['train_ids']
        training = []
        for i, key in enumerate(train_ids):
            others = train_ids[:i] or [key]
            operation = dict(keyframe_id=key, frame_ids=train_ids[:i + 1], steps=20,
                             training_frame_ids=[key] * 10 + [others[j % len(others)] for j in range(10)],
                             gaussian_count=1000 * (i + 1))
            if policy == 'coverage':
                prepass = i > 0 and i != 2
                operation['seed_selection'] = dict(prepass_seconds=.001 if prepass else None,
                                                   depth_candidates=1000 if i == 2 else 1200,
                                                   uncovered_candidates=700 if prepass else None,
                                                   reserved_uncovered=500 if prepass else 0)
            training.append(operation)
        origins = {str(key): 1000 for key in train_ids}
        submaps.append(dict(submap_id=group_id, anchor_id=group['anchor_id'],
                           parent_from_submap=np.eye(4).tolist(), retained_frame_ids=train_ids,
                           gaussian_origins=origins, training=training, training_seconds=1.,
                           gpu_peak_allocated_bytes=1024, gpu_peak_reserved_bytes=2048))
        for i, key in enumerate(group['held_out_ids']):
            height = i + 1
            view = dict(keyframe_id=key, submap_id=group_id, role='interior' if i else 'near_group_start',
                        capture_time=100. + key, calibration_id=0, width=4, height=height,
                        K=np.eye(3).tolist(), camera_from_parent=np.eye(4).tolist(),
                        depth_samples=f'views/{key}/depth-samples.npy', single=None, joint=None)
            samples = np.zeros(4, dtype=[('pixel_index', '<u4'), ('observed_z_m', '<f4'),
                                        ('single_z_m', '<f4'), ('joint_z_m', '<f4'),
                                        ('single_covered', '?'), ('joint_covered', '?')])
            samples['pixel_index'] = range(4)
            samples['observed_z_m'] = 4.
            selected = ([0, 1] if policy == 'uniform' else [1, 2]) if not i else (
                [0] if policy == 'uniform' else [3])
            depths = ([5., 4.5] if policy == 'uniform' else [4.25, 6.]) if not i else [5.]
            mse = .25 if policy == 'uniform' else .125 if not i else .0625
            for render in ('single', 'joint') if group_id else ('single',):
                samples[f'{render}_z_m'][selected] = depths
                samples[f'{render}_covered'][selected] = True
                counts = {str(group_id): sum(origins.values())}
                if render == 'joint':
                    counts['0'] = sum(submaps[0]['gaussian_origins'].values())
                view[render] = dict(gaussian_count=sum(counts.values()), gaussian_counts=counts,
                                    metrics=dict(image_pixels=4 * height, covered_pixels=len(selected),
                                                 depth_pixels_available=4, depth_pixels_used=len(selected),
                                                 depth_l1_m=float(np.abs(np.array(depths) - 4).mean()),
                                                 rgb_mse=mse, alpha_threshold=.05),
                                    timings=dict(render_seconds=.001), gpu_peak_allocated_bytes=1024,
                                    gpu_peak_reserved_bytes=2048)
            write_samples(root, view, samples)
            views.append(view)
    report = dict(schema=1, status='succeeded', kind='livo_atlas.gaussian_holdout_evaluation',
                  split_sha256=split_sha256, seed_policy=policy, submaps=submaps, views=views,
                  environment={'device': 'fixture'}, elapsed_seconds=5., peak_rss_bytes=4096,
                  **frontier)
    write_json(root / 'report.json', report)
    return report


class HoldoutComparisonTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.baseline = self.root / 'baseline'
        self.candidate = self.root / 'candidate'
        evaluation(self.baseline, 'uniform')
        self.report = evaluation(self.candidate, 'coverage')
        self.output = self.root / 'comparison.json'

    def test_depth_partitions_and_pixel_weighted_rgb(self):
        result = compare(self.baseline, self.candidate, self.output)
        self.assertEqual(result, json.loads(self.output.read_text()))
        joint = result['summary']['joint']
        self.assertEqual(joint['query_count'], 2)
        self.assertIsNone(result['views'][0]['joint'])
        self.assertEqual(result['summary']['single']['query_count'], 3)
        self.assertAlmostEqual(joint['candidate']['rgb_mse'], (.125 * 4 + .0625 * 8) / 12)
        self.assertAlmostEqual(joint['candidate']['alpha_coverage'], 3 / 12)
        depth = result['views'][1]['joint']['depth']
        self.assertEqual(depth, {
            'common': dict(count=1, baseline_l1_m=.5, candidate_l1_m=.25, delta_l1_m=-.25),
            'candidate_only': dict(count=1, candidate_l1_m=2.),
            'baseline_only': dict(count=1, baseline_l1_m=1.),
            'neither': dict(count=1),
        })
        self.assertEqual(joint['depth']['common'], depth['common'])
        self.assertEqual(result['views'][2]['joint']['depth']['common'],
                         dict(count=0, baseline_l1_m=None, candidate_l1_m=None, delta_l1_m=None))

    def test_mismatched_camera_identity_and_training_are_rejected(self):
        for message, edit in (
            ('camera', lambda row: row['views'][1]['camera_from_parent'][0].__setitem__(3, 2.)),
            ('camera', lambda row: row['views'][1]['K'][0].__setitem__(0, 3.)),
            ('dimensions', lambda row: row['views'][1].update(width=3)),
            ('query ID', lambda row: row['views'].__setitem__(1, row['views'][0])),
            ('training', lambda row: row['submaps'][1]['training'][1]['training_frame_ids'].__setitem__(0, 4)),
            ('split', lambda row: row.update(split_sha256='0' * 64)),
        ):
            document = copy.deepcopy(self.report)
            edit(document)
            write_json(self.candidate / 'report.json', document)
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                compare(self.baseline, self.candidate, self.output)
            self.assertFalse(self.output.exists())

    def test_missing_corrupt_and_different_depth_records_are_rejected(self):
        view = self.report['views'][2]
        path = self.candidate / view['depth_samples']
        samples = np.load(path, allow_pickle=False)
        path.unlink()
        with self.assertRaisesRegex((OSError, ValueError), 'depth-samples.npy'):
            compare(self.baseline, self.candidate, self.output)
        path.write_bytes(b'broken')
        with self.assertRaisesRegex(ValueError, 'SHA256'):
            compare(self.baseline, self.candidate, self.output)
        for message, field, value in (('labels', 'observed_z_m', [5., 4., 4., 4.]),
                                      ('indices', 'pixel_index', [0, 1, 3, 4]),
                                      ('indices', 'pixel_index', [0, 1, 1, 3]),
                                      ('depth', 'joint_z_m', [1., 0., 0., 5.])):
            changed = samples.copy()
            changed[field] = value
            write_samples(self.candidate, view, changed)
            write_json(self.candidate / 'report.json', self.report)
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, message):
                compare(self.baseline, self.candidate, self.output)
            self.assertFalse(self.output.exists())

    def test_reported_depth_l1_must_agree_with_hashed_predictions(self):
        view = self.report['views'][1]
        path = self.candidate / view['depth_samples']
        samples = np.load(path, allow_pickle=False)
        samples['joint_z_m'][samples['joint_covered']] += 1000.
        write_samples(self.candidate, view, samples)
        write_json(self.candidate / 'report.json', self.report)
        with self.assertRaisesRegex(ValueError, 'depth L1'):
            compare(self.baseline, self.candidate, self.output)
        self.assertFalse(self.output.exists())

        samples['joint_covered'][:] = False
        samples['joint_z_m'][:] = 0.
        view['joint']['metrics']['depth_pixels_used'] = 0
        write_samples(self.candidate, view, samples)
        write_json(self.candidate / 'report.json', self.report)
        with self.assertRaisesRegex(ValueError, 'depth L1'):
            compare(self.baseline, self.candidate, self.output)
        self.assertFalse(self.output.exists())
        view['joint']['metrics']['depth_l1_m'] = None
        write_json(self.candidate / 'report.json', self.report)
        result = compare(self.baseline, self.candidate, self.output)
        self.assertIsNone(result['views'][1]['joint']['depth']['common']['candidate_l1_m'])

    def test_coverage_requires_a_valid_selection_record_for_each_update(self):
        for case, edit in (
            ('no record', lambda steps: steps[0].pop('seed_selection')),
            ('no timing field', lambda steps: steps[0]['seed_selection'].pop('prepass_seconds')),
            ('first frame prepass', lambda steps: steps[0]['seed_selection'].update(prepass_seconds=.001)),
            ('budget mismatch', lambda steps: steps[1]['seed_selection'].update(depth_candidates=999)),
            ('missing prepass', lambda steps: steps[1]['seed_selection'].update(prepass_seconds=None)),
            ('too many holes', lambda steps: steps[1]['seed_selection'].update(uncovered_candidates=1201)),
            ('wrong quota', lambda steps: steps[1]['seed_selection'].update(reserved_uncovered=499)),
            ('unused quota claimed', lambda steps: steps[2]['seed_selection'].update(reserved_uncovered=1)),
        ):
            document = copy.deepcopy(self.report)
            edit(document['submaps'][1]['training'])
            write_json(self.candidate / 'report.json', document)
            output = self.root / f'{case.replace(" ", "-")}.json'
            with self.subTest(case=case):
                with self.assertRaisesRegex(ValueError, 'seed selection'):
                    compare(self.baseline, self.candidate, output)
                self.assertFalse(output.exists())

    def test_comparison_must_create_a_new_file(self):
        self.output.write_text('previous')
        with self.assertRaises((ValueError, FileExistsError)):
            compare(self.baseline, self.candidate, self.output)
        self.assertEqual(self.output.read_text(), 'previous')


if __name__ == '__main__':
    unittest.main()
