"""Real small frozen archives exercise the complete CPU diagnostic boundary."""

import copy
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest import mock

import numpy as np
import yaml

from heldout_query import load_evaluation_snapshot
from render_gaussian_map import _sources_report
from sampling_fixtures import make_sampling_pair
from prepare_holdout_split import file_sha
from study_verification import load_study_inputs, verify_inputs, verify_splits_and_static
from diagnose_texture import diagnose


def write_json(path, value):
    path.write_text(json.dumps(value, allow_nan=False))
    return file_sha(path)


def receipt(path):
    return dict(bytes=path.stat().st_size, sha256=file_sha(path))


def make_study(base):
    reference = base/'reference'
    reference.mkdir()
    uniform, coverage = make_sampling_pair(reference)
    snapshot = load_evaluation_snapshot(reference/'map', reference/'keyframes')
    sources = _sources_report(snapshot)
    study, capture, candidates = (base/name for name in ('study', 'capture', 'candidates'))
    for root in (study, capture, candidates):
        root.mkdir()
    conditions = {f'{distribution}-{policy}': dict(distribution=distribution, policy=policy)
                  for distribution in ('flat', 'stratified') for policy in ('uniform', 'coverage')}
    old = json.loads((uniform/'report.json').read_text())
    setting = dict(reference=str(reference), calls=10, queries=2, static_query=9,
                   split_id=snapshot.split.split_id, sources=sources, source_sha256=snapshot.source_digests,
                   final_poses_sha256=hashlib.sha256(np.asarray(snapshot.poses, '<f8').tobytes()).hexdigest(),
                   preparer_sha256=old['preparer_sha256'], selector_runtime_sha256=old['selector_runtime_sha256'])
    protocol = dict(conditions=conditions, seeds=[0, 1], scenes={'Test': setting}, configs={})
    runs = {}
    for condition, values in conditions.items():
        for seed in (0, 1):
            name = f'Test-{condition}-{seed}'
            root = study/name
            shutil.copytree(uniform if values['policy'] == 'uniform' else coverage, root)
            report = json.loads((root/'report.json').read_text())
            options = report['training_options']
            options.update(seed_distribution=values['distribution'], coverage_sampling='replace', coverage_uncovered_ratio=.25)
            config = yaml.safe_dump(dict(gaussian=dict(worker=dict(seed_policy='coverage', **options))))
            (root/'config.yaml').write_text(config)
            (root/'poses.f64le').write_bytes(np.asarray(snapshot.poses, '<f8').tobytes())
            report.update(random_seed=seed, sources=sources, source_sha256=snapshot.source_digests,
                          config_sha256=file_sha(root/'config.yaml'), final_poses_sha256=setting['final_poses_sha256'])
            for group in report['submaps']:
                for operation in group['training']:
                    if values['policy'] == 'coverage':
                        operation['seed_selection'].update(uniform_uncovered=None, replaced_seeds=0)
            for row in report['queries']:
                key = row['keyframe_id']
                body_camera = np.asarray(snapshot.split.frames[key]['camera_from_body']).reshape(4, 4)
                row['query']['camera_from_parent'] = (body_camera @ np.linalg.inv(snapshot.poses[key])).ravel().tolist()
            runs[name] = dict(report_sha256=write_json(root/'report.json', report))
            config_name = 'Test-'+values['distribution']
            if config_name not in protocol['configs']:
                config_path = study/(config_name+'.yaml')
                config_path.write_text(config)
                protocol['configs'][config_name] = dict(path=str(config_path), sha256=file_sha(config_path), options=options.copy())
    cases, depth_runs = {}, {}
    base_run = study/'Test-flat-coverage-0'
    base_report = json.loads((base_run/'report.json').read_text())
    for seed in (0, 1):
        case = f'Test-{seed}'
        a, b = capture/case, candidates/case
        a.mkdir(); b.mkdir()
        for name in ('config.yaml', 'split.json', 'poses.f64le'):
            shutil.copyfile(base_run/name, a/name)
        for name in ('samples.npz', 'single.png', 'joint.png'):
            (a/name).write_bytes(b'frozen '+name.encode())
        shared = {key: copy.deepcopy(base_report[key]) for key in ('sources', 'source_sha256', 'split_id', 'preparer_sha256', 'selector_runtime_sha256')}
        shared.update(random_seed=seed, keyframe_id=9, selected_ids=[3, 4])
        doc = dict(schema=1, kind='livo_atlas.gaussian_coverage_diagnostic', status='succeeded',
                   config_sha256=file_sha(a/'config.yaml'), final_poses_sha256=file_sha(a/'poses.f64le'),
                   artifacts={path.name: receipt(path) for path in a.iterdir()}, **shared)
        capture_sha = write_json(a/'report.json', doc)
        (b/'counts.npz').write_bytes(b'counts')
        depth = dict(schema=1, kind='livo_atlas.depth_candidate_diagnostic', status='succeeded',
                     diagnostic_directory=str(a), diagnostic_report=receipt(a/'report.json'),
                     diagnostic_artifacts=doc['artifacts'], artifacts={'counts.npz': receipt(b/'counts.npz')}, **shared)
        depth_sha = write_json(b/'report.json', depth)
        cases[case] = dict(capture_report_sha256=capture_sha, depth_report_sha256=depth_sha)
        depth_runs[case] = dict(report_sha256=depth_sha)
    (study/'static-selections.npz').write_bytes(b'selections')
    (study/'reference_sampler.py').write_bytes(b'# frozen sampler\n')
    (capture/'audit_study.py').write_bytes(b'# frozen counter\n')
    depth_sha = write_json(candidates/'audit.json', dict(status='succeeded', runs=depth_runs))
    static_sha = write_json(study/'static-support.json', dict(kind='frozen_feedback_support_proxy', training_calls=0,
        render_calls=0, depth_audit_sha256=depth_sha, selections_sha256=file_sha(study/'static-selections.npz'), runs=cases))
    write_json(study/'static-audit.json', dict(status='succeeded', selections=152, runs=cases,
        static_report_sha256=static_sha, reference_sampler_sha256=file_sha(study/'reference_sampler.py'),
        independent_counter_sha256=file_sha(capture/'audit_study.py')))
    protocol_sha = write_json(study/'protocol.json', protocol)
    implementation_sha = write_json(study/'implementation.json', dict(files={}))
    review_sha = write_json(study/'review-verification.json', dict(status='succeeded'))
    write_json(study/'audit.json', dict(status='succeeded', protocol_sha256=protocol_sha,
        implementation_sha256=implementation_sha, review_verification_sha256=review_sha, calls=80, steps=1600, receipts=runs))
    write_json(study/'resume-complete.json', dict(status='succeeded', runs=runs))
    write_json(study/'comparison-audit.json', dict(status='succeeded', runs={}))
    write_json(study/'summary.json', dict(status='succeeded'))
    evidence = {name.replace('-', '_')+'_sha256': file_sha(study/(name+'.json')) for name in
                ('protocol', 'implementation', 'audit', 'static-audit', 'comparison-audit', 'summary', 'review-verification')}
    write_json(study/'final-evidence.json', dict(status='succeeded', successful_runs=8, successful_training_calls=80,
        successful_training_steps=1600, **evidence))
    return study, capture, candidates


class TextureDiagnosticTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.roots = make_study(self.base)

    def test_valid_archive_and_late_split_change(self):
        inputs = load_study_inputs(*self.roots)
        self.assertEqual(len(inputs['runs']), 8)
        verify_inputs(inputs)
        path = self.roots[0]/'Test-flat-uniform-0/split.json'
        path.write_bytes(path.read_bytes()+b' ')
        with self.assertRaises(ValueError):
            verify_inputs(inputs)
        with self.assertRaises(ValueError):
            verify_splits_and_static(*self.roots)

    def test_static_artifacts_and_bound_reports_cannot_change(self):
        paths = [self.roots[1]/'Test-0'/name for name in
                 ('samples.npz', 'single.png', 'joint.png', 'config.yaml', 'poses.f64le', 'split.json', 'report.json')]
        paths += [self.roots[2]/'Test-0/counts.npz', self.roots[2]/'Test-0/report.json',
                  self.roots[0]/'static-selections.npz', self.roots[1]/'audit_study.py']
        for path in paths:
            raw = path.read_bytes()
            path.write_bytes(raw+b' ')
            with self.subTest(path=path), self.assertRaises(ValueError):
                verify_splits_and_static(*self.roots)
            path.write_bytes(raw)

    def test_diagnostic_uses_eight_run_common_mask_and_exact_source_scores(self):
        output = self.base/'output'
        result = diagnose(*self.roots, output)
        self.assertEqual(result['status'], 'succeeded')
        self.assertEqual(result['new_training_calls'], 0)
        self.assertEqual(result['new_render_calls'], 0)
        metrics = json.loads((output/'metrics.json').read_text())
        self.assertEqual(len(metrics['runs']), 8)
        self.assertEqual(len(metrics['pairs']), 4)
        for run in metrics['runs'].values():
            self.assertEqual(run['summary']['single']['common']['all']['pixels'], 92)
            self.assertEqual(run['summary']['single']['image']['all']['pixels'], 96)
            self.assertIsNone(run['summary']['single']['common']['all']['ssim_rgb'])
        self.assertEqual(metrics['pairs']['Test-uniform-0']['summary']['single']['image_pixels'], 96)

    def test_snapshot_changes_duplicate_json_and_output_budget_are_rejected(self):
        inputs = load_study_inputs(*self.roots)
        source = self.base/'reference/keyframes/frames/0/image.png'
        raw = source.read_bytes()
        source.write_bytes(raw+b'x')
        with self.assertRaises(ValueError):
            verify_inputs(inputs)
        source.write_bytes(raw)
        protocol = self.roots[0]/'protocol.json'
        raw = protocol.read_bytes()
        protocol.write_bytes(b'{"conditions":{},"conditions":{}}')
        with self.assertRaises(ValueError):
            verify_splits_and_static(*self.roots)
        protocol.write_bytes(raw)
        with mock.patch('diagnose_texture.MAX_OUTPUT', 128):
            with self.assertRaises(ValueError):
                diagnose(*self.roots, self.base/'budget')
        self.assertFalse((self.base/'budget/report.json').exists())
        self.assertTrue((self.base/'budget/failure.json').exists())

    def test_rejects_overlap_and_low_disk_without_publishing(self):
        with self.assertRaises(ValueError):
            diagnose(*self.roots, self.roots[0]/'output')
        with mock.patch('diagnose_texture.shutil.disk_usage', return_value=shutil._ntuple_diskusage(100, 90, 10)):
            with self.assertRaises(ValueError):
                diagnose(*self.roots, self.base/'output')
        self.assertFalse((self.base/'output/report.json').exists())

    def test_rechecks_inputs_metrics_and_pending_report_before_publication(self):
        import diagnose_texture
        for target in ('source', 'metrics', 'pending'):
            output = self.base/target
            original = diagnose_texture.verify_inputs
            changed = []
            def mutate(inputs):
                original(inputs)
                if changed:
                    return
                changed.append(True)
                if target == 'source':
                    path = self.roots[0]/'Test-flat-uniform-0/split.json'
                elif target == 'metrics':
                    path = output/'metrics.json'
                else:
                    path = output/'report.pending'
                raw = path.read_bytes()
                path.write_bytes(raw+b' ')
            source = self.roots[0]/'Test-flat-uniform-0/split.json'
            raw = source.read_bytes()
            with mock.patch('diagnose_texture.verify_inputs', side_effect=mutate):
                with self.subTest(target=target), self.assertRaises(ValueError):
                    diagnose(*self.roots, output)
            self.assertFalse((output/'report.json').exists())
            self.assertTrue((output/'failure.json').exists())
            source.write_bytes(raw)


if __name__ == '__main__':
    unittest.main()
