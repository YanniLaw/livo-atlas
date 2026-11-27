import copy
import unittest

import numpy as np

import diagnose_depth_candidates as diagnostic


class CandidateSupportTest(unittest.TestCase):
    def query(self):
        return dict(image_mask=np.ones((1, 5), bool),
                    depth_mask=np.array([[True, True, True, True, False]]),
                    depth_m=np.full((1, 5), 2.))

    def test_backprojection_preserves_masks_and_nonidentity_camera(self):
        view = dict(image_mask=np.array([[True, False, True]]),
                    depth_mask=np.array([[True, True, True]]), depth_m=np.full((1, 3), 2.),
                    K=np.eye(3), split='train')
        camera = np.eye(4)
        camera[0, 3] = 1
        points = diagnostic.candidate_points(view, camera)
        self.assertEqual(points.dtype, np.dtype('f4'))
        np.testing.assert_array_equal(points, [[-1., 0, 2.], [3., 0, 2.]])
        view['split'] = 'validation'
        with self.assertRaises(ValueError): diagnostic.candidate_points(view, camera)

    def test_chunking_counts_every_point_including_final_partial_chunk(self):
        points = np.tile(np.array([[0., 0., 2.]]), (64001, 1))
        maps = diagnostic.accumulate(points, np.eye(4), np.eye(3), self.query())
        self.assertEqual(maps[0]['any'][0, 0], 64001)
        self.assertEqual(maps[2]['any'][0, 2], 64001)
        self.assertEqual(maps[2]['any'][0, 3], 0)
        self.assertEqual(maps[4]['consistent'][0, 4], 0)
        self.assertEqual(maps[4]['any'][0, 4], 64001)

    def support(self):
        # One fixed hole each: retained sample, retired sample, unsampled input,
        # no input, and unknown-depth retained input. Alpha covers pixel 0.
        points = dict(input_all=[0, 1, 2, 4], input_retained=[0, 2, 4],
                      all_initial=[0, 1], retained_initial=[0])
        return {key: diagnostic.accumulate(np.array([[2.*x, 0, 2.] for x in xs]),
                    np.eye(4), np.eye(3), self.query()) for key, xs in points.items()}

    def test_sampling_and_retention_have_separate_fixed_hole_denominators(self):
        result = diagnostic.candidate_summary(self.query(), np.array([[.1, 0, 0, 0, 0.]]), self.support())
        self.assertEqual(result['holes'], 4)
        self.assertEqual(result['hole_depth_known'], 3)
        self.assertEqual(result['hole_depth_unknown'], 1)
        self.assertEqual(result['support']['input_all']['0']['labelled_holes'],
                         dict(consistent=2, front_only=0, behind_only=0, mixed=0, none=1))
        pairs = result['contrasts']
        self.assertEqual(pairs['sampling_all']['0'], dict(holes_losing_neighbors=2, known_holes_losing_consistent_neighbors=1))
        self.assertEqual(pairs['sampling_retained']['0']['holes_losing_neighbors'], 2)
        self.assertEqual(pairs['input_retention']['0']['holes_losing_neighbors'], 1)
        self.assertEqual(pairs['seed_retention']['0']['holes_losing_neighbors'], 1)

    def test_invalid_subset_and_invalid_alpha_cannot_report_sampling_loss(self):
        q = self.query()
        support = self.support()
        support['retained_initial'][0]['any'][0, 3] = 1
        support['retained_initial'][0]['consistent'][0, 3] = 1
        with self.assertRaisesRegex(ValueError, 'subset'):
            diagnostic.candidate_summary(q, np.zeros((1, 5)), support)
        with self.assertRaises(ValueError):
            diagnostic.candidate_summary(q, np.full((1, 5), np.nan), self.support())
        q['image_mask'][0, 0] = False
        with self.assertRaises(ValueError):
            diagnostic.accumulate(np.array([[0., 0, 2.]]), np.eye(4), np.eye(3), q)


if __name__ == '__main__':
    unittest.main()


class CandidateCliTest(unittest.TestCase):
    def fixture(self):
        import argparse
        from dataclasses import asdict
        import json
        import os
        from pathlib import Path
        import tempfile
        import yaml
        from heldout_fixtures import make_heldout_snapshot
        from heldout_query import canonical_query, load_evaluation_snapshot
        from heldout_selection import EvaluationSelector
        from evaluate_gaussian_sampling import admit_training_view
        from evaluate_gaussian_submaps import camera_from_final_parent
        from evaluate_heldout_snapshot import load_settings
        from prepare_holdout_split import file_sha
        from render_gaussian_map import _sources_report
        from train import initialize_gaussians

        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        directory, keyframes, split, views = make_heldout_snapshot(root, blocks=2)
        source = root/'diagnostic'
        source.mkdir()
        config = source/'config.yaml'
        config.write_text(yaml.safe_dump(dict(gaussian=dict(joint_render_policy='spatial', worker=dict(
            seed_policy='coverage', coverage_sampling='replace', coverage_uncovered_ratio=.25),
            evaluation=dict(max_output_mib=64, min_free_disk_mib=1024)))))
        args = argparse.Namespace(diagnostic_directory=source, map_directory=directory, keyframes=keyframes,
            prepare_executable=Path(os.environ['LIVO_PREPARE_BIN']),
            selector_executable=Path(os.environ['LIVO_SELECTOR_BIN']), output=root/'out')
        snapshot = load_evaluation_snapshot(directory, keyframes)
        selector = EvaluationSelector(snapshot, 'spatial', args.selector_executable)
        choice = selector.select(snapshot.split.frames[1])
        choice['selection'].pop('selection_seconds', None)
        train = admit_training_view(split, dict(views[0], split='train'))
        camera = camera_from_final_parent(train, snapshot.poses[0]) @ snapshot.poses[0]
        means = initialize_gaussians([dict(train, camera_from_parent=camera)], 1000)['means']
        query = canonical_query(split, views[1])
        arrays = dict(reference_rgb=query['rgb'], reference_depth_m=query['depth_m'],
            image_mask=query['image_mask'], depth_mask=query['depth_mask'], K=query['K'],
            camera_from_parent=query['camera_from_body'] @ np.linalg.inv(snapshot.poses[1]),
            block_0_anchor=snapshot.poses[0], block_0_all_initial_means=means, block_0_initial_means=means,
            block_0_all_origin_ids=np.zeros(len(means), dtype='i8'), block_0_origin_ids=np.zeros(len(means), dtype='i8'),
            block_0_prepass_0=np.array([], dtype='u1'), single_trained_alpha=np.zeros_like(query['depth_m']))
        np.savez_compressed(source/'samples.npz', **arrays)
        (source/'split.json').write_bytes(split.payload)
        (source/'poses.f64le').write_bytes(np.asarray(snapshot.poses, dtype='<f8').tobytes())
        (source/'single.png').write_bytes(b'previous image receipt')
        artifacts = {name: dict(bytes=(source/name).stat().st_size, sha256=file_sha(source/name))
                     for name in ('samples.npz', 'config.yaml', 'split.json', 'poses.f64le', 'single.png')}
        report = dict(schema=1, kind='livo_atlas.gaussian_coverage_diagnostic', status='succeeded',
            keyframe_id=1, owner_submap_id=0, selected_ids=[0], selection=choice, selection_policy='spatial',
            selector_runtime_sha256=selector.runtime_sha256, random_seed=0, seed_policy='coverage',
            training_options=asdict(load_settings(config).worker), split_id=split.split_id,
            sources=_sources_report(snapshot), source_sha256=snapshot.source_digests,
            config_sha256=file_sha(config), preparer_sha256=file_sha(args.prepare_executable),
            final_poses_sha256=file_sha(source/'poses.f64le'), groups=[dict(submap_id=0, anchor_id=0,
            retained_frame_ids=[0], training=[dict(keyframe_id=0, seed_selection=dict(depth_candidates=len(means)))])],
            artifacts=artifacts)
        (source/'report.json').write_text(json.dumps(report))

        def prepared(executable, source, output, group, source_hash):
            return {key: dict(views[key], split='train' if key in group['train_ids'] else 'validation')
                    for key in group['train_ids']+group['held_out_ids']}
        return args, prepared

    def test_cpu_cli_reuses_source_and_reports_no_training(self):
        import json
        from unittest import mock
        args, prepared = self.fixture()
        with mock.patch.object(diagnostic, 'prepare_group', side_effect=prepared):
            report = diagnostic.run(args)
        self.assertEqual(report['status'], 'succeeded')
        self.assertEqual(report['selected_ids'], [0])
        self.assertEqual(report['new_training_calls'], 0)
        self.assertEqual(report['new_render_calls'], 0)
        self.assertEqual(report['groups'][0]['candidates'], report['groups'][0]['sampled'])
        self.assertEqual(json.loads((args.output/'report.json').read_text())['summary'], report['summary'])
        with np.load(args.output/'counts.npz', allow_pickle=False) as data:
            self.assertGreater(int(data['block_0_input_all_r0_any'].sum()), 0)

    def test_bad_receipt_and_insufficient_disk_fail_before_output(self):
        from unittest import mock
        args, prepared = self.fixture()
        (args.diagnostic_directory/'single.png').write_bytes(b'changed')
        with self.assertRaises(ValueError): diagnostic.run(args)
        self.assertFalse(args.output.exists())
        args, prepared = self.fixture()
        with mock.patch.object(diagnostic.shutil, 'disk_usage', return_value=mock.Mock(free=1024**3)):
            with self.assertRaisesRegex(ValueError, 'reserve'): diagnostic.run(args)
        self.assertFalse(args.output.exists())

    def test_role_or_seed_lineage_change_cannot_publish(self):
        import json
        from unittest import mock
        for mutation in ('role', 'seed'):
            args, prepared = self.fixture()
            if mutation == 'role':
                path = args.diagnostic_directory/'report.json'
                report = json.loads(path.read_text())
                report['groups'][0]['training'][0]['keyframe_id'] = 1
                path.write_text(json.dumps(report))
            else:
                path = args.diagnostic_directory/'samples.npz'
                with np.load(path, allow_pickle=False) as source:
                    values = {key: source[key] for key in source.files}
                values['block_0_all_initial_means'] += 1
                np.savez_compressed(path, **values)
                report_path = args.diagnostic_directory/'report.json'
                report = json.loads(report_path.read_text())
                from prepare_holdout_split import file_sha
                report['artifacts']['samples.npz'] = dict(bytes=path.stat().st_size, sha256=file_sha(path))
                report_path.write_text(json.dumps(report))
            with self.subTest(mutation=mutation), mock.patch.object(diagnostic, 'prepare_group', side_effect=prepared):
                with self.assertRaises(ValueError): diagnostic.run(args)
            self.assertFalse((args.output/'report.json').exists())

    def test_late_input_artifact_pending_runtime_and_disk_changes_prevent_success(self):
        from unittest import mock
        for mutation in ('input', 'artifact', 'pending', 'runtime', 'disk'):
            args, prepared = self.fixture()
            late = False
            write = diagnostic._exclusive_write
            libraries = diagnostic.EvaluationSelector._runtime_libraries

            def resolved(selector):
                return libraries(selector)+[args.output.parent/'late.so'] if late and mutation == 'runtime' else libraries(selector)

            def changed(path, payload):
                nonlocal late
                write(path, payload)
                if path.name != '.report.pending.json': return
                late = True
                if mutation == 'input':
                    target = args.diagnostic_directory/'report.json'
                    target.write_bytes(target.read_bytes()+b'\n')
                if mutation in ('artifact', 'pending'):
                    target = args.output/'counts.npz' if mutation == 'artifact' else path
                    value = target.read_bytes()
                    target.write_bytes(b'x'+value[1:])
                if mutation == 'runtime': (args.output.parent/'late.so').write_bytes(b'changed resolution')

            with self.subTest(mutation=mutation), mock.patch.object(diagnostic, 'prepare_group', side_effect=prepared), \
                    mock.patch.object(diagnostic, '_exclusive_write', side_effect=changed), \
                    mock.patch.object(diagnostic.EvaluationSelector, '_runtime_libraries', resolved), \
                    mock.patch.object(diagnostic.shutil, 'disk_usage', side_effect=lambda _: mock.Mock(
                        free=0 if late and mutation == 'disk' else 10*1024**3)):
                with self.assertRaises(ValueError): diagnostic.run(args)
                self.assertFalse((args.output/'report.json').exists())
                self.assertTrue((args.output/'failure.json').exists())
