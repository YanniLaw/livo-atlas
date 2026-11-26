import argparse
import json
from pathlib import Path
import tempfile
import shutil
import subprocess
import sys
import unittest
from unittest import mock

import numpy as np
import yaml

import evaluate_heldout_snapshot as tool
from evaluation_metrics import measure
from heldout_fixtures import make_heldout_snapshot


class SnapshotScoringTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.directory, self.keyframes, self.split, self.views = make_heldout_snapshot(self.root)
        self.config = self.root / 'config.yaml'
        self.config.write_text(yaml.safe_dump(dict(gaussian=dict(evaluation=dict(
            max_output_mib=16, min_free_disk_mib=0), joint_render_policy='temporal'))))
        self.preparer = self.root / 'prepare'
        self.preparer.write_text('fixture')
        self.args = argparse.Namespace(map_directory=self.directory, keyframes=self.keyframes,
                                       output=self.root/'output', config=self.config,
                                       prepare_executable=self.preparer)

    def render(self, blocks, K, width, height, device):
        return dict(rgb=self.views[1]['rgb'].copy(), alpha=np.ones((height, width), 'f4'),
                    depth_m=self.views[1]['depth_m'].copy(), gaussian_count=len(blocks))

    def execute(self, renderer=None):
        with mock.patch.object(tool, 'initialize_cuda', return_value=('fake', {})), \
                mock.patch.object(tool, 'prepare_group_queries', return_value={1: self.views[1]}):
            return tool.run(self.args, renderer=renderer or self.render)

    def test_saved_float_samples_reproduce_metrics_without_training(self):
        report = self.execute()
        self.assertEqual(report['status'], 'succeeded')
        self.assertEqual(report['phase'], 'final_snapshot')
        self.assertEqual(report['queries'][0]['status'], 'succeeded')
        self.assertEqual(report['counts'], dict(planned=1, succeeded=1, skipped=0, failed=0))
        with np.load(self.args.output/'views/1/samples.npz', allow_pickle=False) as arrays:
            query = dict(rgb=arrays['reference_rgb'], depth_m=arrays['reference_depth_m'],
                         image_mask=arrays['image_mask'], depth_mask=arrays['depth_mask'])
            result = {key: arrays['single_'+key] for key in ('rgb', 'alpha', 'depth_m')}
            self.assertEqual(measure(result, query), report['queries'][0]['single'])
        self.assertEqual(json.loads((self.args.output/'report.json').read_text()), report)

    def test_budget_and_policy_fail_before_cuda_or_output(self):
        for key, value in (('max_output_mib', 0), ('max_output_mib', 1),
                           ('min_free_disk_mib', 2**31), ('min_free_disk_mib', True)):
            config = dict(gaussian=dict(evaluation={key: value}))
            self.config.write_text(yaml.safe_dump(config))
            with self.subTest(key=key, value=value), mock.patch.object(tool, 'initialize_cuda') as cuda:
                with self.assertRaises(ValueError):
                    tool.run(self.args)
                cuda.assert_not_called()
                self.assertFalse(self.args.output.exists())
        self.config.write_text(yaml.safe_dump(dict(gaussian=dict(joint_render_policy='spatial'))))
        with self.assertRaisesRegex(ValueError, 'spatial'):
            tool.run(self.args)
        self.assertFalse(self.args.output.exists())

    def test_unproven_or_changed_input_fails_before_cuda(self):
        (self.keyframes/'frames/1/image.png').write_bytes(b'changed')
        with mock.patch.object(tool, 'initialize_cuda') as cuda:
            with self.assertRaises(ValueError):
                tool.run(self.args)
            cuda.assert_not_called()
        self.assertFalse(self.args.output.exists())

    def test_multiple_queries_all_receive_terminal_failure_records(self):
        root = self.root / 'multiple'
        root.mkdir()
        directory, keyframes, _, views = make_heldout_snapshot(root, blocks=5)
        self.args.map_directory, self.args.keyframes = directory, keyframes
        with mock.patch.object(tool, 'initialize_cuda', side_effect=RuntimeError('controlled CUDA failure')):
            with self.assertRaises(RuntimeError):
                tool.run(self.args)
        report = json.loads((self.args.output/'report.json').read_text())
        self.assertEqual([row['keyframe_id'] for row in report['queries']], [1, 9])
        self.assertEqual(report['counts']['failed'], 2)

    def test_missing_owner_is_explicit_skip_and_never_initializes_cuda(self):
        map_file = self.directory / 'map.json'
        document = json.loads(map_file.read_text())
        row = document['entries'][0]
        for key in list(row):
            if key in ('content', 'content_source', 'trained_frame_ids', 'gaussian_count') or key.startswith('asset'):
                row.pop(key)
        row.update(availability='missing', resident=0, volatile=0)
        shutil.rmtree(self.directory / 'states/1')
        map_file.write_text(json.dumps(document))
        final = self.directory / 'finalization.json'
        report = json.loads(final.read_text())
        report.update(current=2, missing=1, eligible=2, eligible_submap_ids=['1', '2'], asset_ids=['2', '3'])
        final.write_text(json.dumps(report))
        with mock.patch.object(tool, 'initialize_cuda') as cuda:
            report = tool.run(self.args)
            cuda.assert_not_called()
        self.assertEqual(report['status'], 'completed_with_skips')
        self.assertEqual(report['counts']['skipped'], 1)
        self.assertEqual(report['queries'][0]['reason'], 'missing_model')
        self.assertIsNone(report['summary']['single'])

    def test_artifact_failure_does_not_mark_query_succeeded(self):
        with mock.patch.object(tool, '_save_samples', side_effect=OSError('controlled write failure')):
            with self.assertRaises(OSError):
                self.execute()
        report = json.loads((self.args.output/'report.json').read_text())
        self.assertEqual(report['counts']['failed'], 1)

    def test_budget_preflight_failure_carries_all_trustworthy_frozen_queries(self):
        self.config.write_text(yaml.safe_dump(dict(gaussian=dict(evaluation=dict(max_output_mib=1)))))
        with self.assertRaises(ValueError) as caught:
            tool.run(self.args)
        self.assertTrue(hasattr(caught.exception, 'report'))
        report = caught.exception.report
        self.assertEqual(report['counts']['failed'], 1)
        self.assertEqual(report['queries'][0]['keyframe_id'], 1)
        self.assertEqual(report['queries'][0]['phase'], 'final_snapshot')
        self.assertFalse(self.args.output.exists())

    def test_publication_mutation_invalidates_scores_and_removes_pending_marker(self):
        write = tool._exclusive_write

        def mutate(path, raw):
            write(path, raw)
            if path.name == 'report.pending.json':
                (self.directory/'status.json').write_text('{}')

        with mock.patch.object(tool, '_exclusive_write', side_effect=mutate):
            with self.assertRaises(ValueError):
                self.execute()
        report = json.loads((self.args.output/'report.json').read_text())
        self.assertEqual(report['counts']['failed'], 1)
        self.assertNotIn('single', report['queries'][0])
        self.assertEqual(report['artifacts'], {})
        self.assertFalse((self.args.output/'report.pending.json').exists())

    def test_disk_reserve_is_rechecked_before_publication(self):
        calls = 0

        def space(path):
            nonlocal calls
            calls += 1
            return shutil._ntuple_diskusage(100*_MIB, 0, 100*_MIB if calls <= 2 else 0)

        _MIB = 1024*1024
        self.config.write_text(yaml.safe_dump(dict(gaussian=dict(evaluation=dict(min_free_disk_mib=1)))))
        with mock.patch.object(tool.shutil, 'disk_usage', side_effect=space):
            with self.assertRaises(ValueError):
                self.execute()
        report = json.loads((self.args.output/'report.json').read_text())
        self.assertEqual(report['status'], 'failed')

    def test_cli_preflight_failure_emits_parseable_complete_ledger_without_cuda(self):
        self.config.write_text(yaml.safe_dump(dict(gaussian=dict(evaluation=dict(max_output_mib=1)))))
        result = subprocess.run([sys.executable, tool.__file__, '--map-directory', str(self.directory),
            '--keyframes', str(self.keyframes), '--prepare-executable', str(self.preparer),
            '--output', str(self.args.output), '--config', str(self.config)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 1)
        report = json.loads(result.stderr)
        self.assertEqual(report['stage'], 'preflight')
        self.assertTrue(report['plan_verified'])
        self.assertEqual(report['counts']['failed'], 1)
        self.assertFalse(self.args.output.exists())

    def test_failed_report_write_keeps_full_failure_plan_on_exception(self):
        write = tool._exclusive_write

        def fail_reports(path, raw):
            if path.name.startswith('report'):
                raise OSError('controlled full disk')
            write(path, raw)

        with mock.patch.object(tool, '_exclusive_write', side_effect=fail_reports):
            with self.assertRaises(OSError) as caught:
                self.execute()
        self.assertEqual(caught.exception.report['counts']['failed'], 1)
        self.assertEqual(caught.exception.report['artifacts'], {})

    def test_initial_archive_write_failure_still_has_terminal_plan(self):
        with mock.patch.object(tool, '_exclusive_write', side_effect=OSError('controlled initial write failure')):
            with self.assertRaises(OSError) as caught:
                tool.run(self.args)
        self.assertEqual(caught.exception.report['counts']['failed'], 1)
        self.assertEqual(caught.exception.report['queries'][0]['phase'], 'final_snapshot')

    def test_render_failure_has_terminal_record_for_every_query(self):
        def broken(*args):
            raise RuntimeError('controlled failure')

        with self.assertRaises(RuntimeError):
            self.execute(broken)
        report = json.loads((self.args.output/'report.json').read_text())
        self.assertEqual(report['status'], 'failed')
        self.assertEqual(report['counts']['failed'], 1)
        self.assertEqual(report['queries'][0]['status'], 'failed')
        self.assertIsNone(report['summary'])


if __name__ == '__main__':
    unittest.main()
