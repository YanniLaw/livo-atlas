import gc
from pathlib import Path
import tempfile
import socket
import threading
import time
import subprocess
import sys
import unittest
from unittest import mock

import numpy as np
import torch

from joint_renderer import infer_blocks
from online_model import checked_cuda_device
from online_protocol import encode_message, read_message, write_message
from online_worker import serve_connection
from render_actor_fixtures import METADATA, RenderActor, training_state
from render_fixtures import ASSET_FIELDS, TARGET_FIELDS
from state_fixtures import pose
from online_model import OnlineGaussianModel
from submap_fixtures import SESSION, control_fixture
import rasterizer
import online_render


class OnlineRenderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = checked_cuda_device()
        torch.set_num_threads(1)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='livo-online-render-')
        self.addCleanup(self.temp.cleanup)
        self.fixture = RenderActor(Path(self.temp.name), self.device)
        self.addCleanup(self.fixture.release)
        self.fixture.provision()
        self.actor = self.fixture.actor

    def test_worker_inference_import_does_not_require_offline_yaml(self):
        modules = Path(__file__).resolve().parents[2] / 'tools/3dgs_probe'
        code = ('import sys; sys.path.insert(0, ' + repr(str(modules)) +
                "); import online_render; assert 'yaml' not in sys.modules")
        result = subprocess.run([sys.executable, '-I', '-c', code],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_asset_query_excludes_third_resident_and_preserves_training_state(self):
        before = training_state(self.actor)
        model, optimizer = self.actor.model, self.actor.model.optimizer
        header, body = self.fixture.request()
        real = rasterizer.render
        counts = []

        def checked(*args, **kwargs):
            self.assertFalse(torch.is_grad_enabled())
            self.assertTrue(all(not value.requires_grad for value in args[:5]))
            counts.append(args[0].shape[0])
            return real(*args, **kwargs)

        with mock.patch.object(rasterizer, 'render', side_effect=checked):
            reply, rgb = self.actor.handle(header, body)
        self.assertEqual(reply['type'], 'joint_render_ok')
        self.assertEqual([b['target']['submap_id'] for b in reply['blocks']], ['1', '2'])
        self.assertEqual(reply['query_keyframe_id'], '6')
        self.assertEqual(len(rgb), 3 * 8 * 6)
        self.assertEqual(counts, [2, 3])
        self.assertGreater(reply['changed_pixels'], 0)
        self.assertEqual(training_state(self.actor), before)
        self.assertIs(self.actor.model, model)
        self.assertIs(self.actor.model.optimizer, optimizer)
        continued, _ = self.fixture.view(2)
        self.assertEqual(continued['steps'], 20)
        self.assertTrue(continued['nonzero_gradient'])
        self.assertTrue(continued['parameter_updated'])

    def test_dirty_resident_rejects_its_still_referenced_previous_checkpoint(self):
        root = Path(self.temp.name) / 'dirty'
        root.mkdir(mode=0o700)
        fixture = RenderActor(root, self.device)
        self.addCleanup(fixture.release)
        fixture.activate(0, 0, 2)
        fixture.view(0)
        asset = fixture.checkpoint()
        fixture.view(1)
        fixture.acknowledge()
        header = dict(schema=2, type='joint_render', session_id=SESSION, job_id=str(fixture.job()),
            generation=str(fixture.generation), pose_version='0', map_sequence=str(fixture.sequence),
            frame_count='2', optimized_frame_count='0', parent_frame='odom', query_keyframe_id='0',
            query_submap_id='0', neighbor_reason='no_previous_group', blocks=[dict(
                target={key: fixture.target[key] for key in TARGET_FIELDS}, resident=0,
                content={key: asset[key] for key in ('submap_id', 'source_generation', 'content_job_id')},
                asset={key: asset[key] for key in ASSET_FIELDS}, frame_ids=['0'], gaussian_count=1)])
        body = pose(0).astype('<f8').tobytes() + fixture.target_body
        self.assertIn(int(asset['asset_id']), fixture.actor.references)
        before = training_state(fixture.actor)
        with self.assertRaisesRegex(ValueError, 'superseded'):
            fixture.actor.handle(header, body)
        self.assertEqual(training_state(fixture.actor), before)

    def test_query_cannot_predate_a_newer_known_confirmation(self):
        header, body = self.fixture.request()
        header.update(blocks=header['blocks'][:1], query_submap_id='1', query_keyframe_id='3',
                      neighbor_reason='neighbor_unavailable')
        body = pose(3).astype('<f8').tobytes() + body[128:384]
        with self.assertRaisesRegex(ValueError, 'known confirmation'):
            self.actor.handle(header, body)

    def test_old_capture_and_new_projected_query_do_not_advance_training_frontier(self):
        self.fixture.acknowledge(version=1)
        before = training_state(self.actor)
        header, body = self.fixture.request(version=1)
        reply, _ = self.actor.handle(header, body)
        self.assertEqual(reply['pose_version'], '1')
        self.assertEqual(reply['optimized_frame_count'], '6')
        np.testing.assert_allclose(np.array(reply['camera_from_parent']).reshape(4, 4),
                                   [[0, -1, 0, 11], [1, 0, 0, -4], [0, 0, 1, 3], [0, 0, 0, 1]], atol=1e-9)
        self.assertEqual(training_state(self.actor), before)
        self.assertEqual(self.actor._max_version, 0)

    def test_discarded_repair_fence_can_render_restored_resident_confirmation(self):
        neighbor_generation = self.fixture.targets[1]['generation']
        self.fixture.checkpoint()
        self.fixture.activate(2, 5, 2, asset=self.fixture.assets[2])
        self.fixture.activate(1, 3, 2, asset=self.fixture.assets[1], accept=False)
        before = training_state(self.actor)
        header, body = self.fixture.request()
        block = header['blocks'][-1]
        block.update(target={key: self.fixture.targets[2][key] for key in TARGET_FIELDS},
                     content=dict(self.actor.content), resident=1)
        block.pop('asset')
        header['blocks'][0]['target']['generation'] = neighbor_generation
        reply, _ = self.actor.handle(header, body)
        self.assertEqual(reply['type'], 'joint_render_ok')
        self.assertEqual(reply['generation'], '6')
        self.assertEqual(reply['blocks'][-1]['target']['generation'], '5')
        self.assertEqual(training_state(self.actor), before)

    def test_worker_timing_includes_both_real_metric_evaluations(self):
        real = online_render.image_metrics
        def measured(*args):
            time.sleep(.1)
            return real(*args)
        with mock.patch.object(online_render, 'image_metrics', side_effect=measured):
            reply, _ = self.actor.handle(*self.fixture.request())
        self.assertGreaterEqual(reply['worker_seconds'], .2)

    def test_online_inference_preserves_allocator_history_and_releases_its_tensors(self):
        sentinel = torch.empty(4 * 1024 * 1024, dtype=torch.uint8, device=self.device)
        del sentinel
        torch.cuda.synchronize()
        peak, reserved = torch.cuda.max_memory_allocated(), torch.cuda.memory_reserved()
        allocated = torch.cuda.memory_allocated()
        header, body = self.fixture.request()
        self.actor.handle(header, body)
        gc.collect()
        torch.cuda.synchronize()
        self.assertGreaterEqual(torch.cuda.max_memory_allocated(), peak)
        self.assertGreaterEqual(torch.cuda.memory_reserved(), reserved)
        self.assertEqual(torch.cuda.memory_allocated(), allocated)

    def test_real_allocator_oom_preserves_model_then_allows_training(self):
        before = training_state(self.actor)
        header, body = self.fixture.request()

        def exhaust(*args, **kwargs):
            # Above the existing 40% limit on the pinned 8GiB GPU; no physical fill.
            torch.empty(4 * 2**30, dtype=torch.uint8, device=self.device)
            raise AssertionError('Controlled allocator request unexpectedly fit')

        with mock.patch.object(rasterizer, 'render', side_effect=exhaust):
            reply, rgb = self.actor.handle(header, body)
        self.assertEqual(reply['type'], 'joint_render_error')
        self.assertEqual(reply['code'], 'oom')
        self.assertEqual(rgb, b'')
        self.assertFalse(self.actor.failed)
        self.assertEqual(training_state(self.actor), before)
        continued, _ = self.fixture.view(2)
        self.assertTrue(continued['nonzero_gradient'])
        self.assertTrue(continued['parameter_updated'])

    def test_oom_unicode_diagnostic_remains_bounded_and_encodable(self):
        text = 'CUDA \U0001f9e0 failed: ' + '\0\udcff\U0001f9e0' * 600
        with mock.patch.object(rasterizer, 'render', side_effect=torch.cuda.OutOfMemoryError(text)):
            reply, rgb = self.actor.handle(*self.fixture.request())
        self.assertEqual(reply['code'], 'oom')
        self.assertLessEqual(len(reply['error'].encode('utf-8')), 4096)
        self.assertTrue(encode_message(reply, rgb))
        self.assertIn('CUDA', reply['error'])
        self.assertFalse(self.actor.failed)

    def test_bad_source_is_fatal(self):
        header, body = self.fixture.request()
        header['blocks'][0]['asset']['manifest_sha256'] = '0' * 64
        with self.assertRaises(ValueError):
            self.actor.handle(header, body)
        self.assertTrue(self.actor.failed)

    def test_pending_candidate_cannot_be_rendered(self):
        self.fixture.view(2, accept=False)
        with self.assertRaisesRegex(ValueError, 'pending candidate'):
            self.actor.handle(*self.fixture.request())
        self.assertTrue(self.actor.failed)

    def test_unnegotiated_render_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = RenderActor(Path(directory), self.device, enabled=False)
            try:
                fixture.provision()
                with self.assertRaisesRegex(ValueError, 'negotiated'):
                    fixture.actor.handle(*fixture.request())
                self.assertTrue(fixture.actor.failed)
            finally:
                fixture.release()

    def test_real_socket_negotiates_only_requested_capability(self):
        for enabled in (False, True):
            with tempfile.TemporaryDirectory() as directory:
                client, server = socket.socketpair()
                result = []
                factory = lambda: (OnlineGaussianModel(self.device), METADATA)
                worker = threading.Thread(target=lambda: result.append(
                    serve_connection(server, model_factory=factory)))
                worker.start()
                try:
                    hello = dict(schema=2, type='hello', session_id=SESSION,
                                 state_directory=str(Path(directory) / 'states'))
                    if enabled:
                        hello['capabilities'] = ['joint_render_v1']
                    write_message(client, hello, b'', 5)
                    reply, _ = read_message(client, 5)
                    self.assertEqual(reply['type'], 'hello_ok')
                    self.assertEqual(reply.get('capabilities'),
                                     ['joint_render_v1'] if enabled else None)
                    write_message(client, control_fixture('stop', job=1, generation=0, map_sequence='0'), b'', 5)
                    self.assertEqual(read_message(client, 5)[0]['type'], 'stopped')
                finally:
                    client.close()
                    worker.join(10)
                    server.close()
                self.assertEqual(result, [True])


if __name__ == '__main__':
    unittest.main()
