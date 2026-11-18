"""Evidence must originate from owner acceptance before the input EOF."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

from PIL import Image

sys.path.insert(0, str(Path(__file__).parent / '3dgs_probe'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/3dgs_probe'))
from render_fixtures import render_request_fixture, render_reply_fixture
from tools.joint_render_report import inspect_joint_renders


class JointRenderReportTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.previews = self.root / 'previews'
        self.previews.mkdir()
        request, _ = render_request_fixture()
        reply, _ = render_reply_fixture()
        self.data = {key: copy.deepcopy(reply[key]) for key in (
            'width', 'height', 'calibration_id', 'camera_from_body', 'camera_from_parent',
            'parent_from_body', 'state_timestamp', 'cloud_timestamp', 'inverse_exposure_time',
            'changed_pixels', 'worker_seconds', 'gpu_allocated_bytes', 'gpu_reserved_bytes',
            'gpu_peak_allocated_bytes', 'gpu_peak_reserved_bytes', 'worker_peak_rss_bytes')}
        self.data.update(schema=1, kind='livo_atlas.joint_render', training_view=True,
            as_of=request, K=[4., 0., 3.5, 0., 4., 2.5, 0., 0., 1.], capture_time=100.01,
            accepted_at_unix_seconds=1000., accepted_at_steady_seconds=900., latency_seconds=.1)
        for name in ('single', 'joint'):
            self.data[name] = {key: value for key, value in reply[name].items()
                               if key not in ('image_valid', 'depth_valid', 'psnr_valid')}
            self.data[name]['rgb_perfect'] = False
        self.json = self.previews / ('joint-render-' + request['job_id'] + '.json')
        self.json.with_suffix('.png').write_bytes(b'')
        Image.new('RGB', (8, 6), (12, 34, 56)).save(self.json.with_suffix('.png'))
        event = ('livo_atlas gaussian joint accepted: session=' + request['session_id'] +
            ' job=9007199254740993 map_sequence=9 pose_version=0 frame_count=6'
            ' optimized_frame_count=0 generation=4 query=5 owner=2 blocks=2 changed_pixels=2'
            ' accepted_at_steady_seconds=900.000000000 latency_seconds=0.1 worker_seconds=0.1\n')
        stats = ('livo_atlas gaussian joint: submitted=1 accepted=1 dual_accepted=1 stale=0'
                 ' coalesced=0 skipped=0 failed=0 saved=1 peak_pending=1 peak_in_flight=1 disabled=0\n')
        (self.root / 'mapping.log').write_text(event + stats)
        self.summary = dict(playback_started_steady_seconds=800., playback_eof_steady_seconds=1000.,
                            playback_confirmed_running_until_steady_seconds=999.,
                            joint_render_enabled=True)
        (self.root / 'summary.json').write_text(json.dumps(self.summary))
        self.write()

    def write(self):
        self.json.write_text(json.dumps(self.data))

    def test_valid_historical_as_of_and_big_ids_are_verified(self):
        report = inspect_joint_renders(self.root, self.previews, min_dual=1, min_groups=1)
        self.assertEqual(report['accepted_before_eof'], 1)
        self.assertEqual(report['dual_contributions_before_eof'], 1)
        self.assertEqual(report['pairs'], 1)
        self.assertEqual(report['events'][0]['job'], 9007199254740993)

    def test_eof_after_acceptance_is_required_not_just_a_saved_file(self):
        self.summary['playback_eof_steady_seconds'] = 899.
        (self.root / 'summary.json').write_text(json.dumps(self.summary))
        with self.assertRaisesRegex(ValueError, 'EOF'):
            inspect_joint_renders(self.root, self.previews, min_dual=1)
        with self.assertRaisesRegex(ValueError, 'EOF'):
            inspect_joint_renders(self.root, self.previews)
        self.summary.pop('playback_eof_steady_seconds')
        (self.root / 'summary.json').write_text(json.dumps(self.summary))
        with self.assertRaises(ValueError):
            inspect_joint_renders(self.root, self.previews)

    def test_observer_poll_delay_cannot_count_acceptance_after_last_running_proof(self):
        self.summary['playback_confirmed_running_until_steady_seconds'] = 899.
        (self.root / 'summary.json').write_text(json.dumps(self.summary))
        with self.assertRaisesRegex(ValueError, 'EOF'):
            inspect_joint_renders(self.root, self.previews)

    def test_changed_identity_pixels_or_optional_metrics_are_rejected(self):
        original = copy.deepcopy(self.data)
        for edit in (lambda d: d['as_of'].update(map_sequence='10'),
                     lambda d: d.update(accepted_at_steady_seconds=901.),
                     lambda d: d['joint'].update(rgb_mse=None),
                     lambda d: d['as_of']['blocks'][0]['asset'].update(manifest_sha256='bad'),
                     lambda d: d.update(width=1000001),
                     lambda d: d['camera_from_parent'].__setitem__(3, 10)):
            self.data = copy.deepcopy(original)
            edit(self.data)
            self.write()
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                inspect_joint_renders(self.root, self.previews)

    def test_retained_pair_limit_and_nonreference_contribution_are_enforced(self):
        for job in range(4):
            (self.previews / ('joint-render-' + str(job) + '.json')).write_text('{}')
        with self.assertRaises(ValueError):
            inspect_joint_renders(self.root, self.previews)
        for job in range(4):
            (self.previews / ('joint-render-' + str(job) + '.json')).unlink()
        log = self.root / 'mapping.log'
        log.write_text(log.read_text().replace('changed_pixels=2', 'changed_pixels=0'))
        with self.assertRaises(ValueError):
            inspect_joint_renders(self.root, self.previews, min_dual=1)


if __name__ == '__main__':
    unittest.main()
