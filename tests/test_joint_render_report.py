"""Evidence must originate from owner acceptance before the input EOF."""
import copy
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

from PIL import Image

sys.path.insert(0, str(Path(__file__).parent / '3dgs_probe'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/3dgs_probe'))
from render_fixtures import render_request_fixture, render_reply_fixture
from tools.joint_render_report import inspect_joint_renders


def source_identity(request):
    return json.dumps(request, allow_nan=False).encode('utf-8').hex()


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
            ' accepted_at_steady_seconds=900.000000000 latency_seconds=0.1 worker_seconds=0.1'
            ' source_identity=' + source_identity(request) + '\n')
        stats = ('livo_atlas gaussian joint: submitted=1 accepted=1 dual_accepted=1 stale=0'
                 ' coalesced=0 skipped=0 failed=0 saved=1 peak_pending=1 peak_in_flight=1 disabled=0'
                 ' pending=0 in_flight=0 pending_storage=0\n')
        (self.root / 'mapping.log').write_text(event + stats)
        self.summary = dict(playback_started_steady_seconds=800., playback_eof_steady_seconds=1000.,
                            playback_confirmed_running_until_steady_seconds=999.,
                            playback_input_cutoff_steady_seconds=999.,
                            playback_input_cutoff_source='source_publish_begin',
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

    def test_player_lifetime_alone_is_not_input_publication_evidence(self):
        self.summary.pop('playback_input_cutoff_steady_seconds', None)
        self.summary.pop('playback_input_cutoff_source', None)
        (self.root / 'summary.json').write_text(json.dumps(self.summary))
        with self.assertRaises(ValueError):
            inspect_joint_renders(self.root, self.previews)

    def test_fixed_rate_proof_is_recomputed_and_clock_changes_are_rejected(self):
        self.summary.update(playback_input_cutoff_source='fixed_rate_lower_bound',
            playback_wall_clock_stable=True, playback_rate=1.,
            playback_first_input_timestamp_ns='100000000000',
            playback_last_input_timestamp_ns='299010000000')
        path = self.root / 'summary.json'
        path.write_text(json.dumps(self.summary))
        self.assertEqual(inspect_joint_renders(self.root, self.previews)['accepted_before_eof'], 1)
        self.summary['playback_input_cutoff_steady_seconds'] += .01
        path.write_text(json.dumps(self.summary))
        with self.assertRaisesRegex(ValueError, 'lower bound'):
            inspect_joint_renders(self.root, self.previews)
        self.summary['playback_input_cutoff_steady_seconds'] = 999.
        self.summary['playback_wall_clock_stable'] = False
        path.write_text(json.dumps(self.summary))
        with self.assertRaisesRegex(ValueError, 'clock'):
            inspect_joint_renders(self.root, self.previews)

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
        self.summary['playback_input_cutoff_steady_seconds'] = 899.
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

    def test_valid_looking_source_hash_cannot_replace_accepted_identity(self):
        self.data['as_of']['blocks'][0]['asset']['manifest_sha256'] = 'c' * 64
        self.write()
        with self.assertRaises(ValueError):
            inspect_joint_renders(self.root, self.previews)

    def test_original_request_type_cannot_be_overwritten_by_the_checker(self):
        self.data['as_of']['type'] = 'joint_render_ok'
        self.write()
        with self.assertRaises(ValueError):
            inspect_joint_renders(self.root, self.previews)

    def test_jpeg_disguised_as_png_is_rejected(self):
        Image.new('RGB', (8, 6)).save(self.json.with_suffix('.png'), format='JPEG')
        with self.assertRaises(ValueError):
            inspect_joint_renders(self.root, self.previews)

    def test_terminal_owner_counters_cannot_claim_live_render_storage(self):
        log = self.root / 'mapping.log'
        original = log.read_text()
        for field in ('pending', 'in_flight', 'pending_storage'):
            log.write_text(original.replace(' ' + field + '=0', ' ' + field + '=1'))
            with self.subTest(field=field), self.assertRaises(ValueError):
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

    def test_removing_complete_history_pairs_is_not_a_successful_session(self):
        event, stats = (self.root / 'mapping.log').read_text().splitlines()
        base = int(self.data['as_of']['job_id'])
        events = []
        paths = []
        for offset in range(4):
            job = str(base + offset)
            data = copy.deepcopy(self.data)
            data['as_of']['job_id'] = job
            path = self.previews / ('joint-render-' + job + '.json')
            path.write_text(json.dumps(data))
            if offset:
                shutil.copyfile(self.json.with_suffix('.png'), path.with_suffix('.png'))
            paths.append(path)
            events.append(event.replace('job=' + str(base), 'job=' + job).replace(
                source_identity(self.data['as_of']), source_identity(data['as_of'])))
        for field in ('submitted', 'accepted', 'dual_accepted', 'saved'):
            stats = stats.replace(field + '=1', field + '=4')
        (self.root / 'mapping.log').write_text('\n'.join(events + [stats]) + '\n')
        self.assertEqual(inspect_joint_renders(self.root, self.previews)['pairs'], 4)
        for path in paths[1:]:
            path.unlink()
            path.with_suffix('.png').unlink()
        with self.assertRaisesRegex(ValueError, 'retained'):
            inspect_joint_renders(self.root, self.previews)


if __name__ == '__main__':
    unittest.main()
