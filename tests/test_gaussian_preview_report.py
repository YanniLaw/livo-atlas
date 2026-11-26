"""Receiver evidence must agree with actual owner acceptance and retained pixels."""
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

from tools.gaussian_preview_report import (gaussian_render_record,
    gaussian_image_record, inspect_gaussian_publication)


def message():
    header = NS(frame_id='atlas/gaussian_optical', stamp=100010000000)
    pose = NS(position=NS(x=3., y=1., z=-3.),
              orientation=NS(x=0., y=0., z=-0.7071067811865476, w=0.7071067811865476))
    image = NS(header=header, width=8, height=6, encoding='rgb8', step=24,
               is_bigendian=False, data=bytes([12, 34, 56]) * 48)
    sources = [NS(submap_id=i, binding_generation=i + 1, source_generation=i + 1,
        content_job_id=i + 10, membership_revision=3, resident=False, asset_id=i + 1,
        manifest_sha256='b' * 64, frame_ids=[2 * i, 2 * i + 1], gaussian_count=2)
        for i in (1, 2)]
    return NS(header=header, producer_id='b' * 32, revision=1, session_id='a' * 32,
        status=0, reason='', generation=4, map_sequence=9, pose_version=0, frame_count=6,
        optimized_frame_count=0, parent_frame='camera_init', policy='temporal',
        job_id=9007199254740993, query_keyframe_id=5, query_submap_id=2, neighbor_reason='',
        sources=sources, calibration_id=0, capture_time=100.01,
        k=[4., 0., 3.5, 0., 4., 2.5, 0., 0., 1.], parent_from_camera=pose,
        changed_pixels=2, candidate_count=0, candidate_pixels=0,
        projection_uncertain=False, image=image)


def invalidated():
    value = message()
    value.status, value.reason, value.revision = 2, 'source_changed', 2
    value.header.stamp = 0
    value.job_id = value.query_keyframe_id = value.query_submap_id = 0
    value.capture_time = value.changed_pixels = 0
    value.sources, value.k = [], [0.] * 9
    value.parent_from_camera.position.x = 0
    value.parent_from_camera.position.y = 0
    value.parent_from_camera.position.z = 0
    value.parent_from_camera.orientation.z = 0
    value.parent_from_camera.orientation.w = 1
    value.image.width = value.image.height = value.image.step = 0
    value.image.encoding, value.image.data = '', b''
    return value


class GaussianPublicationRecordTest(unittest.TestCase):
    def test_normalizes_middleware_camera_scalars_to_owned_json_numbers(self):
        # ROS2 fixed arrays contain NumPy scalars. A float subclass exercises
        # the same non-builtin scalar boundary without a NumPy test dependency.
        class MiddlewareFloat(float):
            pass

        source = message()
        source.k = [MiddlewareFloat(value) for value in source.k]
        record = gaussian_render_record(source, lambda stamp: stamp)
        self.assertTrue(all(type(value) is float for value in record['k']))
        self.assertEqual(record['k'], [4., 0., 3.5, 0., 4., 2.5, 0., 0., 1.])
        json.dumps(record, allow_nan=False)
        source.k[2] = 99.
        self.assertEqual(record['k'][2], 3.5)

    def test_records_own_pixels_metadata_and_full_uint64_values(self):
        source = message()
        value = gaussian_render_record(source, lambda stamp: stamp)
        self.assertEqual(value['job_id'], 9007199254740993)
        self.assertEqual(value['image']['stamp_ns'], 100010000000)
        self.assertEqual(value['k'][2], 3.5)
        source.sources[0].frame_ids.clear()
        source.k[2] = 99
        self.assertEqual(value['sources'][0]['frame_ids'], [2, 3])
        self.assertEqual(value['k'][2], 3.5)

    def test_pixel_and_nonready_payload_corruption_is_rejected(self):
        for change in (lambda m: setattr(m.image, 'data', b'x'),
                       lambda m: setattr(m.image, 'width', 1000001),
                       lambda m: setattr(m.image, 'encoding', 'bgr8'),
                       lambda m: setattr(m.image, 'step', 25),
                       lambda m: setattr(m, 'status', 2)):
            source = message()
            change(source)
            with self.subTest(change=change), self.assertRaises(ValueError):
                gaussian_render_record(source, lambda stamp: stamp)
        cleared = gaussian_render_record(invalidated(), lambda stamp: stamp)
        self.assertEqual(cleared['image']['width'], 0)
        self.assertEqual(cleared['sources'], [])
        mirror = gaussian_image_record(invalidated().image, lambda stamp: stamp)
        self.assertEqual(mirror['bytes'], 0)

    def test_sensor_time_conversion_preserves_split_second_rounding(self):
        source = message()
        source.capture_time = 946685572.3998371
        source.header.stamp = 946685572399837136
        self.assertEqual(gaussian_render_record(source, lambda stamp: stamp)['stamp_ns'],
                         946685572399837136)
        source.header.stamp += 1
        with self.assertRaises(ValueError):
            gaussian_render_record(source, lambda stamp: stamp)


class GaussianPublicationEvidenceTest(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(Path(__file__).parent))
        import test_joint_render_report
        fixture = test_joint_render_report.JointRenderReportTest()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.fixture = fixture
        self.root, self.previews = fixture.root, fixture.previews
        records = [gaussian_render_record(m, lambda stamp: stamp)
                   for m in (message(), invalidated())]
        for i, row in enumerate(records):
            row['received_at_steady_seconds'] = 950. + i
        self.observed = dict(enabled=True, optical_frame='atlas/gaussian_optical',
            map_frame='map', samples=records, image_samples=[copy.deepcopy(records[0]['image'])],
            errors=[])
        path = self.root / 'mapping.log'
        path.write_text(path.read_text() +
            'livo_atlas gaussian publication: observed=2 submitted=3 coalesced=0'
            ' published=2 unavailable=1 failed=0 pending=0 in_flight=0 peak_pending=1'
            ' peak_in_flight=1 last_revision=3 max_worker_seconds=0.01 finished=1\n')
        self.write()

    def write(self):
        (self.root / 'gaussian-publication.json').write_text(json.dumps(self.observed))

    def test_matches_received_source_camera_pixels_and_live_cutoff(self):
        report = inspect_gaussian_publication(self.root, self.previews)
        self.assertEqual(report['ready_before_input_cutoff'], 1)
        self.assertEqual(report['png_matches_before_input_cutoff'], 1)
        self.assertFalse(report['terminal_state_received'])
        self.assertEqual(report['received'], 2)

    def test_source_camera_pixels_revision_and_foreign_identity_tamper_fail(self):
        original = copy.deepcopy(self.observed)
        for change in (lambda r: r['samples'][0]['sources'][0].update(content_job_id=999),
                       lambda r: r['samples'][0]['k'].__setitem__(2, 4.),
                       lambda r: r['samples'][0]['image'].update(sha256='0' * 64),
                       lambda r: r['samples'][0].update(revision=3),
                       lambda r: r['samples'][0].update(session_id='c' * 32),
                       lambda r: r['samples'][1].update(producer_id='c' * 32),
                       lambda r: r['samples'][0].update(status=6, reason='stopped')):
            self.observed = copy.deepcopy(original)
            change(self.observed)
            self.write()
            with self.subTest(change=change), self.assertRaises(ValueError):
                inspect_gaussian_publication(self.root, self.previews)

    def test_new_revision_cannot_restore_an_older_known_map_context(self):
        self.observed['samples'][1]['map_sequence'] = 10
        stale = copy.deepcopy(self.observed['samples'][0])
        stale.update(revision=3, received_at_steady_seconds=952.)
        self.observed['samples'].append(stale)
        path = self.root / 'mapping.log'
        path.write_text(path.read_text().replace('published=2', 'published=3')
                        .replace('last_revision=3', 'last_revision=4'))
        self.write()
        with self.assertRaises(ValueError):
            inspect_gaussian_publication(self.root, self.previews)

    def test_context_closing_after_partial_native_calls_does_not_erase_reception(self):
        path = self.root / 'mapping.log'
        path.write_text(path.read_text().replace('published=2 unavailable=1',
                                                 'published=1 unavailable=2'))
        report = inspect_gaussian_publication(self.root, self.previews)
        self.assertEqual(report['received'], 2)
        self.assertEqual(report['native_calls_completed'], 1)
        self.assertEqual(report['unavailable_attempts'], 2)

    def test_late_receipt_and_published_counter_do_not_prove_live_reception(self):
        self.observed['samples'][0]['received_at_steady_seconds'] = 1001.
        self.write()
        with self.assertRaises(ValueError):
            inspect_gaussian_publication(self.root, self.previews)

    def test_missing_receiver_data_and_false_terminal_flag_cannot_prove_delivery(self):
        self.observed['terminal_state_received'] = True
        self.write()
        self.assertFalse(inspect_gaussian_publication(self.root, self.previews)['terminal_state_received'])
        self.observed['samples'] = []
        self.write()
        with self.assertRaises(ValueError):
            inspect_gaussian_publication(self.root, self.previews)

    def test_humble_node_log_is_supported(self):
        (self.root / 'mapping.log').rename(self.root / 'node.log')
        self.assertEqual(inspect_gaussian_publication(self.root, self.previews)['received'], 2)

    def test_shape_is_part_of_the_mirror_content_match(self):
        self.observed['image_samples'][0].update(width=6, height=8, step=18)
        self.write()
        with self.assertRaisesRegex(ValueError, 'No received image mirror matches a READY image'):
            inspect_gaussian_publication(self.root, self.previews)

    def test_enabled_reception_requires_both_topics_but_not_ready(self):
        from tools.gaussian_preview_report import gaussian_reception_errors
        observation = dict(enabled=True, samples=[], image_samples=[], errors=[])
        self.assertEqual(len(gaussian_reception_errors(observation)), 2)
        observation['samples'] = [gaussian_render_record(invalidated(), lambda t: t)]
        self.assertEqual(len(gaussian_reception_errors(observation)), 1)
        observation['image_samples'] = [gaussian_image_record(invalidated().image, lambda t: t)]
        self.assertEqual(gaussian_reception_errors(observation), [])
        observation.update(enabled=False, samples=[], image_samples=[])
        self.assertEqual(gaussian_reception_errors(observation), [])

    def test_withdrawn_job_cannot_become_ready_again(self):
        old = copy.deepcopy(self.observed['samples'][0])
        old.update(revision=3, received_at_steady_seconds=952.)
        self.observed['samples'].append(old)
        path = self.root / 'mapping.log'
        path.write_text(path.read_text().replace('published=2', 'published=3')
                        .replace('last_revision=3', 'last_revision=4'))
        self.write()
        with self.assertRaises(ValueError):
            inspect_gaussian_publication(self.root, self.previews)

    def test_new_revision_cannot_restore_a_job_older_than_the_displayed_job(self):
        self.AddSecondOwnerJob()
        old = copy.deepcopy(self.observed['samples'][0])
        old.update(revision=4, received_at_steady_seconds=953.)
        self.observed['samples'].append(old)
        path = self.root / 'mapping.log'
        path.write_text(path.read_text().replace('published=3', 'published=4')
                        .replace('last_revision=4', 'last_revision=5'))
        self.write()
        with self.assertRaises(ValueError):
            inspect_gaussian_publication(self.root, self.previews)

    def AddSecondOwnerJob(self):
        import test_joint_render_report
        request = copy.deepcopy(self.fixture.data['as_of'])
        original = copy.deepcopy(request)
        request['job_id'] = '9007199254740994'
        path = self.root / 'mapping.log'
        lines = path.read_text().splitlines()
        second = lines[0].replace('job=9007199254740993', 'job=9007199254740994').replace(
            test_joint_render_report.source_identity(original),
            test_joint_render_report.source_identity(request))
        text = '\n'.join((lines[0], second, *lines[1:])) + '\n'
        for key in ('submitted', 'accepted', 'dual_accepted'):
            text = text.replace(key + '=1', key + '=2')
        text = text.replace('published=2', 'published=3').replace('last_revision=3', 'last_revision=4')
        path.write_text(text)
        fresh = copy.deepcopy(self.observed['samples'][0])
        fresh.update(revision=3, job_id=9007199254740994, received_at_steady_seconds=952.)
        self.observed['samples'].append(fresh)
        self.write()

    def test_same_capture_time_with_a_different_job_remains_a_new_version(self):
        self.AddSecondOwnerJob()
        self.assertEqual(inspect_gaussian_publication(self.root, self.previews)['ready_before_input_cutoff'], 2)

    def test_unretained_spatial_camera_must_match_accepted_request(self):
        self.fixture.spatial()
        ready, cleared = self.observed['samples']
        ready.update(policy='spatial', candidate_count=1, candidate_pixels=48)
        ready['sources'][0].update(submap_id=0, binding_generation=1,
                                  source_generation=1, content_job_id=10,
                                  asset_id=1, frame_ids=[0, 1])
        cleared['policy'] = 'spatial'
        self.AddSecondOwnerJob()
        valid = self.observed['samples'][-1]
        report = inspect_gaussian_publication(self.root, self.previews)
        self.assertEqual(report['spatial_camera_matches'], 2)
        self.assertEqual(report['png_matches_before_input_cutoff'], 1)

        for field in ('K', 'capture time', 'calibration', 'dimensions'):
            forged = copy.deepcopy(valid)
            if field == 'K':
                forged['k'][2] = 4.
            elif field == 'capture time':
                forged.update(capture_time=100.02, stamp_ns=100020000000)
                forged['image']['stamp_ns'] = forged['stamp_ns']
            elif field == 'calibration':
                forged['calibration_id'] = 1
            else:
                forged['image'].update(width=6, height=8, step=18)
            self.observed['samples'][-1] = forged
            self.write()
            with self.subTest(field=field):
                with self.assertRaisesRegex(ValueError, 'accepted spatial request'):
                    inspect_gaussian_publication(self.root, self.previews)


if __name__ == '__main__':
    unittest.main()
