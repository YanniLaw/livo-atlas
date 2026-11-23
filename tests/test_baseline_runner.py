from pathlib import Path
import tempfile
import subprocess
import sys
import time
import unittest

import numpy as np

from tools.run_baseline import exited_cleanly, make_trajectory_path
from tools import mapping_log, run_baseline, run_ros2_baseline


class BaselineRunnerTest(unittest.TestCase):
    def test_fixed_rate_boundary_excludes_a_live_publishers_teardown(self):
        launch_before = time.monotonic()
        publisher = subprocess.Popen([sys.executable, '-c',
            'import time; time.sleep(.03); print(time.monotonic(), flush=True); time.sleep(.2)'],
            stdout=subprocess.PIPE, text=True)
        try:
            last_publish_begin = float(publisher.stdout.readline())
            cutoff = run_baseline.fixed_rate_input_cutoff(launch_before, 1_000_000_000, 1_010_000_000, 1.)
            self.assertLess(cutoff, last_publish_begin)
            self.assertLess(cutoff, time.monotonic())
            self.assertIsNone(publisher.poll())
        finally:
            publisher.wait(timeout=5)
            publisher.stdout.close()

    def test_gaussian_preview_requires_a_complete_pair(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertFalse(run_baseline.gaussian_preview_available(root))
            (root / 'preview-1.png').write_bytes(b'image')
            self.assertFalse(run_baseline.gaussian_preview_available(root))
            (root / 'preview-1.json').write_text('{}')
            self.assertTrue(run_baseline.gaussian_preview_available(root))

    def test_mapping_process_requires_zero_exit_status(self):
        self.assertTrue(exited_cleanly(0, '', False))
        self.assertFalse(exited_cleanly(-11, '', False))

    def test_gdb_requires_normal_inferior_exit_evidence(self):
        self.assertFalse(exited_cleanly(0, '', True))
        self.assertTrue(exited_cleanly(0, '[Inferior 1 (process 123) exited normally]', True))

    def test_gdb_fatal_signals_do_not_pass(self):
        for name in ['SIGSEGV', 'SIGABRT', 'SIGBUS', 'SIGFPE']:
            with self.subTest(signal=name):
                self.assertFalse(exited_cleanly(0, 'Program received signal ' + name, True))

    def test_gdb_nonzero_inferior_exit_does_not_pass(self):
        self.assertFalse(exited_cleanly(0, '[Inferior 1 (process 123) exited with code 01]', True))

    def test_gdb_signal_stop_is_rejected_even_with_a_normal_exit_marker(self):
        log = 'Program received signal SIGABRT\n[Inferior 1 (process 123) exited normally]'
        self.assertFalse(exited_cleanly(0, log, True))

    def test_stale_trajectory_is_rejected_and_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory)
            stale = package / 'Log/result/test.txt'
            stale.parent.mkdir(parents=True)
            stale.write_text('old trajectory\n')
            with self.assertRaisesRegex(RuntimeError, 'exists'):
                make_trajectory_path(package, 'test')
            self.assertEqual(stale.read_text(), 'old trajectory\n')

    def test_ros2_separate_logs_preserve_stage_counts_and_fatal_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'node.log').write_text('LIO Mapping Time\nVIO Time\n')
            (root / 'node-stderr.log').write_text('[FATAL] shutdown failed\n')
            stdout, stderr = run_ros2_baseline.read_node_logs(root)
            self.assertEqual(stdout.count('LIO Mapping Time'), 1)
            self.assertEqual(stdout.count('VIO Time'), 1)
            self.assertFalse(run_ros2_baseline.exited_cleanly(0, stdout + stderr))

    def test_ros2_glog_records_are_read_from_stderr(self):
        from tools.mapping_log import read_mapping_log

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'node.log').write_text('LIO Mapping Time\n')
            (root / 'node-stderr.log').write_text(
                'I1122 12:00:00.123456  1234 livo_node.cc:110] VIO Time\n'
                'F1122 12:00:01.000000  1234 livo_main.cc:88] failed\n'
            )
            log = read_mapping_log(root)
            self.assertEqual(log.count('LIO Mapping Time'), 1)
            self.assertEqual(log.count('VIO Time'), 1)
            self.assertFalse(run_ros2_baseline.exited_cleanly(0, log))

    def test_legacy_frontend_timing_retains_updates_without_samples(self):
        log = ('LIO Mapping Time\n| Current Total Time | 0.02 |\n'
               'VIO Time\nLIO Mapping Time\n| Current Total Time | 4e-2 |\n')
        counts, timings = mapping_log.summarize_frontend_updates(log, np)
        self.assertEqual(counts, {'lio': 2, 'vio': 1})
        self.assertEqual(timings['lio']['count'], 2)
        self.assertAlmostEqual(timings['lio']['p50'], .03)
        self.assertAlmostEqual(timings['lio']['p95'], .039)
        self.assertAlmostEqual(timings['lio']['p99'], .0398)
        self.assertNotIn('vio', timings)

    def test_structured_frontend_updates_take_precedence_over_legacy(self):
        log = ('I1124 02:20:00.000000  1 input_diagnostics.h:27] '
               'livo_atlas update kind=lio frontend_process_seconds=0.02\n'
               'livo_atlas update kind=vio frontend_process_seconds=4e-3\n'
               'LIO Mapping Time\n| Current Total Time | 0.02 |\n'
               'VIO Time\n| Current Total Time | 0.004 |\n')
        counts, timings = mapping_log.summarize_frontend_updates(log, np)
        self.assertEqual(counts, {'lio': 1, 'vio': 1})
        self.assertEqual(timings['lio']['count'], 1)
        self.assertEqual(timings['vio']['max'], .004)
        self.assertEqual(timings['vio']['median'], timings['vio']['p50'])

    def test_sensor_handoff_diagnostics(self):
        log = (
            'livo_atlas inputs: lidar=5 imu=200 image=4\n'
            'livo_atlas sensor_input sensor=lidar peak=2 combined_sampled_peak=4 '
            'pending=1 system_pending=2 transport_lost=3 conversion_rejected=1 '
            'system_rejected=0 overflow_rejected=1 closed_rejected=2\n'
        )
        topics = {'lidar': '/lidar', 'imu': '/imu', 'image': '/camera'}
        self.assertEqual(mapping_log.read_input_counts(log, topics),
                         {'/lidar': 5, '/imu': 200, '/camera': 4})
        self.assertEqual(mapping_log.read_sensor_input_stats(log), {
            'lidar': dict(peak=2, combined_sampled_peak=4, pending=1,
                          system_pending=2, transport_lost=3,
                          conversion_rejected=1, system_rejected=0,
                          overflow_rejected=1, closed_rejected=2),
        })
        self.assertEqual(mapping_log.read_input_counts('', topics), {})
        self.assertEqual(mapping_log.read_sensor_input_stats(''), {})

    def test_shutdown_updates_count_toward_estimation_and_skip_ros(self):
        log = ('livo_atlas update kind=lio frontend_process_seconds=0.05 ros_published=1\n'
               'livo_atlas update kind=lio frontend_process_seconds=0.05 ros_published=0\n'
               'livo_atlas update kind=vio frontend_process_seconds=0.01\n')
        counts, timings = mapping_log.summarize_frontend_updates(log, np)
        self.assertEqual(counts, {'lio': 2, 'vio': 1})
        self.assertEqual(timings['lio']['count'], 2)
        self.assertEqual(mapping_log.count_published_updates(log, counts),
                         {'lio': 1, 'vio': 1})

    def test_stage_timings_and_queue_peaks_share_both_replay_summaries(self):
        log = ('I1124 02:20:00.000000  1 livo_system.cc:331] '
               'livo_atlas timing stage=synchronization seconds=0.001\n'
               'livo_atlas timing stage=imu_lock_wait seconds=0.08\n'
               'livo_atlas timing stage=imu_lock_wait seconds=0.02\n'
               'livo_atlas timing stage=imu_lock_wait seconds=nan\n'
               'livo_atlas sensor_buffer_peak queue=imu count=20 span_s=0.12\n')
        timings, peaks = mapping_log.summarize_stage_timings(log, np)
        self.assertEqual(timings['imu_lock_wait']['count'], 2)
        self.assertAlmostEqual(timings['imu_lock_wait']['p95'], .077)
        self.assertEqual(timings['synchronization']['p99'], .001)
        self.assertEqual(peaks, {'imu': {'count': 20, 'span_s': .12}})
        self.assertEqual(mapping_log.summarize_stage_timings('', np), ({}, {}))

    def test_ros2_path_size_uses_configured_capacity(self):
        counts = {'/aft_mapped_to_init': 5, '/rgb_img': 1, '/cloud_registered': 6}
        poses = [[0., 0., 0., 0., 0., 0., 1.]] * 5
        self.assertFalse(run_ros2_baseline.output_issues(
            5, 1, counts, 3, poses, path_max_poses=3))
        self.assertEqual(run_ros2_baseline.output_issues(
            5, 1, counts, 4, poses, path_max_poses=3),
            {'path_size': {'expected': 3, 'observed': 4}})
        self.assertFalse(run_ros2_baseline.output_issues(5, 1, counts, 5, poses))

    def test_path_capacity_comes_from_the_node_across_binary_versions(self):
        logging_prefix = 'I1124 03:45:00.000000 1 livo_node.cc:163] '
        legacy_log = logging_prefix + 'livo_atlas frontend ready\n'
        current_log = logging_prefix + 'livo_atlas frontend ready path_max_poses=3\n'
        self.assertIsNone(mapping_log.read_path_max_poses(legacy_log))
        self.assertEqual(mapping_log.read_path_max_poses(current_log), 3)
        counts = {'/aft_mapped_to_init': 2001, '/rgb_img': 0, '/cloud_registered': 2001}
        poses = [[0., 0., 0., 0., 0., 0., 1.]] * 2001
        self.assertFalse(run_ros2_baseline.output_issues(
            2001, 0, counts, 2001, poses, mapping_log.read_path_max_poses(legacy_log)))


class GlobalBaselineParserTest(unittest.TestCase):
    def test_loop_replays_select_detection_or_pose_graph(self):
        for module, entry in ((run_baseline, ['--workspace', '/ws']),
                              (run_ros2_baseline, ['--executable', '/node'])):
            common = entry + ['--bag', '/bag', '--output', '/output']
            for flag, optimize in (('--loop-closure', True),
                                   ('--loop-detection-only', False)):
                with self.subTest(module=module.__name__, flag=flag):
                    args = module.parse_arguments(common + [flag])
                    self.assertTrue(args.loop_closure)
                    self.assertEqual(args.pose_graph_enabled, optimize)
                    self.assertEqual(args.submap_mode, 'online')

    def test_gaussian_paths_do_not_overlap_working_or_exported_dataset(self):
        for module, node_output in ((run_baseline, 'node_output'),
                                    (run_ros2_baseline, 'node-output')):
            with tempfile.TemporaryDirectory() as name:
                root = Path(name)
                entry = (['--workspace', str(root / 'ws')] if module is run_baseline
                         else ['--executable', str(root / 'node')])
                flags = entry + ['--bag', str(root / 'bag'), '--output', str(root / 'output'),
                                 '--gaussian-socket', str(root / 'worker.sock')]
                for relative in ('keyframes', 'keyframes/previews', node_output,
                                 node_output + '/keyframes', node_output + '/keyframes/previews'):
                    with self.subTest(module=module.__name__, relative=relative):
                        with self.assertRaises(SystemExit) as error:
                            module.parse_arguments(flags + ['--gaussian-output', str(root / 'output' / relative)])
                        self.assertEqual(error.exception.code, 2)
                        self.assertFalse((root / 'output').exists())
                sibling = root / 'output/previews'
                self.assertEqual(module.parse_arguments(flags + ['--gaussian-output', str(sibling)]).gaussian_output,
                                 sibling)

    def invoke(self, script, directory, *flags):
        package = Path(__file__).resolve().parents[1]
        command = [sys.executable, str(package / 'tools' / script),
                   '--bag', str(directory / 'missing-bag'),
                   '--output', str(directory / 'output')]
        if script == 'run_baseline.py':
            command += ['--workspace', str(directory / 'missing-workspace')]
        else:
            command += ['--executable', str(directory / 'missing-node')]
        return subprocess.run(command + list(flags), capture_output=True, text=True)

    def test_global_output_rejects_groups_before_io(self):
        for script in ('run_baseline.py', 'run_ros2_baseline.py'):
            with self.subTest(script=script), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                result = self.invoke(script, root, '--global-output', '--submap-mode', 'keyframe_groups')
                self.assertEqual(result.returncode, 2)
                self.assertIn('--global-output requires online submaps', result.stderr)
                self.assertFalse((root / 'output').exists())

    def test_global_output_preserves_queue_validation(self):
        for script in ('run_baseline.py', 'run_ros2_baseline.py'):
            with self.subTest(script=script), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                result = self.invoke(script, root, '--global-output', '--image-queue-size', '0')
                self.assertEqual(result.returncode, 2)
                self.assertIn('--image-queue-size must be positive', result.stderr)
                self.assertFalse((root / 'output').exists())

    def test_geometry_map_rejects_groups_before_io(self):
        for script in ('run_baseline.py', 'run_ros2_baseline.py'):
            with self.subTest(script=script), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                result = self.invoke(script, root, '--geometry-map', '--submap-mode', 'keyframe_groups')
                self.assertEqual(result.returncode, 2)
                self.assertIn('--geometry-map requires online submaps', result.stderr)
                self.assertFalse((root / 'output').exists())

    def test_publication_requires_joint_render_before_output(self):
        for script in ('run_baseline.py', 'run_ros2_baseline.py'):
            with self.subTest(script=script), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                result = self.invoke(script, root, '--gaussian-publish-preview')
                self.assertEqual(result.returncode, 2)
                self.assertIn('requires --gaussian-joint-render', result.stderr)
                self.assertFalse((root / 'output').exists())

    def test_publication_defaults_forwarding_and_frame_validation_are_shared(self):
        for module in (run_baseline, run_ros2_baseline):
            with tempfile.TemporaryDirectory() as name:
                root = Path(name)
                entry = (['--workspace', str(root / 'ws')] if module is run_baseline
                         else ['--executable', str(root / 'node')])
                flags = entry + ['--bag', str(root / 'bag'), '--output', str(root / 'output'),
                    '--gaussian-socket', str(root / 's'), '--gaussian-output', str(root / 'g'),
                    '--gaussian-persist-submaps', '--gaussian-joint-render']
                default = module.parse_arguments(flags)
                self.assertFalse(default.gaussian_publish_preview)
                self.assertNotIn('gaussian.publish_preview', module.gaussian_parameters(default))
                args = module.parse_arguments(flags + ['--gaussian-publish-preview',
                    '--gaussian-preview-frame-id', 'atlas/camera_optical'])
                actual = module.gaussian_parameters(args)
                self.assertIs(actual['gaussian.publish_preview'], True)
                self.assertEqual(actual['gaussian.preview_frame_id'], 'atlas/camera_optical')
                for frame in ('', '/cam', 'cam/', 'cam//sub', 'bad frame', 'world', 'a' * 129):
                    with self.subTest(frame=frame), self.assertRaises(SystemExit):
                        module.parse_arguments(flags + ['--gaussian-publish-preview',
                            '--gaussian-preview-frame-id', frame])
                self.assertFalse((root / 'output').exists())

    def test_joint_render_requires_persistence_before_output(self):
        for script in ('run_baseline.py', 'run_ros2_baseline.py'):
            with self.subTest(script=script), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                result = self.invoke(script, root, '--gaussian-joint-render')
                self.assertEqual(result.returncode, 2)
                self.assertIn('requires --gaussian-persist-submaps', result.stderr)
                self.assertFalse((root / 'output').exists())

    def test_joint_render_ranges_and_defaults_are_shared_by_both_runners(self):
        for module in (run_baseline, run_ros2_baseline):
            with tempfile.TemporaryDirectory() as name:
                root = Path(name)
                entry = ['--workspace', str(root / 'ws')] if module is run_baseline else ['--executable', str(root / 'node')]
                flags = entry + ['--bag', str(root / 'bag'), '--output', str(root / 'output'),
                    '--gaussian-socket', str(root / 'worker.sock'), '--gaussian-output', str(root / 'g'),
                    '--gaussian-persist-submaps', '--gaussian-joint-render']
                options = module.parse_arguments(flags)
                self.assertTrue(options.gaussian_joint_render)
                self.assertEqual(options.gaussian_joint_render_interval_ms, 1000)
                self.assertEqual(options.gaussian_joint_render_timeout_ms, 5000)
                for argument, values in (('--gaussian-joint-render-interval-ms', ['99', '60001', '1.5']),
                                         ('--gaussian-joint-render-timeout-ms', ['0', '60001', '1.5'])):
                    for value in values:
                        with self.subTest(argument=argument, value=value), self.assertRaises(SystemExit):
                            module.parse_arguments(flags + [argument, value])
                self.assertFalse((root / 'output').exists())

    def test_render_policy_defaults_and_forwarding_match_both_runners(self):
        for module in (run_baseline, run_ros2_baseline):
            with self.subTest(module=module.__name__), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                entry = (['--workspace', str(root / 'ws')] if module is run_baseline
                         else ['--executable', str(root / 'node')])
                flags = entry + ['--bag', str(root / 'bag'), '--output', str(root / 'output'),
                    '--gaussian-socket', str(root / 's'), '--gaussian-output', str(root / 'g'),
                    '--gaussian-persist-submaps', '--gaussian-joint-render']
                default = module.parse_arguments(flags)
                self.assertEqual(default.joint_render_policy, 'temporal')
                for policy in ('temporal', 'spatial'):
                    args = module.parse_arguments(flags + ['--joint-render-policy', policy])
                    self.assertEqual(module.gaussian_parameters(args)['gaussian.joint_render_policy'], policy)
                with self.assertRaises(SystemExit):
                    module.parse_arguments(flags + ['--joint-render-policy', 'unknown'])
                self.assertFalse((root / 'output').exists())

    def test_spatial_policy_requires_joint_render_before_output_creation(self):
        for script in ('run_baseline.py', 'run_ros2_baseline.py'):
            with self.subTest(script=script), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                result = self.invoke(script, root, '--joint-render-policy', 'spatial')
                self.assertEqual(result.returncode, 2)
                self.assertIn('requires --gaussian-joint-render', result.stderr)
                self.assertFalse((root / 'output').exists())

    def test_gaussian_persistence_requires_worker_paths(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            result = self.invoke('run_baseline.py', root, '--gaussian-persist-submaps')
            self.assertEqual(result.returncode, 2)
            self.assertIn('requires --gaussian-socket', result.stderr)
            self.assertFalse((root / 'output').exists())
            args = run_baseline.parse_arguments([
                '--workspace', str(root / 'ws'), '--bag', str(root / 'bag'),
                '--output', str(root / 'output'), '--gaussian-persist-submaps',
                '--gaussian-socket', str(root / 'worker.sock'),
                '--gaussian-output', str(root / 'previews')])
            self.assertTrue(args.gaussian_persist_submaps)

    def test_gaussian_arguments_are_paired_and_require_online_submaps(self):
        for flags in (['--gaussian-socket', '/tmp/worker.sock'],
                      ['--gaussian-output', '/tmp/preview'],
                      ['--gaussian-socket', '', '--gaussian-output', '/tmp/preview'],
                      ['--gaussian-socket', '/tmp/worker.sock', '--gaussian-output', '',],
                      ['--gaussian-socket', '/tmp/worker.sock', '--gaussian-output', '/tmp/preview',
                       '--submap-mode', 'keyframe_groups']):
            with self.subTest(flags=flags), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                result = self.invoke('run_baseline.py', root, *flags)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertFalse((root / 'output').exists())

    def test_gaussian_rejects_existing_output_without_modifying_it(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            previews = root / 'previews'
            previews.mkdir()
            sentinel = previews / 'keep.txt'
            sentinel.write_text('keep')
            result = self.invoke('run_baseline.py', root,
                                 '--gaussian-socket', str(root / 'worker.sock'),
                                 '--gaussian-output', str(previews))
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertEqual(sentinel.read_text(), 'keep')
            self.assertFalse((root / 'output').exists())


class GlobalBaselineOptionsTest(unittest.TestCase):
    def test_omitted_legacy_flags_preserve_loaded_persistence_configuration(self):
        for module in (run_baseline, run_ros2_baseline):
            with self.subTest(module=module.__name__), tempfile.TemporaryDirectory() as name:
                root = Path(name)
                entry = (['--workspace', str(root / 'ws')] if module is run_baseline
                         else ['--executable', str(root / 'node')])
                flags = entry + ['--bag', str(root / 'bag'), '--output', str(root / 'output'),
                    '--gaussian-socket', str(root / 's'), '--gaussian-output', str(root / 'g')]
                configured = {'gaussian.persist_submaps': True,
                              'gaussian.finalize_on_shutdown': True,
                              'gaussian.finalize_timeout_ms': 120000}
                parameters = dict(configured)
                parameters.update(module.gaussian_parameters(module.parse_arguments(flags)))
                self.assertEqual({key: parameters[key] for key in configured}, configured)
                explicit = module.gaussian_parameters(module.parse_arguments(flags + [
                    '--gaussian-persist-submaps', '--gaussian-finalize-on-shutdown',
                    '--gaussian-finalize-timeout-ms', '12345']))
                self.assertIs(explicit['gaussian.persist_submaps'], True)
                self.assertIs(explicit['gaussian.finalize_on_shutdown'], True)
                self.assertEqual(explicit['gaussian.finalize_timeout_ms'], 12345)

    def test_gaussian_arguments_enable_required_storage(self):
        options = run_baseline.parse_arguments(
            ['--workspace', '/unused/workspace', '--bag', '/unused/bag',
             '--output', '/unused/output', '--gaussian-socket', '/tmp/worker.sock',
             '--gaussian-output', '/tmp/gaussian-test-output'])
        self.assertEqual(options.submap_mode, 'online')
        self.assertTrue(options.save_keyframes)
        self.assertTrue(options.save_submaps)
        self.assertFalse(options.loop_closure)
        self.assertFalse(options.global_output)

    def test_geometry_map_enables_only_its_required_inputs(self):
        for module in (run_baseline, run_ros2_baseline):
            with self.subTest(module=module.__name__):
                required = ['--bag', '/unused/bag', '--output', '/unused/output']
                required += (['--workspace', '/unused/workspace'] if module is run_baseline
                             else ['--executable', '/unused/node'])
                options = module.parse_arguments(required + ['--geometry-map'])
                self.assertTrue(options.geometry_map)
                self.assertTrue(options.global_output)
                self.assertTrue(options.save_keyframes)
                self.assertTrue(options.save_submaps)
                self.assertEqual(options.submap_mode, 'online')
                self.assertFalse(options.loop_closure)
                self.assertFalse(options.geometry_submaps)
                self.assertFalse(options.keyframe_trajectory)

    def test_keyframe_trajectory_enables_only_its_required_inputs(self):
        for module in (run_baseline, run_ros2_baseline):
            with self.subTest(module=module.__name__):
                required = ['--bag', '/unused/bag', '--output', '/unused/output']
                required += (['--workspace', '/unused/workspace'] if module is run_baseline
                             else ['--executable', '/unused/node'])
                options = module.parse_arguments(required + ['--keyframe-trajectory'])
                self.assertTrue(options.keyframe_trajectory)
                self.assertTrue(options.global_output)
                self.assertTrue(options.save_keyframes)
                self.assertTrue(options.save_submaps)
                self.assertEqual(options.submap_mode, 'online')
                self.assertFalse(options.loop_closure)
                self.assertFalse(options.geometry_submaps)

    def test_global_only_enables_online_storage_without_loop_or_geometry(self):
        for module in (run_baseline, run_ros2_baseline):
            with self.subTest(module=module.__name__):
                required = ['--bag', '/unused/bag', '--output', '/unused/output']
                required += (['--workspace', '/unused/workspace'] if module is run_baseline
                             else ['--executable', '/unused/node'])
                defaults = module.parse_arguments(required)
                self.assertFalse(defaults.global_output)
                self.assertEqual(defaults.submap_mode, 'keyframe_groups')
                self.assertFalse(defaults.save_keyframes)
                self.assertFalse(defaults.save_submaps)
                options = module.parse_arguments(required + ['--global-output'])
                self.assertEqual(options.submap_mode, 'online')
                self.assertTrue(options.save_submaps)
                self.assertTrue(options.save_keyframes)
                self.assertFalse(options.loop_closure)
                self.assertFalse(options.geometry_submaps)

class GaussianFinalizationRunnerTest(unittest.TestCase):
    def arguments(self, root, *flags):
        return ['--workspace', str(root / 'ws'), '--bag', str(root / 'bag'),
                '--output', str(root / 'output'), *flags]

    def test_finalization_requires_persistence_and_worker_before_io(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            for flags in (['--gaussian-finalize-on-shutdown'],
                          ['--gaussian-finalize-on-shutdown', '--gaussian-socket',
                           str(root / 's'), '--gaussian-output', str(root / 'g')]):
                with self.subTest(flags=flags), self.assertRaises(SystemExit) as error:
                    run_baseline.parse_arguments(self.arguments(root, *flags))
                self.assertEqual(error.exception.code, 2)
            self.assertFalse((root / 'output').exists())

    def test_budget_is_strict_bounded_and_sets_shutdown_wait(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            flags = ['--gaussian-socket', str(root / 's'), '--gaussian-output', str(root / 'g'),
                     '--gaussian-persist-submaps', '--gaussian-finalize-on-shutdown']
            for value in ('0', '-1', '600001', '1.5', 'true'):
                with self.subTest(value=value), self.assertRaises(SystemExit):
                    run_baseline.parse_arguments(self.arguments(
                        root, *flags, '--gaussian-finalize-timeout-ms', value))
            default = run_baseline.parse_arguments(self.arguments(root))
            self.assertEqual(run_baseline.shutdown_wait_seconds(default), 10)
            for budget, wait in ((1, 90.001), (60000, 150), (600000, 690)):
                args = run_baseline.parse_arguments(self.arguments(
                    root, *flags, '--gaussian-finalize-timeout-ms', str(budget)))
                self.assertAlmostEqual(run_baseline.shutdown_wait_seconds(args), wait)

    def child(self, handler):
        import os
        import signal
        code = ('import signal,time\n' + handler + '\nprint("ready", flush=True)\n'
                'while True: time.sleep(.01)\n')
        child = subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.PIPE,
                                 text=True, start_new_session=True)
        self.assertEqual(child.stdout.readline().strip(), 'ready')
        def cleanup():
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
            child.stdout.close()
        self.addCleanup(cleanup)
        return child

    def test_shutdown_waits_for_real_delayed_exit_and_samples(self):
        child = self.child('def finish(*_):\n time.sleep(.2)\n raise SystemExit(0)\n'
                           'signal.signal(signal.SIGINT, finish)')
        samples = []
        result = run_baseline.stop_process(child, timeout=1, sample=samples.append)
        self.assertEqual(child.returncode, 0)
        self.assertFalse(result['forced_termination'])
        self.assertGreaterEqual(result['elapsed_seconds'], .18)
        self.assertGreater(len(samples), 1)

    def test_shutdown_records_forced_termination_and_already_exited(self):
        child = self.child('signal.signal(signal.SIGINT, signal.SIG_IGN)')
        result = run_baseline.stop_process(child, timeout=.1)
        self.assertTrue(result['forced_termination'])
        self.assertNotEqual(child.returncode, 0)
        repeated = run_baseline.stop_process(child, timeout=.1)
        self.assertFalse(repeated['forced_termination'])
