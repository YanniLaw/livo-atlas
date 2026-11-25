import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from tools import run_regression, timing_analysis


class TimingAnalysisTest(unittest.TestCase):
    def test_dynamic_counts_totals_percentiles_and_cache(self):
        lines = [f'livo_atlas timing stage=work seconds={v}\n'
                 for v in [1, 2, 3, 4]]
        lines += ['livo_atlas update kind=lio frontend_process_seconds=.5',
                  'livo_atlas loop_geometry_cache submap=8 pose_version=7 '
                  'reader_reset=1 built=1 cloud_reads=2 evictions=0 '
                  'resident_submaps=1']
        result = timing_analysis.analyze_lines(iter(lines))
        self.assertEqual(result['stages']['work']['count'], 4)
        self.assertEqual(result['stages']['work']['total'], 10)
        self.assertAlmostEqual(result['stages']['work']['p95'], 3.85)
        self.assertEqual(result['frontend']['lio']['count'], 1)
        self.assertEqual(result['cache']['builds'], 1)

    def test_missing_and_invalid_samples_fail(self):
        for value in ['nan', '-1', 'inf', 'bad']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                timing_analysis.analyze_lines([
                    'livo_atlas timing stage=work seconds=' + value])
        with self.assertRaises(ValueError):
            timing_analysis.analyze_lines(['ordinary log'])

    def test_thread_configuration_records_instances_without_guessing_teams(self):
        lines = ['livo_atlas timing stage=work seconds=.1',
                 'frontend.openmp_threads=2 openmp_available=true',
                 'frontend.openmp_threads=1 openmp_available=false, using one thread',
                 'frontend.openmp_threads=2 openmp_available=true',
                 'pose_graph.num_background_threads=4',
                 'pose_graph.num_background_threads=6']
        result = timing_analysis.analyze_lines(lines)['thread_configuration']
        self.assertEqual(result['frontend'], [
            {'openmp_threads': 2, 'openmp_available': True},
            {'openmp_threads': 1, 'openmp_available': False}])
        self.assertEqual(result['pose_graph'], [
            {'background_threads': 4}, {'background_threads': 6}])
        old = timing_analysis.analyze_lines(lines[:1])['thread_configuration']
        self.assertEqual(old, {'frontend': [], 'pose_graph': []})

    def test_invalid_thread_configuration_fails(self):
        for line in ['frontend.openmp_threads=0 openmp_available=true',
                     'pose_graph.num_background_threads=-1',
                     'frontend.openmp_threads=bad openmp_available=true']:
            with self.subTest(line=line), self.assertRaises(ValueError):
                timing_analysis.analyze_lines([
                    'livo_atlas timing stage=work seconds=.1', line])


class RegressionToolsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='regression tools ')
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def project(self, script=None, metadata=False, nested=False):
        source = self.root / 'source'
        source.mkdir()
        tests = ''
        if script is not None:
            path = source / 'case.py'
            path.write_text(script)
            tests = ('add_test(NAME case COMMAND "' + sys.executable +
                     '" "' + str(path) + '")\n')
        (source / 'CMakeLists.txt').write_text(
            'cmake_minimum_required(VERSION 3.16)\n' +
            ('project(livo_atlas CXX)\n' if metadata else 'project(livo_atlas NONE)\n') +
            'set(LIVO_ATLAS_ROS_VERSION 1 CACHE STRING "")\n'
            'include(CTest)\nadd_custom_target(tests)\n' + tests +
            (('include(GNUInstallDirs)\n'
              'set(EIGEN3_VERSION_STRING 3.4.0)\nset(PCL_VERSION 1.12.1)\n'
              'set(OpenCV_VERSION 4.5.4)\nset(GTSAM_VERSION 4.2.2)\n'
              'set(LIVO_ATLAS_OPENMP_AVAILABLE FALSE)\ninclude("' +
              str(run_regression.PACKAGE / 'cmake/build_info.cmake') + '")\n')
             if metadata else ''))
        if nested:
            package = source / 'package'
            package.mkdir()
            (source / 'CMakeLists.txt').rename(package / 'CMakeLists.txt')
            (source / 'CMakeLists.txt').write_text(
                'cmake_minimum_required(VERSION 3.16)\n'
                'project(workspace NONE)\ninclude(CTest)\n'
                'add_subdirectory(package livo-atlas)\n')
        build = self.root / 'build'
        subprocess.run(['cmake', '-S', str(source), '-B', str(build)],
                       check=True, stdout=subprocess.DEVNULL,
                       stderr=subprocess.STDOUT)
        return build

    def test_configured_build_metadata_and_install(self):
        build = self.project('print("ran")', metadata=True)
        output = self.root / 'result'
        output.mkdir()
        result = run_regression.read_build_info(build, output)
        self.assertEqual(result['status'], 'recorded')
        self.assertTrue(result['compiler']['id'])
        self.assertTrue(result['compiler']['version'])
        self.assertEqual(result['dependencies'], dict(
            eigen='3.4.0', pcl='1.12.1', opencv='4.5.4', gtsam='4.2.2'))
        self.assertFalse(result['openmp_available'])
        self.assertEqual(result['target']['ros_version'], '1')
        self.assertTrue((output / 'livo_atlas_build_info.txt').is_file())
        prefix = self.root / 'installed sdk'
        subprocess.run(['cmake', '--install', str(build), '--prefix', str(prefix)],
                       check=True, stdout=subprocess.DEVNULL)
        self.assertTrue((prefix / 'share/livo_atlas/livo_atlas_build_info.txt').is_file())

    def test_nested_package_build_metadata_uses_configured_binary_directory(self):
        build = self.project('print("ran")', metadata=True, nested=True)
        output = self.root / 'result'
        output.mkdir()
        self.assertFalse((build / 'livo_atlas_build_info.txt').exists())
        self.assertTrue((build / 'livo-atlas/livo_atlas_build_info.txt').is_file())
        result = run_regression.read_build_info(build, output)
        self.assertEqual(result['status'], 'recorded')
        self.assertEqual(result['dependencies']['eigen'], '3.4.0')

    def failed_rebuild(self, invalid=False):
        build = self.project('print("ran")', metadata=True)
        source = self.root / 'source/CMakeLists.txt'
        text = source.read_text().replace('EIGEN3_VERSION_STRING 3.4.0',
                                          'EIGEN3_VERSION_STRING 3.4.1')
        if invalid:
            text += ('file(WRITE "${PROJECT_BINARY_DIR}/livo_atlas_build_info.txt" '
                     '"schema_version=99\\n")\n')
        text += 'add_custom_target(broken ALL COMMAND ${CMAKE_COMMAND} -E false)\n'
        source.write_text(text)
        output = self.root / 'result'
        output.mkdir()
        with mock.patch.dict(os.environ,
                             {'ROS_DISTRO': 'noetic', 'ROS_VERSION': '1'}):
            with self.assertRaisesRegex(RuntimeError, 'Command exited'):
                run_regression.build_and_test(build, output, 1, None, 30, ros=True)
        return output

    def test_failed_rebuild_refreshes_dependency_versions(self):
        output = self.failed_rebuild()
        result = json.loads((output / 'build-info.json').read_text())
        self.assertEqual(result['dependencies']['eigen'], '3.4.1')

    def test_failed_rebuild_does_not_keep_stale_valid_metadata(self):
        output = self.failed_rebuild(invalid=True)
        result = json.loads((output / 'build-info.json').read_text())
        self.assertEqual(result['status'], 'invalid')
        self.assertEqual((output / 'livo_atlas_build_info.txt').read_text(),
                         'schema_version=99\n')

    def test_missing_build_metadata_is_explicit(self):
        output = self.root / 'out'
        output.mkdir()
        result = run_regression.read_build_info(self.root, output)
        self.assertEqual(result['status'], 'unavailable')
        self.assertNotIn('dependencies', result)

    def test_empty_dependency_version_remains_unknown(self):
        build = self.project('print("ran")', metadata=True)
        path = build / 'livo_atlas_build_info.txt'
        path.write_text(path.read_text().replace('eigen_version=3.4.0', 'eigen_version='))
        output = self.root / 'result'
        output.mkdir()
        result = run_regression.read_build_info(build, output)
        self.assertIsNone(result['dependencies']['eigen'])

    def test_failing_tests_keep_build_metadata(self):
        build = self.project('raise SystemExit(1)', metadata=True)
        output = self.root / 'result'
        output.mkdir()
        with mock.patch.dict(os.environ,
                             {'ROS_DISTRO': 'noetic', 'ROS_VERSION': '1'}):
            with self.assertRaises(RuntimeError):
                run_regression.build_and_test(build, output, 1, None, 30, ros=True)
        report = json.loads((output / 'build-info.json').read_text())
        self.assertEqual(report['status'], 'recorded')
        self.assertEqual(report['dependencies']['eigen'], '3.4.0')

    def test_bad_build_metadata_is_preserved_and_rejected(self):
        text = ('schema_version=1\ncompiler_id=GNU\ncompiler_version=9.4.0\n'
                'system_name=Linux\nsystem_processor=x86_64\nros_version=0\n'
                'build_type=Release\neigen_version=3.3.7\npcl_version=1.10.0\n'
                'opencv_version=4.2.0\ngtsam_version=4.2.2\nopenmp_available=FALSE\n')
        for index, data in enumerate([
                text.replace('schema_version=1', 'schema_version=2'),
                text.replace('openmp_available=FALSE', 'openmp_available=maybe'),
                'schema_version=1\n']):
            with self.subTest(index=index):
                (self.root / 'livo_atlas_build_info.txt').write_text(data)
                output = self.root / ('out-' + str(index))
                output.mkdir()
                with self.assertRaises(RuntimeError):
                    run_regression.read_build_info(self.root, output)
                self.assertEqual((output / 'livo_atlas_build_info.txt').read_text(), data)

    def test_real_ctest_success(self):
        build = self.project('print("ran")')
        out = self.root / 'result'
        out.mkdir()
        self.assertEqual(run_regression.run_ctest(build, out)['tests'], 1)

    def test_real_ctest_failure_is_not_a_pass(self):
        build = self.project('raise SystemExit(1)')
        out = self.root / 'result'
        out.mkdir()
        with self.assertRaises(RuntimeError):
            run_regression.run_ctest(build, out)
        self.assertTrue((out / 'ctest.xml').is_file())

    def test_zero_tests_is_not_a_pass(self):
        build = self.project()
        out = self.root / 'result'
        out.mkdir()
        with self.assertRaisesRegex(RuntimeError, 'No tests'):
            run_regression.run_ctest(build, out)

    def test_failing_gtest_xml_even_when_command_returns_zero(self):
        build = self.project(
            'from pathlib import Path\n'
            'p=Path.cwd()/"test_results"\np.mkdir(exist_ok=True)\n'
            '(p/"case.xml").write_text('
            '"<testsuites tests=\'1\' failures=\'1\' errors=\'0\'/>")\n')
        out = self.root / 'result'
        out.mkdir()
        with self.assertRaisesRegex(RuntimeError, 'GTest'):
            run_regression.run_ctest(build, out)
        self.assertTrue((out / 'gtest/case.xml').is_file())

    def test_configured_catkin_results_cannot_hide_failing_gtest(self):
        results = self.root / 'custom catkin results'
        build = self.project(
            'from pathlib import Path\n'
            'p=Path(' + repr(str(results)) + ')\np.mkdir()\n'
            '(p/"case.xml").write_text('
            '"<testsuites tests=\'1\' failures=\'1\' errors=\'0\'/>")\n')
        with (build / 'CMakeCache.txt').open('a') as stream:
            stream.write('\nCATKIN_TEST_RESULTS_DIR:PATH=' + str(results) + '\n')
        out = self.root / 'result'
        out.mkdir()
        with self.assertRaisesRegex(RuntimeError, 'GTest'):
            run_regression.run_ctest(build, out)

    def test_missing_requested_ros_environment_is_explicit(self):
        build = self.project('print("ran")')
        out = self.root / 'out'
        with mock.patch.dict(os.environ, {'ROS_DISTRO': '', 'ROS_VERSION': ''}):
            with contextlib.redirect_stdout(io.StringIO()):
                code = run_regression.main([
                    '--ros-build', str(build), '--output', str(out)])
        self.assertEqual(code, 2)
        report = json.loads((out / 'summary.json').read_text())
        self.assertEqual(report['stages']['ros']['status'], 'unavailable')
        self.assertEqual(report['stages']['sdk']['status'], 'not_requested')

    def test_runner_records_real_requested_scope(self):
        build = self.project('print("ran")')
        out = self.root / 'out'
        with mock.patch.dict(os.environ,
                             {'ROS_DISTRO': 'noetic', 'ROS_VERSION': '1',
                              'OMP_THREAD_LIMIT': '7'}):
            with contextlib.redirect_stdout(io.StringIO()):
                code = run_regression.main([
                    '--ros-build', str(build), '--output', str(out)])
        self.assertEqual(code, 0)
        report = json.loads((out / 'summary.json').read_text())
        self.assertEqual(report['stages']['ros']['tests'], 1)
        self.assertEqual(report['stages']['ros']['ros_version'], '1')
        self.assertEqual(report['stages']['sdk']['status'], 'not_requested')
        self.assertEqual(report['thread_environment']['OMP_THREAD_LIMIT'], '7')
        self.assertEqual(report['stages']['ros']['build_info']['status'], 'unavailable')

    def test_existing_output_is_never_replaced(self):
        out = self.root / 'out'
        out.mkdir()
        sentinel = out / 'keep.txt'
        sentinel.write_text('original')
        with self.assertRaises((ValueError, FileExistsError)):
            run_regression.main(['--sdk-build', str(self.root),
                                 '--output', str(out)])
        self.assertEqual(sentinel.read_text(), 'original')

    def test_existing_output_symlinks_are_rejected_before_resolving(self):
        for target_exists in [False, True]:
            with self.subTest(target_exists=target_exists):
                target = self.root / ('target-' + str(target_exists))
                if target_exists:
                    target.mkdir()
                    (target / 'keep').write_text('original')
                link = self.root / ('link-' + str(target_exists))
                link.symlink_to(target, target_is_directory=True)
                with self.assertRaises(FileExistsError):
                    run_regression.main(['--sdk-build', str(self.root / 'build'),
                                         '--output', str(link)])
                self.assertEqual(target.exists(), target_exists)
                self.assertTrue(link.is_symlink())
                if target_exists:
                    self.assertEqual((target / 'keep').read_text(), 'original')

    def test_dangling_output_parent_is_rejected(self):
        target = self.root / 'missing-parent'
        link = self.root / 'linked-parent'
        link.symlink_to(target, target_is_directory=True)
        with self.assertRaises(FileExistsError):
            run_regression.main(['--sdk-build', str(self.root / 'build'),
                                 '--output', str(link / 'result')])
        self.assertFalse(target.exists())

    def test_missing_prefix_cannot_hide_dangling_output_alias(self):
        for parent in [False, True]:
            with self.subTest(parent=parent):
                target = self.root / ('target-' + str(parent))
                link = self.root / ('link-' + str(parent))
                link.symlink_to(target, target_is_directory=True)
                output = self.root / 'missing/..' / link.name
                if parent:
                    output = output / 'result'
                with contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaises((ValueError, FileExistsError)):
                        run_regression.main([
                            '--sdk-build', str(self.root / 'missing-build'),
                            '--output', str(output)])
                self.assertFalse(target.exists())
                self.assertFalse((self.root / 'missing').exists())

    def test_existing_parent_link_allows_new_output(self):
        link = self.root / 'linked-parent'
        link.symlink_to(self.root, target_is_directory=True)
        with contextlib.redirect_stdout(io.StringIO()):
            code = run_regression.main([
                '--sdk-build', str(self.root / 'missing-build'),
                '--output', str(link / 'new-output')])
        self.assertEqual(code, 2)
        self.assertTrue((self.root / 'new-output/summary.json').is_file())

    def test_failed_sdk_blocks_implicit_benchmark(self):
        output = self.root / 'out'
        with contextlib.redirect_stdout(io.StringIO()):
            code = run_regression.main([
                '--sdk-build', str(self.root / 'missing-build'),
                '--dataset', str(self.root / 'dataset'), '--output', str(output)])
        self.assertEqual(code, 2)
        report = json.loads((output / 'summary.json').read_text())
        self.assertEqual(report['stages']['benchmark']['status'], 'blocked')
        self.assertTrue(report['stages']['benchmark']['requested'])

    def test_consumer_failure_blocks_implicit_benchmark(self):
        output = self.root / 'out'
        with mock.patch.object(run_regression, 'build_and_test', return_value={}):
            with mock.patch.object(run_regression, 'install_consumer',
                                   side_effect=RuntimeError('installation failed')):
                with contextlib.redirect_stdout(io.StringIO()):
                    code = run_regression.main([
                        '--sdk-build', str(self.root / 'build'),
                        '--dataset', str(self.root / 'dataset'),
                        '--output', str(output)])
        self.assertEqual(code, 1)
        report = json.loads((output / 'summary.json').read_text())
        self.assertEqual(report['stages']['benchmark']['status'], 'blocked')

    def test_explicit_benchmark_still_runs_when_sdk_is_unavailable(self):
        output = self.root / 'out'
        with mock.patch.object(run_regression, 'run_benchmark', return_value={}) as run:
            with contextlib.redirect_stdout(io.StringIO()):
                code = run_regression.main([
                    '--sdk-build', str(self.root / 'missing-build'),
                    '--dataset', str(self.root / 'dataset'),
                    '--benchmark-executable', str(self.root / 'explicit-binary'),
                    '--output', str(output)])
        self.assertEqual(code, 2)
        run.assert_called_once()
        report = json.loads((output / 'summary.json').read_text())
        self.assertEqual(report['stages']['benchmark']['status'], 'passed')

    def test_replay_output_is_created_by_existing_replay_tool(self):
        out = self.root / 'out'

        def replay(args, output, prefix):
            self.assertFalse((output / 'replay').exists())
            (output / 'replay').mkdir()
            return {'directory': str(output / 'replay')}

        with mock.patch.object(run_regression, 'run_replay', side_effect=replay):
            with contextlib.redirect_stdout(io.StringIO()):
                code = run_regression.main([
                    '--replay', 'ros1', '--output', str(out), '--replay-args',
                    '--workspace', str(self.root / 'snapshot'),
                    '--bag', str(self.root / 'input.bag')])
        self.assertEqual(code, 0)

    def test_interrupt_retains_results_and_stops_later_stages(self):
        out = self.root / 'out'
        with mock.patch.object(run_regression, 'build_and_test',
                               side_effect=KeyboardInterrupt):
            with mock.patch.object(run_regression, 'install_consumer') as consumer:
                with contextlib.redirect_stdout(io.StringIO()):
                    code = run_regression.main([
                        '--sdk-build', str(self.root / 'build'),
                        '--ros-build', str(self.root / 'ros-build'),
                        '--dataset', str(self.root / 'dataset'),
                        '--output', str(out)])
        self.assertEqual(code, 130)
        consumer.assert_not_called()
        report = json.loads((out / 'summary.json').read_text())
        self.assertEqual(report['status'], 'interrupted')
        self.assertEqual(report['artifacts']['removed_files'], 0)
        for name in ['consumer', 'ros', 'benchmark']:
            self.assertEqual(report['stages'][name]['status'], 'interrupted')
            self.assertTrue(report['stages'][name]['requested'])
        self.assertEqual(report['stages']['replay']['status'], 'not_requested')

    def test_artifact_interrupt_is_reported(self):
        output = self.root / 'out'
        with mock.patch.object(run_regression, 'build_and_test', return_value={}):
            with mock.patch.object(run_regression, 'finalize_artifacts',
                                   side_effect=KeyboardInterrupt):
                with contextlib.redirect_stdout(io.StringIO()):
                    try:
                        code = run_regression.main([
                            '--ros-build', str(self.root / 'build'),
                            '--output', str(output)])
                    except KeyboardInterrupt:
                        self.fail('Artifact interruption was not recorded')
        self.assertEqual(code, 130)
        report = json.loads((output / 'summary.json').read_text())
        self.assertEqual(report['status'], 'interrupted')
        self.assertEqual(report['stages']['ros']['status'], 'passed')

    def test_stage_summary_interrupt_reconciles_running_and_pending(self):
        output = self.root / 'result'
        original = run_regression.write_json
        interrupted = False

        def write(path, report):
            nonlocal interrupted
            if path.name == 'summary.json' and not interrupted:
                interrupted = True
                raise KeyboardInterrupt
            original(path, report)

        with mock.patch.object(run_regression, 'write_json', side_effect=write):
            with contextlib.redirect_stdout(io.StringIO()):
                code = run_regression.main([
                    '--sdk-build', str(self.root / 'build'), '--output', str(output)])
        self.assertEqual(code, 130)
        report = json.loads((output / 'summary.json').read_text())
        self.assertEqual(report['stages']['sdk']['status'], 'interrupted')
        self.assertEqual(report['stages']['consumer']['status'], 'interrupted')

    def test_metadata_collection_interrupt_keeps_a_report(self):
        output = self.root / 'result'
        with mock.patch.object(run_regression, 'source_info', side_effect=KeyboardInterrupt):
            with contextlib.redirect_stdout(io.StringIO()):
                try:
                    code = run_regression.main([
                        '--sdk-build', str(self.root / 'build'), '--output', str(output)])
                except KeyboardInterrupt:
                    self.fail('Metadata interruption escaped without a report')
        self.assertEqual(code, 130)
        report = json.loads((output / 'summary.json').read_text())
        self.assertEqual(report['status'], 'interrupted')
        self.assertEqual(report['stages']['sdk']['status'], 'interrupted')

    def test_final_summary_interrupt_is_reported(self):
        output = self.root / 'result'
        original = run_regression.write_json
        interrupted = False

        def write(path, report):
            nonlocal interrupted
            if path.name == 'summary.json' and report['status'] != 'running' and not interrupted:
                interrupted = True
                raise KeyboardInterrupt
            original(path, report)

        with mock.patch.object(run_regression, 'write_json', side_effect=write):
            with contextlib.redirect_stdout(io.StringIO()):
                try:
                    code = run_regression.main([
                        '--sdk-build', str(self.root / 'missing-build'),
                        '--output', str(output)])
                except KeyboardInterrupt:
                    self.fail('Final summary interruption escaped without a report')
        self.assertEqual(code, 130)
        report = json.loads((output / 'summary.json').read_text())
        self.assertEqual(report['status'], 'interrupted')

    def test_json_replace_interrupt_keeps_previous_snapshot(self):
        path = self.root / 'summary.json'
        original = '{"original": true}\n'
        path.write_text(original)
        with mock.patch.object(Path, 'replace', side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                run_regression.write_json(path, {'new': True})
        self.assertEqual(path.read_text(), original)
        self.assertEqual(list(self.root.iterdir()), [path])

    def replay_files(self):
        output = self.root / 'owned'
        frames = output / 'replay/keyframes/frames/0'
        frames.mkdir(parents=True)
        (frames / 'cloud.pcd').write_bytes(b'generated points')
        (frames / 'frame.yml').write_text('metadata')
        original = self.root / 'input'
        original.mkdir()
        (original / 'cloud.pcd').write_bytes(b'original points')
        (frames / 'linked.pcd').symlink_to(original / 'cloud.pcd')
        return output, frames, original

    def test_compact_keeps_metadata_manifest_and_input(self):
        output, frames, original = self.replay_files()
        result = run_regression.finalize_artifacts(output, True, 'compact')
        self.assertFalse((frames / 'cloud.pcd').exists())
        self.assertTrue((frames / 'frame.yml').exists())
        self.assertEqual((original / 'cloud.pcd').read_bytes(), b'original points')
        self.assertTrue((frames / 'linked.pcd').is_symlink())
        self.assertEqual(result['removed_files'], 1)
        manifest = json.loads((output / 'replay-files.json').read_text())
        self.assertTrue(manifest['files']['keyframes/frames/0/cloud.pcd']['sha256'])

    def test_failed_or_all_mode_keeps_generated_payload(self):
        output, frames, _ = self.replay_files()
        for success, retain in [(False, 'compact'), (True, 'all')]:
            result = run_regression.finalize_artifacts(output, success, retain)
            self.assertEqual(result['removed_files'], 0)
            self.assertTrue((frames / 'cloud.pcd').exists())

    def test_compact_removes_both_ros2_product_copies(self):
        output, _, _ = self.replay_files()
        for directory in ['products', 'node-output']:
            path = output / 'replay' / directory / 'PCD/scan.pcd'
            path.parent.mkdir(parents=True)
            path.write_bytes(b'generated product')
        result = run_regression.finalize_artifacts(output, True, 'compact')
        self.assertEqual(result['removed_files'], 3)
        self.assertFalse((output / 'replay/node-output/PCD/scan.pcd').exists())

    def test_replay_cannot_redirect_output_to_external_directories(self):
        for flags in [['--legacy-output', '--save-products'],
                      ['--gaussian-socket', str(self.root / 'worker.sock'),
                       '--gaussian-output', str(self.root / 'external')]]:
            with self.subTest(flags=flags), self.assertRaisesRegex(
                    ValueError, 'output|Output'):
                run_regression.arguments([
                    '--replay', 'ros1', '--output', str(self.root / 'new'),
                    '--replay-args', '--workspace', str(self.root / 'snapshot'),
                    '--bag', str(self.root / 'input.bag')] + flags)
        self.assertFalse((self.root / 'new').exists())

    def test_replay_summary_cannot_hide_failure_or_empty_input(self):
        for report in [{}, {'clean_shutdown': True, 'lio_cycles': 0},
                       {'clean_shutdown': False, 'lio_cycles': 3},
                       {'clean_shutdown': True, 'lio_cycles': 3,
                        'trajectory_rows': 3, 'errors': ['failed']}]:
            with self.assertRaises(RuntimeError):
                run_regression.check_replay_summary(report, 'ros1')

    def test_inconsistent_benchmark_report_is_not_a_pass(self):
        executable = self.root / 'benchmark'
        report = dict(schema_version=1, geometry_matches=True, frame_count=3,
                      submap_count=2, samples=[{}],
                      reference_fingerprint='0' * 16,
                      mapper_fingerprint='1' * 16)
        executable.write_text('#!' + sys.executable + '\nprint(' +
                              repr(json.dumps(report)) + ')\n')
        executable.chmod(0o700)
        out = self.root / 'out'
        with contextlib.redirect_stdout(io.StringIO()):
            code = run_regression.main([
                '--dataset', str(self.root / 'dataset'),
                '--benchmark-executable', str(executable), '--rounds', '1',
                '--output', str(out)])
        self.assertEqual(code, 1)
        result = json.loads((out / 'summary.json').read_text())
        self.assertEqual(result['stages']['benchmark']['status'], 'failed')

    def test_benchmark_version_and_timing_samples_are_validated(self):
        sample = dict(round=0, reference_first=1,
                      **{key: .1 for key in (
                          'reference_add_seconds', 'reference_snapshot_seconds',
                          'mapper_total_seconds', 'mapper_build_seconds',
                          'mapper_snapshot_seconds', 'mapper_read_seconds')})
        for index, (version, value) in enumerate([
                (0, sample), (3, {}),
                (3, dict(sample, mapper_total_seconds=float('nan'))),
                (3, dict(sample, mapper_total_seconds=-.1))]):
            with self.subTest(version=version, sample=value):
                report = dict(schema_version=1, geometry_matches=True,
                              pose_version=version, frame_count=3, submap_count=2,
                              samples=[value], reference_fingerprint='0' * 16,
                              mapper_fingerprint='0' * 16)
                executable = self.root / ('benchmark-' + str(index))
                executable.write_text('#!' + sys.executable + '\nprint(' +
                                      repr(json.dumps(report)) + ')\n')
                executable.chmod(0o700)
                with contextlib.redirect_stdout(io.StringIO()):
                    code = run_regression.main([
                        '--dataset', str(self.root / 'dataset'),
                        '--benchmark-executable', str(executable),
                        '--pose-version', '3', '--rounds', '1',
                        '--output', str(self.root / ('result-' + str(index)))])
                self.assertEqual(code, 1)

    def run_replay_fixture(self, name, mode, reported_loop=False,
                           requested_loop=False):
        output = self.root / name
        script = self.root / (name + '.py')
        report = dict(clean_shutdown=True, passed=True, lio_cycles=3,
                      trajectory_rows=3, loop_closure_enabled=reported_loop)
        log = ('livo_atlas timing stage=work seconds=.1\n'
               'frontend.openmp_threads=2 openmp_available=true\n'
               'pose_graph.num_background_threads=4\n')
        script.write_text(
            'import json,sys\nfrom pathlib import Path\n'
            'p=Path(sys.argv[-1]);p.mkdir()\n'
            '(p/"summary.json").write_text(' + repr(json.dumps(report)) + ')\n'
            'f=p/"keyframes/frames/0";f.mkdir(parents=True)\n'
            '(f/"cloud.pcd").write_bytes(b"keep on failure")\n'
            '(p/"node.log").write_text(' +
            repr('stdout banner\n' if mode == 'ros2' else log) + ')\n'
            '(p/"node-stderr.log").write_text(' + repr(log if mode == 'ros2' else '') +
            ')\n')
        options = SimpleNamespace(bag=self.root / 'input.bag',
                                  loop_closure=requested_loop)
        with mock.patch.object(run_regression, 'replay_options',
                               return_value=(script, options)):
            with mock.patch.dict(os.environ, {'ROS_DISTRO':
                                 'humble' if mode == 'ros2' else 'noetic'}):
                with contextlib.redirect_stdout(io.StringIO()):
                    code = run_regression.main([
                        '--replay', mode, '--output', str(output)])
        return code, output

    def test_ros2_replay_reads_glog_from_stderr(self):
        code, output = self.run_replay_fixture('humble', 'ros2')
        self.assertEqual(code, 0)
        timings = json.loads((output / 'replay-timing.json').read_text())
        self.assertEqual(timings['stages']['work']['count'], 1)
        report = json.loads((output / 'summary.json').read_text())
        self.assertEqual(report['stages']['replay']['thread_configuration'],
                         timings['thread_configuration'])

    def test_missing_loop_archive_retains_payload(self):
        for reported, requested in [(True, False), (False, True)]:
            with self.subTest(reported=reported, requested=requested):
                code, output = self.run_replay_fixture(
                    'loop-' + str(reported), 'ros1', reported, requested)
                self.assertEqual(code, 1)
                self.assertTrue((output / 'replay/keyframes/frames/0/cloud.pcd').exists())
                report = json.loads((output / 'summary.json').read_text())
                self.assertEqual(report['artifacts']['removed_files'], 0)


if __name__ == '__main__':
    unittest.main()
