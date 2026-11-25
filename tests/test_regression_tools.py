import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
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


class RegressionToolsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='regression tools ')
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def project(self, script=None):
        source = self.root / 'source'
        source.mkdir()
        tests = ''
        if script is not None:
            path = source / 'case.py'
            path.write_text(script)
            tests = ('add_test(NAME case COMMAND "' + sys.executable +
                     '" "' + str(path) + '")\n')
        (source / 'CMakeLists.txt').write_text(
            'cmake_minimum_required(VERSION 3.16)\n'
            'project(livo_atlas NONE)\n'
            'set(LIVO_ATLAS_ROS_VERSION 1 CACHE STRING "")\n'
            'include(CTest)\nadd_custom_target(tests)\n' + tests)
        build = self.root / 'build'
        subprocess.run(['cmake', '-S', str(source), '-B', str(build)],
                       check=True, stdout=subprocess.DEVNULL,
                       stderr=subprocess.STDOUT)
        return build

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
                             {'ROS_DISTRO': 'noetic', 'ROS_VERSION': '1'}):
            with contextlib.redirect_stdout(io.StringIO()):
                code = run_regression.main([
                    '--ros-build', str(build), '--output', str(out)])
        self.assertEqual(code, 0)
        report = json.loads((out / 'summary.json').read_text())
        self.assertEqual(report['stages']['ros']['tests'], 1)
        self.assertEqual(report['stages']['ros']['ros_version'], '1')
        self.assertEqual(report['stages']['sdk']['status'], 'not_requested')

    def test_existing_output_is_never_replaced(self):
        out = self.root / 'out'
        out.mkdir()
        sentinel = out / 'keep.txt'
        sentinel.write_text('original')
        with self.assertRaises((ValueError, FileExistsError)):
            run_regression.main(['--sdk-build', str(self.root),
                                 '--output', str(out)])
        self.assertEqual(sentinel.read_text(), 'original')

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
                        '--output', str(out)])
        self.assertEqual(code, 130)
        consumer.assert_not_called()
        report = json.loads((out / 'summary.json').read_text())
        self.assertEqual(report['status'], 'interrupted')
        self.assertEqual(report['artifacts']['removed_files'], 0)

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


if __name__ == '__main__':
    unittest.main()
