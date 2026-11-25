#!/usr/bin/env python3
"""Run explicitly selected, configured LIVO builds and optional data checks."""

import argparse
from contextlib import ExitStack
import hashlib
import importlib
from itertools import chain
import json
import math
import os
import platform
import re
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

try:
    from .timing_analysis import analyze_lines
except ImportError:
    from timing_analysis import analyze_lines


PACKAGE = Path(__file__).resolve().parents[1]
BULK_SUFFIXES = {'.pcd', '.ply', '.png', '.jpg', '.jpeg', '.exr'}


class Unavailable(RuntimeError):
    pass


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            result.update(block)
    return result.hexdigest()


def source_info(source):
    result = dict(directory=str(source))
    try:
        result['commit'] = subprocess.check_output(
            ['git', '-C', str(source), 'rev-parse', 'HEAD'],
            stderr=subprocess.DEVNULL, text=True).strip()
        result['dirty'] = bool(subprocess.check_output(
            ['git', '-C', str(source), 'status', '--porcelain'],
            stderr=subprocess.DEVNULL, text=True).strip())
    except (OSError, subprocess.CalledProcessError):
        result['git_available'] = False
    return result


def cache_values(build):
    path = build / 'CMakeCache.txt'
    if not path.is_file():
        raise Unavailable('No configured CMake build: ' + str(build))
    values = {}
    for line in path.read_text().splitlines():
        if line and not line.startswith(('#', '//')) and '=' in line:
            key, value = line.split('=', 1)
            values[key.split(':', 1)[0]] = value
    return values


def command(arguments, log, cwd=None, env=None, timeout=1800):
    if shutil.which(str(arguments[0]), path=(env or os.environ).get('PATH')) is None:
        raise Unavailable('Command unavailable: ' + str(arguments[0]))
    with log.open('w') as stream:
        stream.write(json.dumps([str(v) for v in arguments]) + '\n')
        stream.flush()
        process = subprocess.Popen([str(v) for v in arguments], cwd=cwd,
                                   env=env, stdout=stream,
                                   stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            code = process.wait(timeout=timeout)
        except BaseException:
            # Interrupt only this command's own group, allowing replay cleanup.
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            raise
    if code:
        raise RuntimeError('Command exited %d; see %s' % (code, log))


def library_environment(build):
    env = os.environ.copy()
    directories = [str(build)]
    devel = cache_values(build).get('CATKIN_DEVEL_PREFIX')
    if devel:
        directories.append(str(Path(devel) / 'lib'))
    if env.get('LD_LIBRARY_PATH'):
        directories.append(env['LD_LIBRARY_PATH'])
    env['LD_LIBRARY_PATH'] = os.pathsep.join(directories)
    return env


def run_ctest(build, output, pattern=None, timeout=1800):
    build = Path(build).resolve()
    results = Path(cache_values(build).get('CATKIN_TEST_RESULTS_DIR', 'test_results'))
    if not results.is_absolute():
        results = build / results
    started = time.time_ns()
    before = {p: (p.stat().st_mtime_ns, p.stat().st_size)
              for p in results.rglob('*.xml')}
    arguments = ['ctest', '-T', 'Test', '--no-compress-output',
                 '--output-on-failure', '-j', '1']
    if pattern:
        arguments += ['-R', pattern]
    report, gtest_reports = None, []
    try:
        command(arguments, output / 'ctest.log', cwd=build,
                env=library_environment(build), timeout=timeout)
    finally:
        # Preserve this run's XML even when CTest fails or catkin hides an error.
        tag_path = build / 'Testing/TAG'
        if tag_path.is_file():
            tag = tag_path.read_text().splitlines()[0]
            candidate = build / 'Testing' / tag / 'Test.xml'
            if candidate.is_file() and \
                    candidate.stat().st_mtime_ns >= started - 1_000_000_000:
                report = output / 'ctest.xml'
                shutil.copy2(str(candidate), str(report))
        for path in results.rglob('*.xml'):
            stat = path.stat()
            if before.get(path) == (stat.st_mtime_ns, stat.st_size):
                continue
            destination = output / 'gtest' / path.relative_to(results)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(str(path), str(destination))
            gtest_reports.append(destination)
    if report is None:
        raise RuntimeError('CTest report is missing or stale')
    tests = ET.parse(str(report)).findall('.//Testing/Test')
    if not tests:
        raise RuntimeError('No tests executed')
    if any(test.get('Status') != 'passed' for test in tests):
        raise RuntimeError('CTest contains failed or unexecuted tests')
    for path in gtest_reports:
        root = ET.parse(str(path)).getroot()
        if root.tag not in ('testsuite', 'testsuites'):
            continue
        if int(root.get('tests', '0')) == 0 or any(
                int(node.get(field, '0')) for node in root.iter()
                for field in ('failures', 'errors')):
            raise RuntimeError('GTest report has failures or no cases: ' + str(path))
    return dict(tests=len(tests), test_names=[t.findtext('Name') for t in tests],
                test_regex=pattern)


def build_and_test(build, output, jobs, pattern, timeout, ros=False):
    values = cache_values(build)
    mode = values.get('LIVO_ATLAS_ROS_VERSION')
    if ros:
        distro = {'1': 'noetic', '2': 'humble'}.get(mode)
        if not distro or os.environ.get('ROS_DISTRO') != distro or \
                os.environ.get('ROS_VERSION') != mode:
            raise Unavailable('Source the matching Noetic/Humble environment')
    elif mode != '0':
        raise Unavailable('SDK build must use LIVO_ATLAS_ROS_VERSION=0')
    command(['cmake', '--build', build, '--parallel', str(jobs)],
            output / 'build.log', timeout=timeout)
    if mode == '1':
        command(['cmake', '--build', build, '--target', 'tests',
                 '--parallel', str(jobs)], output / 'build-tests.log',
                timeout=timeout)
    result = run_ctest(build, output, pattern, timeout)
    result.update(ros_version=mode, build_directory=str(build),
                  configured_source=values.get('CMAKE_HOME_DIRECTORY'),
                  source=source_info(values.get('CMAKE_HOME_DIRECTORY', build)),
                  cache_sha256=digest(build / 'CMakeCache.txt'))
    return result


def install_consumer(build, output, jobs, timeout):
    prefix = output / 'sdk-install'
    command(['cmake', '--install', build, '--prefix', prefix],
            output / 'install.log', timeout=timeout)
    consumer = output / 'consumer-build'
    packages = list(prefix.rglob('livo_atlasConfig.cmake'))
    if len(packages) != 1:
        raise RuntimeError('Expected exactly one installed SDK package')
    arguments = ['cmake', '-S', PACKAGE / 'tests/installed_consumer',
                 '-B', consumer, '-DCMAKE_BUILD_TYPE=Release',
                 '-Dlivo_atlas_DIR=' + str(packages[0].parent)]
    values = cache_values(build)
    for key in ('Eigen3_DIR', 'PCL_DIR', 'OpenCV_DIR', 'Sophus_DIR',
                'glog_DIR', 'GTSAM_DIR', 'CMAKE_C_COMPILER',
                'CMAKE_CXX_COMPILER', 'CMAKE_TOOLCHAIN_FILE'):
        if values.get(key) and not values[key].endswith('-NOTFOUND'):
            arguments.append('-D' + key + '=' + values[key])
    command(arguments, output / 'consumer-configure.log', timeout=timeout)
    command(['cmake', '--build', consumer, '--parallel', str(jobs)],
            output / 'consumer-build.log', timeout=timeout)
    command([consumer / 'installed_consumer'], output / 'consumer.log',
            timeout=timeout)
    return dict(prefix=str(prefix))


def geometry_arguments(path):
    if path is None:
        return []
    try:
        import yaml
    except ImportError as error:
        raise Unavailable('PyYAML is needed for --geometry-config') from error
    fields = yaml.safe_load(path.read_text())['loop_closure']['geometry']
    names = {'voxel_size_m': 'voxel-size', 'min_points_per_voxel': 'min-points',
             'max_plane_rms': 'max-plane-rms', 'max_plane_ratio': 'max-plane-ratio',
             'max_voxels_per_submap': 'max-voxels',
             'max_points_per_keyframe': 'max-points',
             'max_resident_submaps': 'max-resident-submaps'}
    if not isinstance(fields, dict) or fields.keys() - names.keys():
        raise ValueError('Unknown geometry configuration fields')
    return [v for key, value in fields.items()
            for v in ('--' + names[key], str(value))]


def run_benchmark(args, prefix, output):
    executable = args.benchmark_executable
    if executable is None and prefix:
        executable = Path(prefix) / 'bin/livo_atlas_benchmark_geometry'
    if executable is None or not executable.is_file():
        raise Unavailable('Provide --benchmark-executable or a successful SDK stage')
    arguments = [executable, '--dataset', args.dataset,
                 '--pose-version', str(args.pose_version), '--rounds', str(args.rounds),
                 '--max-preload-mb', str(args.max_preload_mb)]
    arguments += geometry_arguments(args.geometry_config)
    # Keep machine stdout separate from GLOG diagnostics.
    with (output / 'geometry.json').open('w') as stream, \
            (output / 'geometry.log').open('w') as errors:
        result = subprocess.run([str(v) for v in arguments], stdout=stream,
                                stderr=errors, timeout=args.timeout)
    if result.returncode:
        raise RuntimeError('Geometry benchmark failed; see geometry.log')
    report = json.loads((output / 'geometry.json').read_text())
    reference = report.get('reference_fingerprint', '')
    if report.get('schema_version') != 1 or \
            report.get('geometry_matches') is not True or \
            report.get('pose_version') != args.pose_version or \
            len(report.get('samples', [])) != args.rounds or \
            report.get('frame_count', 0) <= 0 or report.get('submap_count', 0) <= 0 or \
            not re.fullmatch('[0-9a-f]{16}', reference) or \
            report.get('mapper_fingerprint') != reference:
        raise RuntimeError('Geometry benchmark did not validate its results')
    timings = ('reference_add_seconds', 'reference_snapshot_seconds',
               'mapper_total_seconds', 'mapper_build_seconds',
               'mapper_snapshot_seconds', 'mapper_read_seconds')
    for index, sample in enumerate(report['samples']):
        if sample.get('round') != index or \
                sample.get('reference_first') != (index % 2 == 0) or any(
                    type(sample.get(key)) not in (int, float) or
                    not math.isfinite(sample[key]) or sample[key] < 0
                    for key in timings):
            raise RuntimeError('Invalid geometry benchmark timing sample')
    return dict(command=[str(v) for v in arguments], frame_count=report['frame_count'],
                submap_count=report['submap_count'], rounds=len(report['samples']),
                executable_sha256=digest(executable),
                report_sha256=digest(output / 'geometry.json'))


def check_replay_summary(report, mode):
    passed = report.get('clean_shutdown') if mode == 'ros1' else report.get('passed')
    if passed is not True or report.get('lio_cycles', 0) + report.get('vio_cycles', 0) <= 0 \
            or report.get('trajectory_rows', 0) < 2:
        raise RuntimeError('Replay failed or has no validated input')
    for key in ('errors', 'input_count_mismatches', 'node_input_count_mismatches',
                'output_issues'):
        if report.get(key):
            raise RuntimeError('Replay summary reports ' + key)


def replay_options(args):
    name = 'run_baseline' if args.replay == 'ros1' else 'run_ros2_baseline'
    module = importlib.import_module(('tools.' if __package__ else '') + name)
    parsed = module.parse_arguments(args.replay_args +
                                    ['--output', str(args.output / 'replay')])
    if getattr(parsed, 'legacy_output', False):
        raise ValueError('--legacy-output can write outside this run output')
    gaussian = getattr(parsed, 'gaussian_output', None)
    if gaussian and args.output / 'replay' not in gaussian.resolve().parents:
        raise ValueError('--gaussian-output must be inside this run replay output')
    return PACKAGE / 'tools' / (name + '.py'), parsed


def run_replay(args, output, prefix):
    expected = 'noetic' if args.replay == 'ros1' else 'humble'
    if os.environ.get('ROS_DISTRO') != expected:
        raise Unavailable('Source ' + expected + ' before replay')
    script, options = replay_options(args)
    arguments = [sys.executable, script] + args.replay_args + \
        ['--output', output / 'replay']
    command(arguments, output / 'replay.log', timeout=args.timeout)
    replay = output / 'replay'
    report = json.loads((replay / 'summary.json').read_text())
    check_replay_summary(report, args.replay)
    dataset = replay / 'keyframes'
    archive = dataset / 'loop_closure'
    if options.loop_closure or report.get('loop_closure_enabled') or archive.is_dir():
        if not archive.is_dir():
            raise RuntimeError('Expected loop archive is missing')
        checker = args.loop_checker
        if checker is None and prefix:
            checker = Path(prefix) / 'bin/livo_atlas_check_loops'
        if checker is None and args.ros_build:
            checker = args.ros_build / 'livo_atlas_check_loops'
        if checker is None or not checker.is_file():
            raise Unavailable('A loop checker is required before compacting the replay')
        command([checker, dataset, output / 'replay-inspection'],
                output / 'inspection.log', timeout=args.timeout)
    names = ['mapping.log'] if args.replay == 'ros1' else ['node.log', 'node-stderr.log']
    # The ROS2 runner separates stdout and GLOG stderr; parse both streams.
    logs = [replay / name for name in names if (replay / name).is_file()]
    with ExitStack() as stack:
        streams = [stack.enter_context(path.open(errors='replace')) for path in logs]
        timings = analyze_lines(chain.from_iterable(streams))
    timings['logs'] = [str(path) for path in logs]
    write_json(output / 'replay-timing.json', timings)
    return dict(directory=str(replay), summary=str(replay / 'summary.json'))


def finalize_artifacts(output, success, retain):
    result = dict(mode=retain if success else 'all_on_failure',
                  removed_files=0, removed_bytes=0)
    replay = output / 'replay'
    if not success or not replay.is_dir() or replay.is_symlink():
        return result
    files, removable = {}, []
    for directory, folders, names in os.walk(replay, followlinks=False):
        folders[:] = [n for n in folders if not (Path(directory) / n).is_symlink()]
        for name in names:
            path = Path(directory) / name
            relative = path.relative_to(replay)
            if path.is_symlink() or not path.is_file():
                continue
            files[str(relative)] = dict(size=path.stat().st_size, sha256=digest(path))
            if relative.parts[0] in ('keyframes', 'products', 'node_output', 'node-output') and \
                    path.suffix.lower() in BULK_SUFFIXES:
                removable.append(path)
    # Manifest must reach disk before the first payload is removed.
    write_json(output / 'replay-files.json', dict(schema_version=1, files=files))
    if retain == 'compact':
        for path in removable:
            result['removed_bytes'] += files[str(path.relative_to(replay))]['size']
            path.unlink()
            result['removed_files'] += 1
    return result


def arguments(values):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sdk-build', type=Path)
    parser.add_argument('--ros-build', type=Path)
    parser.add_argument('--dataset', type=Path)
    parser.add_argument('--benchmark-executable', type=Path)
    parser.add_argument('--geometry-config', type=Path)
    parser.add_argument('--pose-version', type=int, default=0)
    parser.add_argument('--rounds', type=int, default=5)
    parser.add_argument('--max-preload-mb', type=int, default=512)
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--timeout', type=float, default=1800)
    parser.add_argument('--ctest-regex')
    parser.add_argument('--retain', choices=('compact', 'all'), default='compact')
    parser.add_argument('--replay', choices=('ros1', 'ros2'))
    parser.add_argument('--loop-checker', type=Path)
    parser.add_argument('--replay-args', nargs=argparse.REMAINDER, default=[])
    args = parser.parse_args(values)
    if not any((args.sdk_build, args.ros_build, args.dataset, args.replay)):
        parser.error('Request at least one build, benchmark or replay')
    if args.jobs <= 0 or args.rounds <= 0 or args.rounds > 1000 or \
            args.pose_version < 0 or not 0 < args.max_preload_mb <= 65536 or \
            not 0 < args.timeout < float('inf'):
        parser.error('Invalid jobs, rounds, version, limit or timeout')
    if args.replay_args and not args.replay:
        parser.error('--replay-args requires --replay')
    for name, value in vars(args).items():
        if isinstance(value, Path):
            setattr(args, name, value.resolve())
    inputs = [args.dataset] if args.dataset else []
    if args.replay:
        _, replay = replay_options(args)
        inputs.append(replay.bag.resolve())
    for path in inputs:
        if args.output == path or path in args.output.parents or args.output in path.parents:
            parser.error('Output must be separate from input data')
    return args


def main(values=None):
    args = arguments(values)
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(schema_version=1, status='running', scope='requested_stages',
                  source=source_info(PACKAGE), platform=platform.platform(),
                  machine=platform.machine(), command=sys.argv if values is None else values,
                  tool_sha256={name: digest(PACKAGE / 'tools' / name) for name in
                               ('run_regression.py', 'timing_analysis.py')},
                  stages={name: dict(status='not_requested') for name in
                          ('sdk', 'consumer', 'ros', 'benchmark', 'replay')})
    prefix = None
    (args.output / 'stages').mkdir()

    def stage(name, action):
        destination = args.output / 'stages' / name
        destination.mkdir()
        report['stages'][name] = dict(status='running')
        write_json(args.output / 'summary.json', report)
        try:
            result = action(destination) or {}
            report['stages'][name] = dict(status='passed', **result)
            return result
        except Unavailable as error:
            report['stages'][name] = dict(status='unavailable', reason=str(error))
        except KeyboardInterrupt:
            report['stages'][name] = dict(status='interrupted')
            raise
        except Exception as error:
            report['stages'][name] = dict(status='failed', reason=str(error))
        finally:
            write_json(args.output / 'summary.json', report)
        return None

    try:
        if args.sdk_build:
            result = stage('sdk', lambda out: build_and_test(
                args.sdk_build, out, args.jobs, args.ctest_regex, args.timeout))
            if result is not None:
                installed = stage('consumer', lambda out: install_consumer(
                    args.sdk_build, out, args.jobs, args.timeout))
                prefix = installed['prefix'] if installed else None
            else:
                report['stages']['consumer'] = dict(
                    status='blocked', reason='SDK stage failed')
        if args.ros_build:
            stage('ros', lambda out: build_and_test(
                args.ros_build, out, args.jobs, args.ctest_regex, args.timeout, ros=True))
        if args.dataset:
            stage('benchmark', lambda out: run_benchmark(args, prefix, out))
        if args.replay:
            stage('replay', lambda _: run_replay(args, args.output, prefix))
    except KeyboardInterrupt:
        report['status'] = 'interrupted'
    statuses = [value['status'] for value in report['stages'].values()]
    if report['status'] != 'interrupted':
        report['status'] = ('failed' if 'failed' in statuses else
                            'incomplete' if any(s in statuses for s in
                                                ('unavailable', 'blocked')) else 'passed')
    try:
        report['artifacts'] = finalize_artifacts(
            args.output, report['status'] == 'passed', args.retain)
    except Exception as error:
        report['status'] = 'failed'
        report['artifact_error'] = str(error)
    write_json(args.output / 'summary.json', report)
    print(json.dumps(report, indent=2, allow_nan=False))
    return {'passed': 0, 'failed': 1, 'incomplete': 2,
            'interrupted': 130}[report['status']]


if __name__ == '__main__':
    raise SystemExit(main())
