#!/usr/bin/python3
"""Validate a built ROS1 snapshot against a bag in a private local ROS graph.

Pass a separate catkin source copy to pin the implementation. Current nodes write
to an explicit runtime output directory; --legacy-output supports older nodes
whose output directory is compiled into the executable.
"""
import argparse
import collections
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import threading
import time
import uuid
import xmlrpc.client


def exited_cleanly(returncode, log, gdb_enabled):
    if returncode != 0:
        return False
    if not gdb_enabled:
        return True
    if re.search(r'\breceived signal SIG[A-Z0-9]+\b', log):
        return False
    return re.search(r'\[Inferior .* exited normally\]', log) is not None


def make_trajectory_path(package, sequence_name):
    path = package / ('Log/result/' + sequence_name + '.txt')
    if path.exists():
        raise RuntimeError('Trajectory already exists; cannot validate stale output: ' + str(path))
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True, type=Path,
                        help='Isolated catkin workspace containing src/livo-atlas and devel')
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--rate', type=float, default=0.5)
    parser.add_argument('--port', type=int, default=11341)
    parser.add_argument('--gdb', action='store_true', help='Capture a stack trace on shutdown faults')
    parser.add_argument('--semantic-output', action='store_true',
                        help='Hash declared cloud fields and image pixels, excluding padding/clocks')
    parser.add_argument('--save-products', action='store_true',
                        help='Enable and capture world PCD, images and COLMAP output')
    parser.add_argument('--legacy-output', action='store_true',
                        help='Read older node outputs from its source package Log directory')
    args = parser.parse_args()
    source_package = Path(__file__).resolve().parents[1]
    workspace = args.workspace.resolve()
    package = workspace / 'src/livo-atlas'
    binary = workspace / 'devel/lib/livo_atlas/fastlivo_mapping'
    bag = args.bag.resolve()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError('Output already exists; use a new directory')
    if not math.isfinite(args.rate) or not math.isfinite(args.duration) or args.rate <= 0 or args.duration <= 0:
        raise ValueError('Duration and playback rate must be finite and positive')
    if not bag.is_file():
        raise FileNotFoundError(bag)
    if not binary.is_file():
        raise FileNotFoundError(binary)
    if package.resolve() == source_package:
        raise ValueError('Use a source copy for validation; the node writes into its source tree')

    # Capture the actual snapshot inputs, even when the development checkout has
    # uncommitted edits or changes again while the replay is running.
    source_files = [package / name for name in ['CMakeLists.txt', 'package.xml']]
    for directory in ['src', 'include', 'config']:
        source_files.extend(p for p in (package / directory).rglob('*') if p.is_file())
    source = {'snapshot_workspace': str(workspace),
              'file_sha256': {str(p.relative_to(package)): hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in sorted(source_files)}}
    output.mkdir(parents=True)
    env = os.environ.copy()
    env.update({
        'ROS_MASTER_URI': 'http://127.0.0.1:%d' % args.port,
        'ROS_IP': '127.0.0.1',
        'ROS_LOG_DIR': str(output / 'ros_logs'),
        'ROS_HOME': str(output / 'ros_home'),
        'ROS_PACKAGE_PATH': str(workspace / 'src') + ':/opt/ros/noetic/share',
        'CMAKE_PREFIX_PATH': str(workspace / 'devel') + ':/opt/ros/noetic',
        'LD_LIBRARY_PATH': str(workspace / 'devel/lib') + ':' + env.get('LD_LIBRARY_PATH', ''),
    })
    env.pop('ROS_HOSTNAME', None)
    if args.gdb:
        # Keep GDB's inferior-exit messages deterministic for the status check.
        env['LC_ALL'] = 'C'
    os.environ.update(env)
    os.environ.pop('ROS_HOSTNAME', None)
    socket.setdefaulttimeout(2.0)
    processes = []
    log_handles = []
    counts = collections.Counter()
    semantic = {'/cloud_registered': [], '/rgb_img': []}
    odometry = []
    cloud_sizes = []
    resources = []
    lock = threading.Lock()
    seq_name = 'baseline_' + uuid.uuid4().hex
    started = time.monotonic()
    commands = []
    rospy = None

    def launch(command, name):
        handle = (output / (name + '.log')).open('w')
        log_handles.append(handle)
        commands.append(command)
        proc = subprocess.Popen(command, env=env, stdout=handle,
                                stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, proc))
        return proc

    def run(command):
        commands.append(command)
        result = subprocess.run(command, env=env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, timeout=20)
        if result.returncode:
            raise RuntimeError('%s\n%s' % (command, result.stdout))

    def stop(proc):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)

    try:
        node_output = package / 'Log' if args.legacy_output else output / 'node_output'
        trajectory = (make_trajectory_path(package, seq_name) if args.legacy_output
                      else node_output / 'result' / (seq_name + '.txt'))
        # Do not attach to or interrupt an existing master on this port.
        probe = socket.socket()
        try:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(('127.0.0.1', args.port))
        finally:
            probe.close()
        master_proc = launch(['roscore', '-p', str(args.port)], 'roscore')
        master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        deadline = time.monotonic() + 20
        while True:
            if master_proc.poll() is not None:
                raise RuntimeError('roscore exited; see roscore.log')
            try:
                if master.getPid('/baseline_runner')[0] == 1:
                    break
            except (OSError, xmlrpc.client.Error):
                pass
            if time.monotonic() > deadline:
                raise TimeoutError('ROS master did not start')
            time.sleep(0.1)

        run(['rosparam', 'load', str(package / 'config/avia.yaml')])
        run(['rosparam', 'load', str(package / 'config/camera_pinhole.yaml'), '/laserMapping'])
        run(['rosparam', 'set', '/use_sim_time', 'true'])
        run(['rosparam', 'set', '/evo/pose_output_en', 'true'])
        run(['rosparam', 'set', '/evo/seq_name', seq_name])
        if not args.legacy_output:
            run(['rosparam', 'set', '/output/directory', str(node_output)])
        if args.save_products:
            for key, value in [('pcd_save/pcd_save_en', 'true'),
                               ('pcd_save/colmap_output_en', 'true'),
                               ('pcd_save/interval', '-1'),
                               ('image_save/img_save_en', 'true')]:
                run(['rosparam', 'set', '/' + key, value])
        run(['rosparam', 'dump', str(output / 'parameters.yaml')])

        import psutil
        import rosbag
        import rospy as ros_module
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, PointCloud2
        try:
            from .output_digest import cloud_digest, image_digest
        except ImportError:
            from output_digest import cloud_digest, image_digest
        rospy = ros_module
        rospy.init_node('baseline_observer', disable_signals=True)

        def count(topic):
            def receive(_message):
                with lock:
                    counts[topic] += 1
            return receive

        def receive_odom(msg):
            pose = msg.pose.pose
            with lock:
                counts['/aft_mapped_to_init'] += 1
                odometry.append([msg.header.stamp.to_sec(), pose.position.x,
                                 pose.position.y, pose.position.z, pose.orientation.x,
                                 pose.orientation.y, pose.orientation.z, pose.orientation.w])

        def receive_cloud(msg):
            with lock:
                counts['/cloud_registered'] += 1
                cloud_sizes.append(msg.width * msg.height)
                if args.semantic_output:
                    semantic['/cloud_registered'].append(cloud_digest(msg))

        def receive_image(msg):
            with lock:
                counts['/rgb_img'] += 1
                semantic['/rgb_img'].append(image_digest(msg))

        subscribers = [rospy.Subscriber(topic, rospy.AnyMsg, count(topic),
                       queue_size=2000, buff_size=8 * 1024 * 1024)
                       for topic in ['/livox/lidar', '/livox/imu', '/left_camera/image']]
        subscribers.append(rospy.Subscriber(
            '/rgb_img', Image if args.semantic_output else rospy.AnyMsg,
            receive_image if args.semantic_output else count('/rgb_img'),
            queue_size=2000, buff_size=8 * 1024 * 1024))
        subscribers.extend([
            rospy.Subscriber('/aft_mapped_to_init', Odometry, receive_odom, queue_size=2000),
            rospy.Subscriber('/cloud_registered', PointCloud2, receive_cloud,
                             queue_size=2000, buff_size=8 * 1024 * 1024),
        ])
        node_command = [str(binary), '__name:=laserMapping']
        if args.gdb:
            node_command = ['gdb', '-batch', '-ex', 'set pagination off',
                            '-ex', 'handle SIGINT nostop noprint pass',
                            '-ex', 'run', '-ex', 'thread apply all bt',
                            '--args'] + node_command
        node = launch(node_command, 'mapping')
        deadline = time.monotonic() + 20
        required = {'/livox/lidar', '/livox/imu', '/left_camera/image'}
        while True:
            if node.poll() is not None:
                raise RuntimeError('Mapping node exited before playback; see mapping.log')
            state = master.getSystemState('/baseline_runner')
            ready = {topic for topic, nodes in state[2][1] if '/laserMapping' in nodes}
            if required <= ready:
                break
            if time.monotonic() > deadline:
                raise TimeoutError('Mapping subscriptions did not become ready')
            time.sleep(0.1)

        with rosbag.Bag(str(bag)) as data:
            bag_start, bag_end = data.get_start_time(), data.get_end_time()
            replay_end = min(bag_end, bag_start + args.duration)
            expected = collections.Counter()
            for topic, _, stamp in data.read_messages(raw=True):
                if stamp.to_sec() >= bag_start + args.duration:
                    break
                expected[topic] += 1

        player = launch(['rosbag', 'play', str(bag), '--clock', '--rate', str(args.rate),
                         '--duration', str(args.duration), '--delay', '1', '--queue', '2000'], 'player')
        process_info = psutil.Process(node.pid)
        playback_started = time.monotonic()
        deadline = playback_started + args.duration / args.rate + 90
        last_report = playback_started
        while player.poll() is None:
            if node.poll() is not None:
                raise RuntimeError('Mapping node exited during playback; see mapping.log')
            now = time.monotonic()
            if now > deadline:
                raise TimeoutError('Playback exceeded its timeout')
            cpu = process_info.cpu_times()
            resources.append([now - playback_started, process_info.memory_info().rss,
                              cpu.user + cpu.system])
            if now - last_report >= 15:
                with lock:
                    print('progress', dict(counts), 'rss_mib', round(resources[-1][1] / 2**20, 1), flush=True)
                last_report = now
            time.sleep(0.2)
        playback_wall_s = time.monotonic() - playback_started
        if player.returncode:
            raise RuntimeError('rosbag play exited with %s' % player.returncode)

        # ROS Rate sleeps on simulated time. Advance only /clock after EOF so
        # queued messages can drain; no artificial sensor observations are added.
        clock_pub = rospy.Publisher('/clock', Clock, queue_size=20)
        time.sleep(0.2)
        for i in range(200):
            clock_pub.publish(Clock(rospy.Time.from_sec(replay_end + (i + 1) * 0.01)))
            time.sleep(0.01)
        with lock:
            saved_counts = dict(counts)
        stop(node)
        if args.semantic_output:
            (output / 'semantic-output.json').write_text(json.dumps(semantic, indent=2) + '\n')
        if args.save_products:
            for directory in ['pcd', 'image', 'Colmap']:
                shutil.copytree(node_output / directory, output / 'products' / directory)
        for name in ['mat_pre.txt', 'mat_out.txt', 'imu.txt']:
            shutil.copy2(node_output / name, output / name)
        if not trajectory.exists():
            raise RuntimeError('No sensor-timestamped trajectory was produced')
        shutil.copy2(trajectory, output / 'trajectory.tum')

        import numpy as np
        poses = np.loadtxt(output / 'trajectory.tum', ndmin=2)
        if poses.shape[1] != 8 or poses.shape[0] < 10 or not np.isfinite(poses).all():
            raise RuntimeError('Trajectory is missing, too short, or non-finite')
        if not (np.diff(poses[:, 0]) > 0).all():
            raise RuntimeError('Trajectory sensor timestamps are not strictly increasing')
        if not np.allclose(np.linalg.norm(poses[:, 4:8], axis=1), 1.0, atol=1e-5):
            raise RuntimeError('Trajectory quaternions are not normalized')
        mismatches = {topic: [number, saved_counts.get(topic, 0)]
                      for topic, number in expected.items() if saved_counts.get(topic, 0) != number}
        log = re.sub(r'\x1b\[[0-9;]*m', '', (output / 'mapping.log').read_text(errors='replace'))
        timings = collections.defaultdict(list)
        stage = None
        for line in log.splitlines():
            if 'LIO Mapping Time' in line:
                stage = 'lio'
            elif 'VIO Time' in line:
                stage = 'vio'
            match = re.search(r'\|\s*Current Total Time\s*\|\s*([0-9.eE+-]+)', line)
            if match and stage:
                timings[stage].append(float(match.group(1)))
        errors = [line for line in log.splitlines() if '[ERROR]' in line or 'Assertion' in line]
        summary = {
            'source': source, 'bag_path': str(bag),
            'sequence_name': seq_name,
            'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
            'duration_requested_s': args.duration, 'playback_rate': args.rate,
            'playback_wall_s': playback_wall_s, 'ros_master_port': args.port,
            'use_sim_time': True, 'clock_drain_after_eof_s': 2.0,
            'expected_input_counts': dict(expected), 'observed_counts': saved_counts,
            'input_count_mismatches': mismatches,
            'lio_cycles': log.count('LIO Mapping Time'), 'vio_cycles': log.count('VIO Time'),
            'trajectory_rows': int(poses.shape[0]),
            'trajectory_first_sensor_time_s': float(poses[0, 0]),
            'trajectory_last_sensor_time_s': float(poses[-1, 0]),
            'trajectory_final_pose_xyzw': poses[-1, 1:].tolist(),
            'path_length_m': float(np.linalg.norm(np.diff(poses[:, 1:4], axis=0), axis=1).sum()),
            'peak_sampled_rss_mib': max((row[1] for row in resources), default=0) / 2**20,
            'sampled_resource_process': 'debugger' if args.gdb else 'mapping_node',
            'timing_seconds': {key: {'count': len(values), 'median': float(np.median(values)),
                                   'p95': float(np.percentile(values, 95)), 'max': max(values)}
                               for key, values in timings.items()},
            'errors': errors, 'node_returncode_after_sigint': node.returncode,
            'gdb_enabled': args.gdb,
            'clean_shutdown': exited_cleanly(node.returncode, log, args.gdb),
            'player_returncode': player.returncode,
            'commands': commands,
            'elapsed_wall_s': time.monotonic() - started,
            'limitations': ['No ground truth in this bag; this is a behavior baseline.',
                            'ROS output timestamps use simulated receipt time; use trajectory.tum sensor times for comparison.',
                            'Timing is from legacy printed stage timers; memory is sampled process RSS.',
                            'Observer input counts confirm delivery to the observer, not estimator processing.'],
        }
        np.savetxt(output / 'odometry_published.tum', np.asarray(odometry), fmt='%.9f')
        np.savetxt(output / 'resources.csv', np.asarray(resources), delimiter=',',
                   header='playback_elapsed_s,rss_bytes,cpu_seconds', comments='')
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps({key: summary[key] for key in ['trajectory_rows', 'lio_cycles', 'vio_cycles',
                         'input_count_mismatches', 'peak_sampled_rss_mib', 'errors', 'timing_seconds']}, indent=2), flush=True)
        if mismatches or errors or not summary['clean_shutdown']:
            raise RuntimeError('Baseline needs investigation; see summary.json')
    except Exception as exc:
        (output / 'failure.json').write_text(json.dumps({'error': str(exc), 'commands': commands}, indent=2) + '\n')
        raise
    finally:
        if rospy is not None:
            rospy.signal_shutdown('baseline complete')
        for _, proc in reversed(processes):
            stop(proc)
        for handle in log_handles:
            handle.close()


if __name__ == '__main__':
    main()
