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
import sys
import threading
import time
import uuid
import xmlrpc.client


try:
    from tools.gaussian_replay_options import (add_gaussian_arguments,
        gaussian_parameters, nonempty_path, validate_gaussian_arguments)
    from tools.mapping_log import (glog_severity, summarize_frontend_updates,
                                   summarize_stage_timings)
except ImportError:
    from gaussian_replay_options import (add_gaussian_arguments,
        gaussian_parameters, nonempty_path, validate_gaussian_arguments)
    from mapping_log import (glog_severity, summarize_frontend_updates,
                             summarize_stage_timings)


def exited_cleanly(returncode, log, gdb_enabled):
    if returncode != 0 or any(glog_severity(line) == 'F'
                              for line in log.splitlines()):
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


def gaussian_preview_available(directory):
    """A published metadata file and its image form one complete preview."""
    return any(path.with_suffix('.png').is_file()
               for path in Path(directory).glob('preview-*.json'))


def fixed_rate_input_cutoff(launch_before, first_ns, last_ns, rate):
    """Lower bound on final publication, excluding startup and teardown time.

    The native player anchors its fixed-rate timeline after process launch and
    advertising. No immediate/skip/interactive playback is permitted here.
    Subtract the 10 ms clock-drift tolerance rather than crediting startup delay.
    """
    if (not math.isfinite(launch_before) or launch_before < 0 or
            type(first_ns) is not int or type(last_ns) is not int or
            not 0 <= first_ns <= last_ns or not math.isfinite(rate) or rate <= 0):
        raise ValueError('Invalid fixed-rate input timeline')
    return launch_before + (last_ns - first_ns) * 1e-9 / rate - .01


def shutdown_wait_seconds(options):
    return (options.gaussian_finalize_timeout_ms / 1000 + 90
            if options.gaussian_finalize_on_shutdown else 10)


def stop_process(proc, timeout=10, sample=None):
    """Wait for owner cleanup and record escalation separately from exit status."""
    started = time.monotonic()
    forced = False
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        deadline = started + timeout
        while proc.poll() is None and time.monotonic() < deadline:
            if sample is not None:
                sample(time.monotonic() - started)
            try:
                proc.wait(timeout=min(.05, max(0, deadline - time.monotonic())))
            except subprocess.TimeoutExpired:
                pass
        if proc.poll() is None:
            forced = True
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
    return {'elapsed_seconds': time.monotonic() - started, 'forced_termination': forced}


def parse_arguments(arguments=None):
    """Validate feature requirements before creating output or importing ROS."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True, type=Path,
                        help='Isolated catkin workspace containing src/livo-atlas and devel')
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--rate', type=float, default=0.5)
    parser.add_argument('--cpu-affinity', type=int,
                        help='Pin the mapping process to one logical CPU before exec')
    parser.add_argument('--image-queue-size', type=int,
                        help='Override the positive image publication queue depth (node default: 1)')
    parser.add_argument('--stage-timing', action='store_true',
                        help='Enable detailed stage timing in the node')
    parser.add_argument('--port', type=int, default=11341)
    parser.add_argument('--gdb', action='store_true', help='Capture a stack trace on shutdown faults')
    parser.add_argument('--semantic-output', action='store_true',
                        help='Hash declared cloud fields and image pixels, excluding padding/clocks')
    parser.add_argument('--save-products', action='store_true',
                        help='Enable and capture world PCD, images and COLMAP output')
    parser.add_argument('--save-keyframes', action='store_true',
                        help='Capture and retain the P5 keyframe dataset')
    parser.add_argument('--save-submaps', action='store_true',
                        help='Enable keyframes and persistent submap membership')
    parser.add_argument('--legacy-output', action='store_true',
                        help='Read older node outputs from its source package Log directory')
    parser.add_argument('--submap-mode', choices=('keyframe_groups', 'online'),
                        help='Select the submap index format (default: keyframe_groups)')
    parser.add_argument('--geometry-submaps', action='store_true',
                        help='Enable online geometry submaps and their required inputs')
    loop = parser.add_mutually_exclusive_group()
    loop.add_argument('--loop-closure', action='store_true',
                      help='Detect loops and optimize the pose graph')
    loop.add_argument('--loop-detection-only', action='store_true',
                      help='Detect loops and record matches')
    parser.add_argument('--global-output', action='store_true',
                        help='Publish map correction and current global pose using online storage')
    parser.add_argument('--keyframe-trajectory', action='store_true',
                        help='Record versioned keyframe history; enables global output and online storage')
    parser.add_argument('--geometry-map', action='store_true',
                        help='Record versioned geometry maps; enables global output and online storage')
    add_gaussian_arguments(parser)
    args = parser.parse_args(arguments)
    args.pose_graph_enabled = args.loop_closure
    args.loop_closure = args.loop_closure or args.loop_detection_only
    working_dataset = (args.workspace / 'src/livo-atlas/Log/keyframes' if args.legacy_output
                       else args.output / 'node_output/keyframes')
    validate_gaussian_arguments(args, parser, working_dataset)
    if args.geometry_map and args.submap_mode == 'keyframe_groups':
        parser.error('--geometry-map requires online submaps')
    if args.keyframe_trajectory or args.geometry_map:
        args.global_output = True
    if args.global_output and args.submap_mode == 'keyframe_groups':
        parser.error('--global-output requires online submaps')
    if args.loop_closure and args.submap_mode == 'keyframe_groups':
        flag = '--loop-detection-only' if args.loop_detection_only else '--loop-closure'
        parser.error(flag + ' requires online submaps')
    if args.geometry_submaps and args.submap_mode == 'keyframe_groups':
        parser.error('--geometry-submaps requires online submaps')
    if args.submap_mode is None:
        args.submap_mode = 'online' if (args.geometry_submaps or args.loop_closure or args.global_output or args.gaussian_socket) else 'keyframe_groups'
    if args.image_queue_size is not None and args.image_queue_size <= 0:
        parser.error('--image-queue-size must be positive')
    if args.submap_mode == 'online':
        args.save_submaps = True
    if args.save_submaps:
        args.save_keyframes = True
    if args.save_keyframes and args.legacy_output:
        parser.error('--save-keyframes requires the current standalone core')
    return args


def main():
    args = parse_arguments()
    try:
        from .cpu_affinity import with_cpu_affinity
    except ImportError:
        from cpu_affinity import with_cpu_affinity
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
    for directory in ['src', 'include', 'config', 'cmake', 'third_party']:
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
        'GLOG_minloglevel': '0',
        'GLOG_log_prefix': '1',
        'GLOG_logtostdout': '0',
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
    shutdown_resources = []
    lock = threading.Lock()
    seq_name = 'baseline_' + uuid.uuid4().hex
    started = time.monotonic()
    commands = []
    rospy = None
    node = None

    def launch(command, name):
        handle = (output / (name + '.log')).open('w')
        log_handles.append(handle)
        commands.append(command)
        proc = subprocess.Popen(command, env=env, stdout=handle,
                                stdin=subprocess.DEVNULL if name == 'player' else None,
                                stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, proc))
        return proc

    def run(command):
        commands.append(command)
        result = subprocess.run(command, env=env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, timeout=20)
        if result.returncode:
            raise RuntimeError('%s\n%s' % (command, result.stdout))

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

        for name in ('backend.yaml', 'gaussian.yaml'):
            defaults = package / 'config' / name
            if defaults.is_file():
                run(['rosparam', 'load', str(defaults)])
        run(['rosparam', 'load', str(package / 'config/avia.yaml')])
        run(['rosparam', 'load', str(package / 'config/camera_pinhole.yaml'), '/laserMapping'])
        run(['rosparam', 'set', '/use_sim_time', 'true'])
        run(['rosparam', 'set', '/evo/pose_output_en', 'true'])
        run(['rosparam', 'set', '/evo/seq_name', seq_name])
        if not args.legacy_output:
            run(['rosparam', 'set', '/output/directory', str(node_output)])
        if args.save_keyframes:
            run(['rosparam', 'set', '/keyframes/enabled', 'true'])
        if args.save_submaps:
            run(['rosparam', 'set', '/submaps/enabled', 'true'])
            run(['rosparam', 'set', '/submaps/mode', args.submap_mode])
        if args.global_output:
            run(['rosparam', 'set', '/global_output/enabled', 'true'])
        if args.keyframe_trajectory:
            run(['rosparam', 'set', '/global_output/trajectory/enabled', 'true'])
        if args.geometry_map:
            run(['rosparam', 'set', '/global_output/geometry/enabled', 'true'])
        if args.loop_closure:
            run(['rosparam', 'set', '/loop_closure/enabled', 'true'])
            run(['rosparam', 'set', '/pose_graph/enabled',
                 json.dumps(args.pose_graph_enabled)])
        if args.geometry_submaps:
            run(['rosparam', 'set', '/submaps/geometry/enabled', 'true'])
        for name, value in gaussian_parameters(args).items():
            run(['rosparam', 'set', '/' + name.replace('.', '/'),
                 json.dumps(value) if isinstance(value, bool) else str(value)])
        if args.image_queue_size is not None:
            run(['rosparam', 'set', '/publish/image_queue_size', str(args.image_queue_size)])
        if args.stage_timing:
            run(['rosparam', 'set', '/diagnostics/stage_timing', 'true'])
        if args.save_products:
            for key, value in [('pcd_save/pcd_save_en', 'true'),
                               ('pcd_save/colmap_output_en', 'true'),
                               ('pcd_save/interval', '-1'),
                               ('image_save/img_save_en', 'true')]:
                run(['rosparam', 'set', '/' + key, value])
        parameters_path = output / 'parameters.yaml'
        run(['rosparam', 'dump', str(parameters_path)])

        import psutil
        import rosbag
        import rospy as ros_module
        import yaml
        from nav_msgs.msg import Odometry, Path as RosPath
        from geometry_msgs.msg import PoseStamped
        from tf2_msgs.msg import TFMessage
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, PointCloud2

        parameters = yaml.safe_load(parameters_path.read_text())
        keyframes_enabled = parameters.get('keyframes', {}).get('enabled', False)
        submaps = parameters.get('submaps', {})
        loop_enabled = parameters.get('loop_closure', {}).get('enabled', False)
        pose_graph = parameters.get('pose_graph', {})
        # Use the merged YAML and CLI settings for observations and shutdown.
        gaussian = parameters.get('gaussian', {})
        gaussian_directory = gaussian.get('output_directory')
        args.gaussian_output = (Path(gaussian_directory) if gaussian_directory and
                                gaussian.get('enabled', False) else None)
        args.gaussian_finalize_on_shutdown = gaussian.get('finalize_on_shutdown', False)
        args.gaussian_finalize_timeout_ms = gaussian.get('finalize_timeout_ms', 60000)
        args.gaussian_joint_render = gaussian.get('joint_render_enabled', False)
        args.joint_render_policy = gaussian.get('joint_render_policy', 'temporal')

        try:
            from .output_digest import cloud_digest, image_digest
            from .gaussian_preview_report import (gaussian_render_record, gaussian_image_record,
                gaussian_reception_errors)
            from .geometry_map_records import geometry_map_record
            from .trajectory_records import trajectory_record, trajectory_path_record
        except ImportError:
            from output_digest import cloud_digest, image_digest
            from gaussian_preview_report import (gaussian_render_record, gaussian_image_record,
                gaussian_reception_errors)
            from geometry_map_records import geometry_map_record
            from trajectory_records import trajectory_record, trajectory_path_record
        rospy = ros_module
        rospy.init_node('baseline_observer', disable_signals=True)
        image_queue_size = rospy.get_param('/publish/image_queue_size', 1)
        global_observation = dict(
            enabled=rospy.get_param('/global_output/enabled', False),
            publish_tf=rospy.get_param('/global_output/publish_tf', True),
            map_frame=rospy.get_param('/global_output/map_frame', 'map'),
            raw_poses=[], global_poses=[], tf_messages=[])
        trajectory_observation = dict(
            enabled=rospy.get_param('/global_output/trajectory/enabled', False),
            samples=[], path_samples=[])
        geometry_observation = dict(
            enabled=rospy.get_param('/global_output/geometry/enabled', False),
            samples=[], errors=[])

        gaussian_observation = dict(
            enabled=rospy.get_param('/gaussian/publish_preview', False),
            optical_frame=rospy.get_param('/gaussian/preview_frame_id', 'gaussian_camera_optical'),
            map_frame=global_observation['map_frame'], samples=[], image_samples=[], errors=[])

        def receive_gaussian(message, mirror=False):
            try:
                value = (gaussian_image_record(message, lambda stamp: stamp.to_nsec()) if mirror
                         else gaussian_render_record(message, lambda stamp: stamp.to_nsec()))
                with lock:
                    gaussian_observation['image_samples' if mirror else 'samples'].append(value)
            except Exception as error:
                with lock:
                    gaussian_observation['errors'].append(str(error))

        def receive_geometry(message):
            try:
                value = geometry_map_record(message, lambda stamp: stamp.to_nsec())
            except Exception as error:
                with lock:
                    geometry_observation['errors'].append(str(error))
                return
            with lock:
                geometry_observation['samples'].append(value)

        def receive_trajectory(message):
            value = trajectory_record(message, lambda stamp: stamp.to_nsec())
            with lock:
                trajectory_observation['samples'].append(value)

        def receive_trajectory_path(message):
            value = trajectory_path_record(message, lambda stamp: stamp.to_nsec())
            with lock:
                trajectory_observation['path_samples'].append(value)

        def pose_record(message, pose):
            return dict(stamp_ns=message.header.stamp.to_nsec(),
                        frame=message.header.frame_id,
                        translation=[pose.position.x, pose.position.y, pose.position.z],
                        rotation_xyzw=[pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z, pose.orientation.w])

        def receive_global(message):
            with lock:
                global_observation['global_poses'].append(pose_record(message, message.pose))

        def receive_tf(message):
            rows = []
            for transform in message.transforms:
                value = transform.transform
                rows.append(dict(stamp_ns=transform.header.stamp.to_nsec(),
                                 parent=transform.header.frame_id, child=transform.child_frame_id,
                                 translation=[value.translation.x, value.translation.y, value.translation.z],
                                 rotation_xyzw=[value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w]))
            with lock:
                global_observation['tf_messages'].append(rows)

        def count(topic):
            def receive(_message):
                with lock:
                    counts[topic] += 1
            return receive

        def receive_odom(msg):
            pose = msg.pose.pose
            with lock:
                counts['/aft_mapped_to_init'] += 1
                global_observation['raw_poses'].append(pose_record(msg, pose))
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
        subscribers.extend([
            rospy.Subscriber('/global_pose', PoseStamped, receive_global, queue_size=2000),
            rospy.Subscriber('/tf', TFMessage, receive_tf, queue_size=2000),
        ])
        if (trajectory_observation['enabled'] or geometry_observation['enabled'] or
                gaussian_observation['enabled']):
            # Match the generated schema to the explicitly selected workspace.
            sys.path.insert(0, str(workspace / 'devel/lib/python3/dist-packages'))
        if trajectory_observation['enabled']:
            from livo_atlas.msg import KeyframeTrajectory
            subscribers.extend([
                rospy.Subscriber('/keyframe_trajectory', KeyframeTrajectory,
                                 receive_trajectory, queue_size=1, buff_size=16 * 1024 * 1024),
                rospy.Subscriber('/keyframe_path', RosPath, receive_trajectory_path,
                                 queue_size=1, buff_size=32 * 1024 * 1024),
            ])
        if geometry_observation['enabled']:
            from livo_atlas.msg import GeometryMap
            subscribers.append(rospy.Subscriber(
                '/geometry_map', GeometryMap, receive_geometry,
                queue_size=1, buff_size=72 * 1024 * 1024))
        if gaussian_observation['enabled']:
            from livo_atlas.msg import GaussianRenderFrame
            subscribers.extend([
                rospy.Subscriber('/gaussian_render', GaussianRenderFrame, receive_gaussian,
                                 queue_size=1, buff_size=8 * 1024 * 1024),
                rospy.Subscriber('/gaussian_render/image', Image,
                                 lambda msg: receive_gaussian(msg, True),
                                 queue_size=1, buff_size=8 * 1024 * 1024),
            ])
        node_command = [str(binary), '__name:=laserMapping']
        if args.gdb:
            node_command = ['gdb', '-batch', '-ex', 'set pagination off',
                            '-ex', 'handle SIGINT nostop noprint pass',
                            '-ex', 'run', '-ex', 'thread apply all bt',
                            '--args'] + node_command
        node = launch(with_cpu_affinity(node_command, args.cpu_affinity), 'mapping')
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

        mapping_affinity = psutil.Process(node.pid).cpu_affinity()
        with rosbag.Bag(str(bag)) as data:
            bag_start, bag_end = data.get_start_time(), data.get_end_time()
            replay_end = min(bag_end, bag_start + args.duration)
            expected = collections.Counter()
            first_input_ns = last_input_ns = None
            for topic, _, stamp in data.read_messages(raw=True):
                if stamp.to_sec() >= bag_start + args.duration:
                    break
                expected[topic] += 1
                if first_input_ns is None:
                    first_input_ns = stamp.to_nsec()
                last_input_ns = stamp.to_nsec()

        playback_started = time.monotonic()
        playback_wall_started = time.time()
        input_cutoff = fixed_rate_input_cutoff(playback_started, first_input_ns, last_input_ns, args.rate)
        wall_clock_stable = True
        player = launch(['rosbag', 'play', str(bag), '--clock', '--rate', str(args.rate),
                         '--duration', str(args.duration), '--delay', '1', '--queue', '2000'], 'player')
        process_info = psutil.Process(node.pid)
        first_gaussian_preview_seconds = None
        deadline = playback_started + args.duration / args.rate + 90
        last_report = playback_started
        confirmed_running_until = playback_started
        while True:
            sampled_before_poll = time.monotonic()
            if abs((time.time() - playback_wall_started) - (sampled_before_poll - playback_started)) > .01:
                wall_clock_stable = False
            if player.poll() is not None:
                break
            confirmed_running_until = sampled_before_poll
            if node.poll() is not None:
                raise RuntimeError('Mapping node exited during playback; see mapping.log')
            now = time.monotonic()
            if (args.gaussian_output and first_gaussian_preview_seconds is None and
                    gaussian_preview_available(args.gaussian_output)):
                first_gaussian_preview_seconds = now - playback_started
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
        playback_eof_steady_seconds = time.monotonic()
        if abs((time.time() - playback_wall_started) -
               (playback_eof_steady_seconds - playback_started)) > .01:
            wall_clock_stable = False
        playback_wall_s = playback_eof_steady_seconds - playback_started
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
        def sample_shutdown(elapsed):
            try:
                cpu = process_info.cpu_times()
                shutdown_resources.append([elapsed, process_info.memory_info().rss,
                                           cpu.user + cpu.system])
            except psutil.NoSuchProcess:
                pass

        shutdown = stop_process(node, shutdown_wait_seconds(args),
                                sample_shutdown if args.gaussian_finalize_on_shutdown else None)
        with lock:
            if geometry_observation['enabled'] and not geometry_observation['samples']:
                geometry_observation['errors'].append('No geometry map messages received')
            gaussian_observation['errors'] = gaussian_reception_errors(gaussian_observation)
            (output / 'gaussian-publication.json').write_text(
                json.dumps(gaussian_observation, indent=2, allow_nan=False) + '\n')
            (output / 'global-output.json').write_text(json.dumps(global_observation, indent=2) + '\n')
            (output / 'keyframe-trajectory.json').write_text(
                json.dumps(trajectory_observation, indent=2) + '\n')
            (output / 'geometry-map.json').write_text(
                json.dumps(geometry_observation, indent=2) + '\n')
        if args.semantic_output:
            (output / 'semantic-output.json').write_text(json.dumps(semantic, indent=2) + '\n')
        if keyframes_enabled:
            shutil.move(str(node_output / 'keyframes'), str(output / 'keyframes'))
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
        try:
            from .geometry_diagnostics import summarize_geometry_timings
        except ImportError:
            from geometry_diagnostics import summarize_geometry_timings
        geometry_timings = summarize_geometry_timings(log, np)
        updates, timings = summarize_frontend_updates(log, np)
        stage_timings, buffer_peaks = summarize_stage_timings(log, np)
        errors = [line for line in log.splitlines()
                  if '[ERROR]' in line or 'Assertion' in line
                  or glog_severity(line) in ('E', 'F')]
        errors.extend('Geometry map record: ' + error for error in geometry_observation['errors'])
        errors.extend('Gaussian record: ' + error for error in gaussian_observation['errors'])
        summary = {
            'source': source, 'bag_path': str(bag),
            'sequence_name': seq_name,
            'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
            'duration_requested_s': args.duration, 'playback_rate': args.rate,
            'requested_cpu_affinity': args.cpu_affinity,
            'mapping_cpu_affinity': mapping_affinity,
            'image_queue_size': image_queue_size,
            'playback_wall_s': playback_wall_s, 'ros_master_port': args.port,
            'shutdown_wall_s': shutdown['elapsed_seconds'],
            'forced_termination': shutdown['forced_termination'],
            'shutdown_wait_budget_s': shutdown_wait_seconds(args),
            'peak_shutdown_rss_mib': max((row[1] for row in shutdown_resources), default=0) / 2**20,
            'playback_started_steady_seconds': playback_started,
            'playback_eof_steady_seconds': playback_eof_steady_seconds,
            'playback_confirmed_running_until_steady_seconds': confirmed_running_until,
            'playback_input_cutoff_steady_seconds': input_cutoff if wall_clock_stable else None,
            'playback_input_cutoff_source': 'fixed_rate_lower_bound',
            'playback_wall_clock_stable': wall_clock_stable,
            'playback_first_input_timestamp_ns': str(first_input_ns),
            'playback_last_input_timestamp_ns': str(last_input_ns),
            'gaussian_publish_preview': gaussian_observation['enabled'],
            'joint_render_enabled': args.gaussian_joint_render,
            'joint_render_policy': args.joint_render_policy,
            'use_sim_time': True, 'clock_drain_after_eof_s': 2.0,
            'expected_input_counts': dict(expected), 'observed_counts': saved_counts,
            'input_count_mismatches': mismatches,
            'lio_cycles': updates['lio'], 'vio_cycles': updates['vio'],
            'trajectory_rows': int(poses.shape[0]),
            'trajectory_first_sensor_time_s': float(poses[0, 0]),
            'trajectory_last_sensor_time_s': float(poses[-1, 0]),
            'trajectory_final_pose_xyzw': poses[-1, 1:].tolist(),
            'path_length_m': float(np.linalg.norm(np.diff(poses[:, 1:4], axis=0), axis=1).sum()),
            'peak_sampled_rss_mib': max((row[1] for row in resources), default=0) / 2**20,
            'sampled_resource_process': 'debugger' if args.gdb else 'mapping_node',
            'timing_seconds': timings,
            'stage_timing_seconds': stage_timings,
            'sensor_buffer_peaks': buffer_peaks,
            'errors': errors, 'node_returncode_after_sigint': node.returncode,
            'keyframes_enabled': keyframes_enabled,
            'submaps_enabled': submaps.get('enabled', False),
            'submap_mode': submaps.get('mode', 'keyframe_groups'),
            'geometry_enabled': submaps.get('geometry', {}).get('enabled', False),
            'geometry_map_enabled': geometry_observation['enabled'],
            'geometry_map_samples': len(geometry_observation['samples']),
            'loop_closure_enabled': loop_enabled,
            'pose_graph_enabled': pose_graph.get('enabled', loop_enabled),
            'geometry_timing_seconds': geometry_timings,
            'gdb_enabled': args.gdb,
            'clean_shutdown': (not shutdown['forced_termination'] and
                               exited_cleanly(node.returncode, log, args.gdb)),
            'player_returncode': player.returncode,
            'first_gaussian_preview_during_playback_s': first_gaussian_preview_seconds,
            'commands': commands,
            'elapsed_wall_s': time.monotonic() - started,
            'limitations': ['No ground truth in this bag; this is a behavior baseline.',
                            'ROS output timestamps use simulated receipt time; use trajectory.tum sensor times for comparison.',
                            'Frontend timing covers Frontend::Process. Memory is sampled process RSS.',
                            'Observer input counts confirm delivery to the observer, not estimator processing.'],
        }
        np.savetxt(output / 'odometry_published.tum', np.asarray(odometry), fmt='%.9f')
        np.savetxt(output / 'resources.csv', np.asarray(resources), delimiter=',',
                   header='playback_elapsed_s,rss_bytes,cpu_seconds', comments='')
        if args.gaussian_finalize_on_shutdown:
            np.savetxt(output / 'shutdown-resources.csv', np.asarray(shutdown_resources), delimiter=',',
                       header='shutdown_elapsed_s,rss_bytes,cpu_seconds', comments='')
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
            stop_process(proc, shutdown_wait_seconds(args) if proc is node else 10)
        for handle in log_handles:
            handle.close()


if __name__ == '__main__':
    main()
