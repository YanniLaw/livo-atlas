#!/usr/bin/env python3
"""Replay a rosbag2 through DDS and record a Humble frontend acceptance summary."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time


def exited_cleanly(returncode, log):
    return returncode == 0 and not any(text in log for text in (
        'terminate called', 'Segmentation fault', 'AddressSanitizer', '[FATAL]'))


def count_mismatches(expected, observed):
    return {topic: dict(expected=expected.get(topic, 0), observed=observed.get(topic, 0))
            for topic in expected.keys() | observed.keys()
            if expected.get(topic, 0) != observed.get(topic, 0)}


def output_issues(lio, vio, counts, path_size, odometry):
    expected = {'/aft_mapped_to_init': lio, '/rgb_img': vio,
                '/cloud_registered': lio + vio}
    issues = count_mismatches(expected, counts)
    if path_size != lio:
        issues['path_size'] = dict(expected=lio, observed=path_size)
    if len(odometry) != lio:
        issues['odometry_rows'] = dict(expected=lio, observed=len(odometry))
    for pose in odometry:
        if len(pose) != 7 or not all(math.isfinite(value) for value in pose):
            issues['odometry_values'] = 'non-finite or invalid pose'
            break
        norm = math.sqrt(sum(value * value for value in pose[3:7]))
        if not math.isclose(norm, 1.0, abs_tol=1e-5):
            issues['odometry_values'] = 'quaternion is not normalized'
            break
    return issues


def file_digest(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            digest.update(block)
    return digest.hexdigest()


def load_config(path):
    import yaml
    result = {}

    def flatten(mapping, prefix=''):
        for name, value in mapping.items():
            key = prefix + '.' + name if prefix else name
            if isinstance(value, dict):
                flatten(value, key)
            else:
                if key.startswith('extrin_calib.') and isinstance(value, list):
                    value = [float(number) for number in value]
                result[key] = value
    flatten(yaml.safe_load(Path(path).read_text()))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    package = Path(__file__).resolve().parents[1]
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=package / 'config/avia.yaml')
    parser.add_argument('--camera', type=Path, default=package / 'config/camera_pinhole.yaml')
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--rate', type=float, default=.5)
    parser.add_argument('--domain', type=int, default=74)
    parser.add_argument('--save-products', action='store_true')
    parser.add_argument('--save-keyframes', action='store_true')
    parser.add_argument('--save-submaps', action='store_true')
    parser.add_argument('--default-threads', action='store_true')
    args = parser.parse_args()
    if args.save_submaps:
        args.save_keyframes = True
    if not math.isfinite(args.duration) or not math.isfinite(args.rate) or args.duration <= 0 or args.rate <= 0:
        raise ValueError('Duration and playback rate must be finite and positive')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')

    import numpy as np
    import psutil
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.utilities import get_rmw_implementation_identifier
    import rosbag2_py
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import Image, PointCloud2
    from rosidl_runtime_py.utilities import get_message
    import yaml
    from output_digest import cloud_digest, image_digest

    def reader():
        result = rosbag2_py.SequentialReader()
        result.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        return result

    before = reader()
    types = {item.name: item.type for item in before.get_all_topics_and_types()}
    expected = Counter()
    expected_hash = defaultdict(hashlib.sha256)
    start = None
    while before.has_next():
        topic, data, timestamp = before.read_next()
        if start is None:
            start = timestamp
        if timestamp >= start + round(args.duration * 1e9):
            break
        expected[topic] += 1
        expected_hash[topic].update(data)
    del before
    if start is None or not expected:
        raise ValueError('Bag has no selected input')
    parameters = load_config(args.config)
    parameters.update(load_config(args.camera))
    parameters.update({'use_sim_time': True, 'qos.reliability': 'reliable',
                       'evo.pose_output_en': True, 'evo.seq_name': 'p4_humble',
                       'output.directory': str(output / 'node-output')})
    if args.save_keyframes:
        parameters['keyframes.enabled'] = True
    if args.save_submaps:
        parameters['submaps.enabled'] = True
    if args.save_products:
        parameters.update({'pcd_save.pcd_save_en': True, 'pcd_save.colmap_output_en': True,
                           'pcd_save.interval': -1, 'image_save.img_save_en': True})
    config = output / 'parameters.yaml'
    config.write_text(yaml.safe_dump({'laserMapping': {'ros__parameters': parameters}}))
    child_env = os.environ.copy()
    if args.default_threads:
        child_env.pop('OMP_THREAD_LIMIT', None)
    else:
        child_env['OMP_THREAD_LIMIT'] = '1'
    child_env.update(ROS_HOME=str(output / 'ros-home'), ROS_LOG_DIR=str(output / 'ros-logs'))
    rclpy.init(args=[])
    observer = rclpy.create_node('livo_baseline_observer')
    received, outputs = Counter(), Counter()
    observed_hash = defaultdict(hashlib.sha256)
    publishers, subscriptions = {}, []
    semantic = {'/cloud_registered': [], '/rgb_img': []}
    odometry, cloud_sizes = [], []
    path_size = 0
    peak_rss = 0

    def input_callback(topic):
        def receive(data):
            received[topic] += 1
            observed_hash[topic].update(data)
        return receive

    for topic in expected:
        message = get_message(types[topic])
        depth = 20 if types[topic] == 'sensor_msgs/msg/Image' else 2000
        qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE)
        publishers[topic] = observer.create_publisher(message, topic, qos)
        subscriptions.append(observer.create_subscription(message, topic, input_callback(topic), qos, raw=True))
    clock = observer.create_publisher(Clock, '/clock', 10)

    def receive_odom(message):
        outputs['/aft_mapped_to_init'] += 1
        pose = message.pose.pose
        odometry.append([pose.position.x, pose.position.y, pose.position.z,
                         pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def receive_cloud(message):
        outputs['/cloud_registered'] += 1
        cloud_sizes.append(message.width * message.height)
        semantic['/cloud_registered'].append(cloud_digest(message))

    def receive_image(message):
        outputs['/rgb_img'] += 1
        semantic['/rgb_img'].append(image_digest(message))

    def receive_path(message):
        nonlocal path_size
        path_size = len(message.poses)

    subscriptions += [observer.create_subscription(Odometry, '/aft_mapped_to_init', receive_odom, 2000),
                      observer.create_subscription(PointCloud2, '/cloud_registered', receive_cloud, 100),
                      observer.create_subscription(Image, '/rgb_img', receive_image, 20),
                      observer.create_subscription(RosPath, '/path', receive_path, 10)]
    log_path = output / 'node.log'
    process = None
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(['nice', '-n', '10', str(args.executable), '--ros-args',
                                        '--params-file', str(config)], env=child_env,
                                       stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=.02)
                if process.poll() is not None:
                    raise RuntimeError('Node failed during startup: ' + log_path.read_text())
                if all(pub.get_subscription_count() >= 2 for pub in publishers.values()):
                    break
            else:
                raise TimeoutError('DDS inputs did not discover both node and observer')
            # Allow output subscriptions and /clock discovery to settle.
            settle = time.monotonic() + 1
            while time.monotonic() < settle:
                message = Clock()
                message.clock.sec, message.clock.nanosec = divmod(start, 1000000000)
                clock.publish(message)
                rclpy.spin_once(observer, timeout_sec=.01)
            stream = reader()
            began = time.monotonic()
            while stream.has_next():
                topic, data, timestamp = stream.read_next()
                if timestamp >= start + round(args.duration * 1e9):
                    break
                due = began + (timestamp - start) * 1e-9 / args.rate
                while time.monotonic() < due:
                    rclpy.spin_once(observer, timeout_sec=min(.01, max(0, due - time.monotonic())))
                    if process.poll() is not None:
                        raise RuntimeError('Node exited during playback')
                message = Clock()
                message.clock.sec, message.clock.nanosec = divmod(timestamp, 1000000000)
                clock.publish(message)
                publishers[topic].publish(data)  # Humble native serialized publish; no object conversion.
                rclpy.spin_once(observer, timeout_sec=0)
                peak_rss = max(peak_rss, psutil.Process(process.pid).memory_info().rss)
            del stream
            settle = time.monotonic() + 3
            while time.monotonic() < settle:
                rclpy.spin_once(observer, timeout_sec=.02)
                if process.poll() is not None:
                    raise RuntimeError('Node exited while draining inputs')
            process.send_signal(signal.SIGINT)
            process.wait(timeout=30)
            # Drain the final already-published output samples.
            settle = time.monotonic() + .2
            while time.monotonic() < settle:
                rclpy.spin_once(observer, timeout_sec=.01)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        observer.destroy_node()
        rclpy.shutdown()
    node_log = log_path.read_text()
    counter = re.search(r'livo_atlas inputs: lidar=(\d+) imu=(\d+) image=(\d+)', node_log)
    node_counts = dict(zip((parameters['common.lid_topic'], parameters['common.imu_topic'], parameters['common.img_topic']),
                          map(int, counter.groups()))) if counter else {}
    mismatches = count_mismatches(dict(expected), dict(received))
    node_mismatches = count_mismatches(dict(expected), node_counts)
    payload_exact = all(expected_hash[topic].digest() == observed_hash[topic].digest() for topic in expected)
    node_output = output / 'node-output'
    for name in ('mat_pre.txt', 'mat_out.txt', 'imu.txt'):
        shutil.copy2(node_output / name, output / name)
    shutil.copy2(node_output / 'result/p4_humble.txt', output / 'trajectory.tum')
    if args.save_keyframes:
        shutil.move(str(node_output / 'keyframes'), str(output / 'keyframes'))
    if args.save_products:
        for name in ('pcd', 'image', 'Colmap'):
            shutil.copytree(node_output / name, output / 'products' / name)
    arrays = {name: np.loadtxt(output / name, ndmin=2) for name in ('trajectory.tum', 'mat_pre.txt', 'mat_out.txt', 'imu.txt')}
    finite = all(np.isfinite(value).all() for value in arrays.values())
    poses = arrays['trajectory.tum']
    trajectory_valid = (poses.shape[1] == 8 and len(poses) > 0 and
                        bool((np.diff(poses[:, 0]) > 0).all()) and
                        bool(np.allclose(np.linalg.norm(poses[:, 4:8], axis=1), 1, atol=1e-5)))
    clean = exited_cleanly(process.returncode, node_log)
    lio = node_log.count('LIO Mapping Time')
    vio = node_log.count('VIO Time')
    publication_issues = output_issues(lio, vio, dict(outputs), path_size, odometry)
    (output / 'semantic-output.json').write_text(json.dumps(semantic, indent=2))
    np.savetxt(output / 'odometry.txt', np.asarray(odometry))
    summary = dict(passed=clean and not mismatches and not node_mismatches and payload_exact and finite and trajectory_valid and not publication_issues,
                   clean_shutdown=clean, returncode=process.returncode, duration_requested_s=args.duration,
                   keyframes_enabled=args.save_keyframes,
                   submaps_enabled=args.save_submaps,
                   playback_rate=args.rate, domain_id=args.domain, rmw=get_rmw_implementation_identifier(),
                   input_reliability='reliable', expected_input_counts=dict(expected), observed_counts=dict(received),
                   node_input_counts=node_counts, input_count_mismatches=mismatches,
                   node_input_count_mismatches=node_mismatches, input_serialized_payloads_exact=payload_exact,
                   outputs=dict(outputs), trajectory_rows=len(poses), path_size=path_size,
                   lio_cycles=lio, vio_cycles=vio, output_issues=publication_issues,
                   arrays={name: dict(shape=list(value.shape), finite=bool(np.isfinite(value).all())) for name, value in arrays.items()},
                   peak_sampled_rss_mib=peak_rss / 1048576, default_threads=args.default_threads,
                   executable=str(args.executable), executable_sha256=file_digest(args.executable),
                   bag_metadata_sha256=file_digest(args.bag / 'metadata.yaml'),
                   cloud_sizes=cloud_sizes, odometry_rows=len(odometry),
                   files={str(path.relative_to(output)): file_digest(path) for path in output.rglob('*')
                          if path.is_file() and path.suffix in ('.txt', '.pcd', '.png')})
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({key: summary[key] for key in ('passed', 'trajectory_rows', 'lio_cycles', 'vio_cycles',
                     'node_input_count_mismatches', 'input_serialized_payloads_exact', 'peak_sampled_rss_mib')}, indent=2))
    if not summary['passed']:
        raise RuntimeError('Humble playback requires investigation; see summary.json')


if __name__ == '__main__':
    main()
