#!/usr/bin/python3
"""Exercise real ROS1 startup with valid YAML and malformed configuration.

Use a source copy: the legacy node writes logs into its compile-time package.
This tool starts and stops its own ROS graph and never reuses an output folder.
"""

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import resource
import signal
import socket
import subprocess
import time
import xmlrpc.client

import yaml


def cases():
    result = []

    def add(name, changes=None, remove=(), error=None, **options):
        result.append(dict(name=name, changes=changes or {}, remove=remove,
                           error=error, **options))

    add('avia')
    add('hilti', algorithm='HILTI22.yaml', camera='camera_fisheye_HILTI22.yaml')
    add('ntu', algorithm='NTU_VIRAL.yaml', camera='camera_NTU_VIRAL.yaml')
    add('mars', algorithm='MARS_LVIG.yaml', camera='camera_MARS_LVIG.yaml')
    add('renamed_node', node='renamed_mapping')
    add('optional_defaults', remove=('laserMapping/scale', 'laserMapping/cam_d0',
                                   'laserMapping/cam_d1', 'laserMapping/cam_d2',
                                   'laserMapping/cam_d3'))
    add('fractional_dimensions', {'laserMapping/cam_width': 1281})
    add('integer_intrinsics', {'laserMapping/cam_fx': 1293,
                              'laserMapping/cam_fy': 1293,
                              'laserMapping/scale': 1})
    add('grid_tail_inside_border', {'vio/grid_size': 150})
    add('missing_rotation', remove=('extrin_calib/extrinsic_R',),
        error='extrin_calib/extrinsic_R')
    add('short_rotation', {'extrin_calib/extrinsic_R': [1, 0, 0]},
        error='extrin_calib/extrinsic_R')
    add('short_translation', {'extrin_calib/Pcl': [0, 0]}, error='extrin_calib')
    add('array_element_type', {'extrin_calib/extrinsic_T': [0, 'bad', 0]},
        error='extrin_calib/extrinsic_T')
    add('boolean_array_element', {'extrin_calib/extrinsic_T': [0, True, 0]},
        error='extrin_calib/extrinsic_T')
    add('reflection', {'extrin_calib/extrinsic_R': [-1, 0, 0, 0, 1, 0, 0, 0, 1]},
        error='extrin_calib')
    add('nonfinite_extrinsic', {'extrin_calib/Pcl': [0, float('nan'), 0]},
        error='extrin_calib')
    add('missing_focal', remove=('laserMapping/cam_fx',), error='cam_fx')
    add('wrong_focal_type', {'laserMapping/cam_fx': 'bad'}, error='cam_fx')
    add('boolean_focal', {'laserMapping/cam_fx': True}, error='cam_fx')
    add('zero_focal', {'laserMapping/cam_fx': 0}, error='cam_fx')
    add('nonfinite_focal', {'laserMapping/cam_fx': float('inf')}, error='cam_fx')
    add('nonfinite_distortion', {'laserMapping/cam_d2': float('nan')},
        error='distortion')
    add('wrong_optional_type', {'laserMapping/cam_d0': 'bad'}, error='cam_d0')
    add('wrong_scale_type', {'laserMapping/scale': 'bad'}, error='scale')
    add('unknown_model', {'laserMapping/cam_model': 'PolynomialCamera'},
        error='cam_model')
    add('zero_width', {'laserMapping/cam_width': 0}, error='cam_width')
    add('fractional_width_type', {'laserMapping/cam_width': 1280.5},
        error='cam_width')
    add('empty_scaled_image', {'laserMapping/scale': 0.0001}, error='camera')
    add('dimension_overflow', {'laserMapping/scale': 1e100}, error='camera')
    add('pixel_index_overflow', {'laserMapping/cam_width': 100000,
                                 'laserMapping/cam_height': 100000},
        error='camera')
    add('incompatible_grid', {'laserMapping/cam_height': 16}, error='grid')
    add('zero_grid_rows', {'vio/grid_n_height': 0}, error='grid')
    add('grid_larger_than_image', {'vio/grid_size': 10000}, error='grid')
    add('truncated_grid_rows', {'vio/grid_size': 300}, error='grid')
    add('truncated_grid_columns', {'vio/grid_n_height': 2}, error='grid')
    add('odd_patch', {'vio/patch_size': 7}, error='patch_size')
    add('patch_overflow', {'vio/patch_size': 100000}, error='patch_size')
    add('zero_pyramid', {'vio/patch_pyrimid_level': 0}, error='patch_pyrimid_level')
    add('pyramid_overflow', {'vio/patch_pyrimid_level': 31},
        error='patch_pyrimid_level')
    add('fisheye_colmap', {'pcd_save/colmap_output_en': True},
        algorithm='HILTI22.yaml', camera='camera_fisheye_HILTI22.yaml',
        error='colmap_output_en')
    add('output_default', default_output=True)
    add('empty_output_directory', {'output/directory': ''}, error='output/directory')
    add('wrong_output_directory_type', {'output/directory': 42}, error='output/directory')
    add('invalid_sequence_name', {'evo/seq_name': '../escape'}, error='evo/seq_name')
    add('zero_publish_interval', {'publish/pub_scan_num': 0}, error='publish/pub_scan_num')
    add('zero_image_interval', {'image_save/img_save_en': True,
                               'image_save/interval': 0}, error='image_save/interval')
    add('zero_pcd_interval', {'pcd_save/pcd_save_en': True,
                             'pcd_save/interval': 0}, error='pcd_save/interval')
    add('invalid_pcd_type', {'pcd_save/pcd_save_en': True,
                           'pcd_save/type': 2}, error='pcd_save/type')
    add('invalid_pcd_filter', {'pcd_save/pcd_save_en': True,
                             'pcd_save/filter_size_pcd': float('nan')},
        error='pcd_save/filter_size_pcd')
    add('blocked_output_directory', blocked_output=True, error='blocked_file')
    return result


def set_nested(data, key, value, remove=False):
    parts = key.split('/')
    for part in parts[:-1]:
        data = data.setdefault(part, {})
    if remove:
        data.pop(parts[-1], None)
    else:
        data[parts[-1]] = value


def log_fingerprints(package):
    return {str(path.relative_to(package)):
            (path.stat().st_mtime_ns, hashlib.sha256(path.read_bytes()).hexdigest())
            for path in (package / 'Log').rglob('*') if path.is_file()}


def stop(process):
    if process.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        os.killpg(process.pid, sig)
        try:
            process.wait(timeout=5)
            return
        except subprocess.TimeoutExpired:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--port', type=int, default=11371)
    parser.add_argument('--case', action='append', dest='selected',
                        choices=[case['name'] for case in cases()])
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    package = workspace / 'src/livo-atlas'
    binary = workspace / 'devel/lib/livo_atlas/fastlivo_mapping'
    if package.resolve() == Path(__file__).resolve().parents[1]:
        raise ValueError('Use an isolated source copy for startup verification')
    if not binary.is_file():
        raise FileNotFoundError(binary)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d' % args.port,
               ROS_IP='127.0.0.1', ROS_HOME=str(output / 'ros_home'),
               ROS_LOG_DIR=str(output / 'ros_logs'),
               ROS_PACKAGE_PATH=str(workspace / 'src') + ':/opt/ros/noetic/share',
               CMAKE_PREFIX_PATH=str(workspace / 'devel') + ':/opt/ros/noetic',
               LD_LIBRARY_PATH=str(workspace / 'devel/lib') + ':' +
               env.get('LD_LIBRARY_PATH', ''))
    env.pop('ROS_HOSTNAME', None)
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    socket.setdefaulttimeout(2)
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', args.port))
    results = []
    node = None
    with (output / 'roscore.log').open('w') as master_log:
        master_process = subprocess.Popen(['roscore', '-p', str(args.port)],
                                          env=env, stdout=master_log,
                                          stderr=subprocess.STDOUT,
                                          start_new_session=True)
        try:
            master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
            deadline = time.monotonic() + 20
            while True:
                try:
                    if master.getPid('/calibration_check')[0] == 1:
                        break
                except (OSError, xmlrpc.client.Error):
                    pass
                if master_process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Private ROS master did not start')
                time.sleep(0.1)
            for case in cases():
                if args.selected and case['name'] not in args.selected:
                    continue
                node_name = case.get('node', 'laserMapping')
                parameters = yaml.safe_load((package / 'config' /
                                            case.get('algorithm', 'avia.yaml')).read_text())
                camera = yaml.safe_load((package / 'config' /
                                        case.get('camera', 'camera_pinhole.yaml')).read_text())
                parameters[node_name] = camera
                parameters['use_sim_time'] = False
                parameters.setdefault('evo', {})['pose_output_en'] = False
                parameters.setdefault('pcd_save', {})['pcd_save_en'] = False
                parameters.setdefault('image_save', {})['img_save_en'] = False
                node_output = output / 'node_output' / case['name']
                if case.get('default_output'):
                    parameters.pop('output', None)
                    node_output = Path(env['ROS_HOME']) / 'livo_atlas'
                else:
                    set_nested(parameters, 'output/directory', str(node_output))
                if case.get('blocked_output'):
                    node_output = output / 'blocked_file'
                    node_output.write_text('intentional startup failure fixture\n')
                    set_nested(parameters, 'output/directory', str(node_output))
                for key, value in case['changes'].items():
                    set_nested(parameters, key, copy.deepcopy(value))
                for key in case['remove']:
                    set_nested(parameters, key, None, remove=True)
                response = master.setParam('/calibration_check', '/', parameters)
                if response[0] != 1:
                    raise RuntimeError('Could not set private test parameters: ' + str(response))
                (output / (case['name'] + '.yaml')).write_text(yaml.safe_dump(parameters))
                before = log_fingerprints(package)
                ready = False
                registered = False
                with (output / (case['name'] + '.log')).open('w') as log:
                    node = subprocess.Popen([str(binary), '__name:=' + node_name],
                                            env=env, stdout=log,
                                            stderr=subprocess.STDOUT,
                                            start_new_session=True)
                    deadline = time.monotonic() + 10
                    while node.poll() is None and time.monotonic() < deadline:
                        state = master.getSystemState('/calibration_check')
                        subscriptions = {topic for topic, nodes in state[2][1]
                                         if '/' + node_name in nodes}
                        required = {parameters['common'][key]
                                    for key in ('lid_topic', 'imu_topic', 'img_topic')}
                        registered = bool(required & subscriptions)
                        ready = required <= subscriptions
                        if ready:
                            break
                        time.sleep(0.05)
                    stop(node)
                text = (output / (case['name'] + '.log')).read_text(errors='replace')
                changed_logs = before != log_fingerprints(package)
                expected_error = case['error']
                if expected_error is None:
                    passed = (ready and node.returncode == 0 and not changed_logs and
                              (node_output / 'mat_pre.txt').is_file() and
                              (node_output / 'mat_out.txt').is_file())
                else:
                    passed = (not registered and node.returncode > 0 and
                              expected_error in text and not changed_logs)
                result = dict(name=case['name'], passed=passed, ready=ready,
                              registered_sensor_subscription=registered,
                              returncode=node.returncode, changed_logs=changed_logs,
                              expected_error=expected_error)
                results.append(result)
                print(json.dumps(result), flush=True)
        finally:
            if node is not None:
                stop(node)
            stop(master_process)
    source_files = [package / 'CMakeLists.txt', package / 'package.xml']
    for directory in ('src', 'include', 'config'):
        source_files.extend(path for path in (package / directory).rglob('*') if path.is_file())
    summary = dict(passed=bool(results) and all(item['passed'] for item in results),
                   workspace=str(workspace), cases=results,
                   binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                   source_sha256={str(path.relative_to(package)):
                                  hashlib.sha256(path.read_bytes()).hexdigest()
                                  for path in sorted(source_files)},
                   tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
