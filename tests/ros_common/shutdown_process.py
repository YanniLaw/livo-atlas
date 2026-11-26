"""Check terminal reception from the actual node with a paused ROS clock."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import xmlrpc.client

REPO = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(REPO), str(REPO / 'tools/3dgs_probe')]
from online_protocol import read_message, write_message
from tools.run_ros2_baseline import load_config


class EmptyPeer:
    def __init__(self, path, delay):
        self.delay = delay
        self.closed = threading.Event()
        self.error = None
        self.events = []
        self.client = None
        self.listener = socket.socket(socket.AF_UNIX)
        self.listener.bind(str(path))
        self.listener.listen(1)
        self.listener.settimeout(.1)
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        try:
            while not self.closed.is_set():
                try:
                    self.client, _ = self.listener.accept()
                    break
                except socket.timeout:
                    pass
            if self.client is None:
                return
            while not self.closed.is_set():
                header, _ = read_message(self.client, 20)
                self.events.append(header['type'])
                response = {key: header[key] for key in
                            ('schema', 'session_id', 'job_id', 'generation',
                             'pose_version', 'submap_id', 'operation',
                             'map_sequence') if key in header}
                if header['type'] == 'hello':
                    Path(header['state_directory']).mkdir(exist_ok=True)
                    response.update(type='hello_ok', mode='persistent_submaps',
                                    state_schema=1, capabilities=['joint_render_v1'],
                                    python='controlled', torch='controlled',
                                    gsplat='controlled', cuda='controlled',
                                    device='controlled', gpu_free_bytes='5000000000',
                                    gpu_total_bytes='8000000000')
                elif header['type'] == 'map_committed':
                    response.update(type='map_committed_ok', asset_ids=header['asset_ids'])
                elif header['type'] == 'stop':
                    self.closed.wait(self.delay)
                    response.update(type='stopped', incomplete=header.get('incomplete', 0))
                else:
                    raise ValueError('Unexpected empty-session request: ' + header['type'])
                write_message(self.client, response, b'', 2)
                if header['type'] == 'stop':
                    return
        except (OSError, TimeoutError) as error:
            # A deadline case deliberately closes the connection before reply.
            self.error = str(error)
        except Exception as error:
            self.error = repr(error)

    def close(self):
        self.closed.set()
        if self.client is not None:
            try:
                self.client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.thread.join(timeout=2)
        if self.client is not None:
            self.client.close()
        self.listener.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros-version', type=int, choices=(1, 2), required=True)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--signal', choices=('SIGINT', 'SIGTERM'), default='SIGINT')
    parser.add_argument('--stop-delay', type=float, default=0)
    parser.add_argument('--expect-timeout', action='store_true')
    parser.add_argument('--repeat-signal', action='store_true')
    parser.add_argument('--shutdown-hook', type=Path)
    args = parser.parse_args()
    if args.output:
        output = args.output.resolve()
        output.mkdir(parents=True, exist_ok=False)
    else:
        output = Path(tempfile.mkdtemp(prefix='livo-shutdown-test-'))
    samples = []
    master = process = peer = shutdown_ros = None
    logs = []
    params = {}
    for name in ('avia.yaml', 'backend.yaml', 'gaussian.yaml', 'camera_pinhole.yaml'):
        params.update(load_config(REPO / 'config' / name))
    params.update({'use_sim_time': True, 'output.directory': str(output / 'node'),
                   'keyframes.enabled': True, 'submaps.enabled': True,
                   'submaps.mode': 'online', 'gaussian.enabled': True,
                   'gaussian.persist_submaps': True, 'gaussian.joint_render_enabled': True,
                   'gaussian.publish_preview': True, 'gaussian.finalize_on_shutdown': True,
                   'gaussian.finalize_timeout_ms': 100 if args.expect_timeout else 3000,
                   'gaussian.shutdown_timeout_ms': 100 if args.expect_timeout else 3000,
                   'gaussian.output_directory': str(output / 'gaussian')})
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1', GLOG_logtostderr='1',
               ROS_HOME=str(output / 'ros-home'), ROS_LOG_DIR=str(output / 'ros-logs'))
    entered, released = output / 'shutdown-entered', output / 'shutdown-released'
    if args.shutdown_hook:
        env.update(LD_PRELOAD=str(args.shutdown_hook.resolve()),
                   LIVO_SHUTDOWN_ENTERED_FILE=str(entered),
                   LIVO_SHUTDOWN_RELEASE_FILE=str(released))
    try:
        with tempfile.TemporaryDirectory(prefix='livo-stop-') as ipc:
            path = Path(ipc) / 'worker.sock'
            params['gaussian.socket_path'] = str(path)
            peer = EmptyPeer(path, args.stop_delay)

            def receive(message):
                samples.append(dict(status=int(message.status), reason=message.reason,
                                    revision=int(message.revision), width=message.image.width,
                                    bytes=len(message.image.data), received=time.monotonic()))

            if args.ros_version == 1:
                import yaml
                with socket.socket() as port_socket:
                    port_socket.bind(('127.0.0.1', 0))
                    port = port_socket.getsockname()[1]
                env.update(ROS_MASTER_URI='http://127.0.0.1:' + str(port), ROS_IP='127.0.0.1')
                os.environ.update(env)
                log = (output / 'master.log').open('w')
                logs.append(log)
                master = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                          stdout=log, stderr=subprocess.STDOUT)
                deadline = time.monotonic() + 10
                while True:
                    try:
                        xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/shutdown_test')
                        break
                    except OSError:
                        if master.poll() is not None or time.monotonic() > deadline:
                            raise RuntimeError('Private ROS master did not start')
                        time.sleep(.02)
                import rospy
                from livo_atlas.msg import GaussianRenderFrame
                rospy.init_node('shutdown_observer', disable_signals=True)
                shutdown_ros = lambda: rospy.signal_shutdown('test finished')
                for key, value in params.items():
                    camera = key.startswith('cam_') or key == 'scale'
                    name = ('/laserMapping/' if camera else '/') + key.replace('.', '/')
                    rospy.set_param(name, value)
                reader = rospy.Subscriber('/gaussian_render', GaussianRenderFrame,
                                          receive, queue_size=10)
                poll = lambda: time.sleep(.005)
                command = [str(args.executable.resolve())]
            else:
                import yaml
                import rclpy
                from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
                from livo_atlas.msg import GaussianRenderFrame
                os.environ.update(env)
                rclpy.init()
                observer = rclpy.create_node('shutdown_observer')
                def shutdown_ros():
                    observer.destroy_node()
                    rclpy.shutdown()
                qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
                reader = observer.create_subscription(GaussianRenderFrame,
                                                       '/gaussian_render', receive, qos)
                poll = lambda: rclpy.spin_once(observer, timeout_sec=.005)
                config = output / 'parameters.yaml'
                config.write_text(yaml.safe_dump({'laserMapping': {'ros__parameters': params}}))
                command = [str(args.executable.resolve()), '--ros-args', '--params-file', str(config)]
            log = (output / 'node.log').open('w')
            logs.append(log)
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 15
            while not samples:
                poll()
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('No initial preview from the actual node')
            began = time.monotonic()
            process.send_signal(getattr(signal, args.signal))
            repeated = False
            late = False
            while process.poll() is None and time.monotonic() - began < 8:
                poll()
                if args.shutdown_hook and entered.exists() and not late:
                    process.send_signal(signal.SIGTERM)
                    late = True
                    # Keep shutdown gated until the second signal is handled.
                    until = time.monotonic() + .05
                    while process.poll() is None and time.monotonic() < until:
                        poll()
                    released.touch()
                if args.repeat_signal and not repeated and time.monotonic() - began > .05:
                    process.send_signal(getattr(signal, args.signal))
                    repeated = True
            deadline = time.monotonic() + .2
            while time.monotonic() < deadline:
                poll()
            stopped = [sample for sample in samples if sample['status'] == 6]
            finalization = json.loads((output / 'gaussian/finalization.json').read_text())
            report = dict(directory=str(output), returncode=process.poll(),
                          elapsed_seconds=time.monotonic() - began,
                          stopped=stopped, samples=samples, repeated=repeated,
                          late_signal_sent=late, peer_error=peer.error,
                          peer_events=list(peer.events),
                          finalization=finalization, signal=args.signal)
            (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
            assert process.poll() == (1 if args.expect_timeout else 0), report
            if args.shutdown_hook:
                assert late, report
            assert len(stopped) == 1, 'Expected one received STOPPED: ' + str(report)
            assert stopped[0]['bytes'] == stopped[0]['width'] == 0, report
            assert samples[-1] == stopped[0], report
            assert 'stop' in peer.events, report
            if args.expect_timeout:
                assert finalization['status'] != 'succeeded', report
                assert not peer.error or not peer.error.startswith('ValueError'), report
            else:
                assert finalization['status'] == 'succeeded' and peer.error is None, report
            assert report['elapsed_seconds'] < 5, report
            print(json.dumps(report), flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        if peer is not None:
            peer.close()
        if shutdown_ros is not None:
            shutdown_ros()
        if master is not None:
            master.terminate()
            try:
                master.wait(timeout=5)
            except subprocess.TimeoutExpired:
                master.kill()
                master.wait()
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
