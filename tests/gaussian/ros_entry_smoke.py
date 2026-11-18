#!/usr/bin/env python3
"""Exercise the shared ROS entry and fake worker while simulated time is paused."""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client

import yaml


PACKAGE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE / "tools"))
from run_ros2_baseline import load_config  # noqa: E402


def wait_until(predicate, processes, seconds=30):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        for process in processes:
            if process.poll() is not None:
                raise RuntimeError("Process exited before the expected event")
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError("Timed out waiting for the expected event")


def stop(process, timeout=15):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--system", choices=("1", "2"), required=True)
    parser.add_argument("--node", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--persistent", action="store_true")
    parser.add_argument("--finalize", action="store_true")
    parser.add_argument("--joint-render", action="store_true")
    parser.add_argument("--fail-report", action="store_true",
                        help="Block the report's temporary file to test failed publication")
    parser.add_argument("--expect-unavailable", action="store_true",
                        help="Check that a build without the optional bridge fails before output")
    args = parser.parse_args()
    if args.joint_render and not args.persistent:
        parser.error('--joint-render requires --persistent')
    if args.finalize and not args.persistent:
        parser.error('--finalize requires --persistent')
    if args.fail_report and not args.finalize:
        parser.error('--fail-report requires --finalize')
    output = args.output.absolute()
    output.mkdir()
    socket_dir = output / "socket"
    socket_dir.mkdir(mode=0o700)
    socket_path = socket_dir / "worker.sock"
    previews = output / "previews"
    dataset = output / "node-output"
    environment = os.environ.copy()
    environment.update(OMP_NUM_THREADS="1", OMP_THREAD_LIMIT="1", OPENBLAS_NUM_THREADS="1")
    processes = []
    node = None
    logs = []

    def launch(command, name):
        stream = (output / (name + ".log")).open("w")
        logs.append(stream)
        process = subprocess.Popen(command, env=environment, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    try:
        fake = launch([sys.executable, str(PACKAGE / "tests/gaussian/fake_worker.py"),
                       "--socket", str(socket_path)], "worker")
        wait_until(socket_path.is_socket, [fake])
        common = {"use_sim_time": True, "output.directory": str(dataset),
                  "keyframes.enabled": True, "submaps.enabled": True,
                  "submaps.mode": "online", "loop_closure.enabled": True,
                  "global_output.enabled": True, "global_output.trajectory.enabled": True,
                  "global_output.geometry.enabled": True, "gaussian.enabled": True,
                  "gaussian.socket_path": str(socket_path),
                  "gaussian.output_directory": str(previews)}
        if args.persistent:
            common.update({"gaussian.persist_submaps": True,
                           "gaussian.shutdown_timeout_ms": 30000})
        if args.joint_render:
            common.update({'gaussian.joint_render_enabled': True,
                           'gaussian.joint_render_interval_ms': 1000,
                           'gaussian.joint_render_timeout_ms': 5000})
        if args.finalize:
            common.update({"gaussian.finalize_on_shutdown": True,
                           "gaussian.finalize_timeout_ms": 60000})
        if args.system == "1":
            with socket.socket() as probe:
                probe.bind(("127.0.0.1", 0))
                port = probe.getsockname()[1]
            environment.update(ROS_MASTER_URI="http://127.0.0.1:%d" % port,
                               ROS_IP="127.0.0.1", ROS_HOME=str(output / "ros-home"),
                               ROS_LOG_DIR=str(output / "ros-logs"))
            master = launch(["roscore", "-p", str(port)], "roscore")
            proxy = xmlrpc.client.ServerProxy(environment["ROS_MASTER_URI"])

            def master_ready():
                try:
                    return proxy.getPid("/gaussian_test")[0] == 1
                except (OSError, xmlrpc.client.Error):
                    return False

            wait_until(master_ready, [master])
            subprocess.run(["rosparam", "load", str(PACKAGE / "config/avia.yaml")],
                           env=environment, check=True)
            subprocess.run(["rosparam", "load", str(PACKAGE / "config/camera_pinhole.yaml"),
                            "/laserMapping"], env=environment, check=True)
            for key, value in common.items():
                text = str(value).lower() if isinstance(value, bool) else str(value)
                subprocess.run(["rosparam", "set", "/" + key.replace(".", "/"), text],
                               env=environment, check=True)
            command = [str(args.node), "__name:=laserMapping"]
        else:
            environment["ROS_DOMAIN_ID"] = "93"
            parameters = load_config(PACKAGE / "config/avia.yaml")
            parameters.update(load_config(PACKAGE / "config/camera_pinhole.yaml"))
            parameters.update(common)
            config = output / "parameters.yaml"
            config.write_text(yaml.safe_dump({"laserMapping": {"ros__parameters": parameters}}))
            command = [str(args.node), "--ros-args", "--params-file", str(config)]
        node = launch(command, "node")
        if args.expect_unavailable:
            node.wait(timeout=20)
            text = (output / "node.log").read_text(errors="replace")
            if (node.returncode == 0 or dataset.exists() or previews.exists() or
                    "LIVO_ATLAS_WITH_ONLINE_GAUSSIAN" not in text):
                raise RuntimeError("Disabled build did not reject Gaussian before output")
            print("ROS%s disabled build rejected Gaussian before output" % args.system)
            return
        status = previews / "status.json"
        # No clock publisher or sensor exists: a result proves wall-time polling.
        wait_until(status.is_file, [node, fake])
        snapshot = json.loads(status.read_text())
        if snapshot["generation"] != "0" or snapshot["ready"] != 0:
            raise RuntimeError("Unexpected initial Gaussian status")
        if args.fail_report:
            (previews / 'finalization.tmp.json').mkdir()
        stop(node, 150 if args.finalize else 15)
        fake.wait(timeout=10)
        if node.returncode != (1 if args.fail_report else 0) or fake.returncode != 0:
            raise RuntimeError("ROS node or fake worker did not exit cleanly")
        text = (output / "node.log").read_text(errors="replace")
        for marker in ("livo_atlas loops:", "livo_atlas trajectory:",
                       "livo_atlas geometry map:", "livo_atlas gaussian:"):
            if marker not in text:
                raise RuntimeError("Missing consumer shutdown report: " + marker)
        if args.joint_render:
            if ('capability joint_render_v1' not in (output / 'worker.log').read_text() or
                    'livo_atlas gaussian joint:' not in text):
                raise RuntimeError('Joint render capability was not negotiated/reported')
        if args.persistent:
            state = json.loads((previews / "map.json").read_text())
            if state["entries"] or state["frame_count"] != "0" or not (previews / "states").is_dir():
                raise RuntimeError("Persistent startup map is inconsistent")
            if "incomplete=0" not in text or "map_committed" not in (output / "worker.log").read_text():
                raise RuntimeError("Persistent shutdown did not acknowledge durable map")
            if args.fail_report:
                if ((previews / 'finalization.json').exists() or
                        'status=succeeded' not in text or 'report_persisted=0' not in text):
                    raise RuntimeError('Report failure was not separated from actual convergence')
            elif args.finalize:
                report = json.loads((previews / 'finalization.json').read_text())
                if (report['status'] != 'succeeded' or report['frame_count'] != '0' or
                        report['eligible'] != 0 or report['eligible_set_frozen'] != 1 or
                        report['worker_stopped'] != 1 or report['durable'] != 1 or
                        report['session_id'] != state['session_id'] or
                        report['map_sequence'] != state['map_sequence'] or
                        'livo_atlas gaussian finalization:' not in text or 'report_persisted=1' not in text):
                    raise RuntimeError('Empty finalization did not complete durably')
        elif "reset 0" not in (output / "worker.log").read_text():
            raise RuntimeError("Initial reset was not processed while the clock was paused")
        elif (previews / "states").exists() or (previews / "map.json").exists():
            raise RuntimeError("Legacy mode created persistent state")
        print("ROS%s shared entry: paused-clock %s and four consumers passed" %
              (args.system, "persistent map" if args.persistent else "reset"))
    finally:
        for process in reversed(processes):
            stop(process, 150 if args.finalize and process is node else 15)
        for stream in logs:
            stream.close()


if __name__ == "__main__":
    main()
