#!/usr/bin/env python3
"""Run an isolated ROS1 baseline replay and collect its outputs."""

import argparse
import csv
import datetime
import json
import math
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import traceback
import uuid
import xmlrpc.client
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
ROS_SETUP = Path("/opt/ros/noetic/setup.bash")
SOURCE_LOG_NAMES = ("mat_pre.txt", "mat_out.txt", "imu.txt")
OUTPUT_TOPICS = (
    "/aft_mapped_to_init",
    "/cloud_registered",
    "/rgb_img",
    "/path",
)


def write_json_atomic(path, value):
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(str(temporary), str(path))


def sourced_ros_environment(output_dir):
    devel_setup = WORKSPACE_ROOT / "devel" / "setup.bash"
    if not ROS_SETUP.is_file():
        raise RuntimeError("ROS setup not found: {}".format(ROS_SETUP))
    if not devel_setup.is_file():
        raise RuntimeError("devel setup not found: {}".format(devel_setup))

    home_dir = output_dir / "home"
    ros_home = output_dir / "ros_home"
    ros_log_dir = output_dir / "ros_logs"
    tmp_dir = output_dir / "tmp"
    cache_dir = output_dir / "cache"
    for directory in (home_dir, ros_home, ros_log_dir, tmp_dir, cache_dir):
        directory.mkdir(parents=True, exist_ok=True)

    base_env = os.environ.copy()
    base_env.update({
        "ROS_HOME": str(ros_home),
        "ROS_LOG_DIR": str(ros_log_dir),
        "TMPDIR": str(tmp_dir),
        "XDG_CACHE_HOME": str(cache_dir),
        "PYTHONDONTWRITEBYTECODE": "1",
    })
    source_script = 'source "$1" && source "$2" && env -0'
    result = subprocess.run(
        ["bash", "-c", source_script, "bash", str(ROS_SETUP), str(devel_setup)],
        cwd=str(WORKSPACE_ROOT),
        env=base_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError("failed to source ROS/devel setup: {}".format(
            result.stderr.decode("utf-8", errors="replace").strip()))
    env = {}
    for entry in result.stdout.split(b"\0"):
        if entry and b"=" in entry:
            key, value = entry.split(b"=", 1)
            env[key.decode("utf-8", errors="replace")] = value.decode(
                "utf-8", errors="replace")
    for key in ("ROS_HOME", "ROS_LOG_DIR", "TMPDIR", "XDG_CACHE_HOME",
                "PYTHONDONTWRITEBYTECODE"):
        env[key] = base_env[key]
    env.pop("ROS_HOSTNAME", None)
    return env


def find_program(name, env):
    program = shutil.which(name, path=env.get("PATH"))
    if not program:
        raise RuntimeError("ROS command not found after sourcing setup: {}".format(name))
    return program


def reject_busy_port(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        try:
            listener.bind(("127.0.0.1", port))
        except OSError as error:
            raise RuntimeError("ROS master port {} is already in use: {}".format(
                port, error))


def find_running_mappers():
    matches = []
    proc_dir = Path("/proc")
    for entry in proc_dir.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace").strip()
        except OSError:
            continue
        if "fastlivo_mapping" in command:
            matches.append({"pid": int(entry.name), "command": command})
    return matches


class SourceLogGuard:
    """Keep an on-disk backup and restore mapper-owned source log files."""

    def __init__(self, output_dir):
        self.log_dir = PACKAGE_ROOT / "Log"
        self.backup_dir = output_dir / "source_log_backup"
        self.snapshots = {}

    def capture(self):
        self.backup_dir.mkdir(parents=True, exist_ok=True)
        for name in SOURCE_LOG_NAMES:
            path = self.log_dir / name
            if path.exists():
                if not path.is_file() or path.is_symlink():
                    raise RuntimeError("refusing to back up non-regular source log: {}".format(path))
                content = path.read_bytes()
                mode = path.stat().st_mode & 0o777
                shutil.copy2(str(path), str(self.backup_dir / name))
                self.snapshots[name] = {"existed": True, "content": content, "mode": mode}
            else:
                self.snapshots[name] = {"existed": False, "content": None, "mode": None}

    def restore(self):
        for name, snapshot in self.snapshots.items():
            path = self.log_dir / name
            if snapshot["existed"]:
                fd, temporary_name = tempfile.mkstemp(
                    prefix=".run_baseline_restore_", dir=str(self.log_dir))
                try:
                    with os.fdopen(fd, "wb") as stream:
                        stream.write(snapshot["content"])
                    os.chmod(temporary_name, snapshot["mode"])
                    os.replace(temporary_name, str(path))
                finally:
                    if os.path.exists(temporary_name):
                        os.unlink(temporary_name)
            elif path.exists():
                path.unlink()


class ResourceSampler(threading.Thread):
    def __init__(self, pid, csv_path):
        super().__init__(daemon=True)
        self.pid = pid
        self.csv_path = csv_path
        self.stop_event = threading.Event()
        self.started_at = time.monotonic()
        self.peak_rss_bytes = 0
        self.peak_vmhwm_bytes = 0
        self.samples = 0

    def run(self):
        try:
            with self.csv_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(["elapsed_s", "rss_bytes", "vmhwm_bytes"])
                while not self.stop_event.is_set():
                    status_path = Path("/proc") / str(self.pid) / "status"
                    rss = None
                    vmhwm = None
                    try:
                        for line in status_path.read_text(encoding="ascii").splitlines():
                            if line.startswith("VmRSS:"):
                                rss = int(line.split()[1]) * 1024
                            elif line.startswith("VmHWM:"):
                                vmhwm = int(line.split()[1]) * 1024
                    except (OSError, ValueError, IndexError):
                        pass
                    elapsed = time.monotonic() - self.started_at
                    if rss is not None or vmhwm is not None:
                        writer.writerow(["{:.3f}".format(elapsed), rss, vmhwm])
                        stream.flush()
                        self.samples += 1
                        if rss is not None:
                            self.peak_rss_bytes = max(self.peak_rss_bytes, rss)
                        if vmhwm is not None:
                            self.peak_vmhwm_bytes = max(self.peak_vmhwm_bytes, vmhwm)
                    self.stop_event.wait(1.0)
        except OSError:
            pass

    def stop(self):
        self.stop_event.set()
        self.join(timeout=3.0)


def ros_master_proxy(uri):
    return xmlrpc.client.ServerProxy(uri, allow_none=True)


def wait_for_master(uri, process, timeout=20.0):
    deadline = time.monotonic() + timeout
    last_error = "no response"
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("roscore exited before becoming ready (code {})".format(
                process.returncode))
        try:
            code, message, _ = ros_master_proxy(uri).getPid("/baseline_runner")
            if code == 1:
                return
            last_error = message
        except Exception as error:  # XML-RPC can fail until roscore is ready.
            last_error = str(error)
        time.sleep(0.2)
    raise RuntimeError("ROS master did not become ready: {}".format(last_error))


def wait_for_nodes_and_topics(uri, mapper_process, collector_process, timeout=30.0):
    required = set(OUTPUT_TOPICS)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if mapper_process.poll() is not None:
            raise RuntimeError("mapper exited before subscriptions were ready")
        if collector_process.poll() is not None:
            raise RuntimeError("collector exited before subscriptions were ready")
        try:
            code, _, state = ros_master_proxy(uri).getSystemState("/baseline_runner")
            if code == 1:
                publishers, subscribers, _ = state
                pub_nodes = {topic: set(nodes) for topic, nodes in publishers}
                sub_nodes = {topic: set(nodes) for topic, nodes in subscribers}
                connected = all(
                    "/laserMapping" in pub_nodes.get(topic, set())
                    and "/baseline_collector" in sub_nodes.get(topic, set())
                    for topic in required)
                if connected:
                    return
        except Exception:
            pass
        time.sleep(0.25)
    raise RuntimeError("mapper publishers and collector subscriptions did not connect")


def run_ros_command(args, env, log_path, timeout=40.0):
    with log_path.open("ab") as log_stream:
        log_stream.write(("$ " + " ".join(args) + "\n").encode("utf-8"))
        log_stream.flush()
        result = subprocess.run(
            args,
            cwd=str(WORKSPACE_ROOT),
            env=env,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
    if result.returncode != 0:
        raise RuntimeError("command failed with code {}: {}".format(
            result.returncode, " ".join(args)))


def launch_process(command, env, cwd, log_path):
    log_stream = log_path.open("wb")
    try:
        process = subprocess.Popen(
            command,
            cwd=str(cwd),
            env=env,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except Exception:
        log_stream.close()
        raise
    return process, log_stream


def stop_process(process, label, shutdown_log, signal_group=True):
    if process is None:
        return {"signal": None, "escalated": False, "return_code": None}
    sent_signal = None
    escalated = False
    if process.poll() is None:
        try:
            if signal_group:
                os.killpg(process.pid, signal.SIGINT)
            else:
                process.send_signal(signal.SIGINT)
            sent_signal = "SIGINT"
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            escalated = True
            try:
                os.killpg(process.pid, signal.SIGTERM)
                sent_signal = "SIGTERM"
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                    sent_signal = "SIGKILL"
                except ProcessLookupError:
                    pass
                process.wait(timeout=3.0)
    status = {
        "signal": sent_signal,
        "escalated": escalated,
        "return_code": process.poll(),
    }
    shutdown_log[label] = status
    return status


def collector_main(output_dir):
    import rospy
    import rosbag
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import Image, PointCloud2

    output_dir = Path(output_dir)
    lock = threading.Lock()
    stop_writer = threading.Event()
    topic_stats = {
        topic: {"count": 0, "first": None, "last": None}
        for topic in OUTPUT_TOPICS
    }
    path_lengths = {"first": None, "last": None}
    cloud_counts = {"empty": 0, "nonempty": 0}
    latest_nonempty_cloud = {"message": None, "stamp": None}
    odom_stream = (output_dir / "odom.csv").open("w", newline="", encoding="utf-8")
    odom_writer = csv.writer(odom_stream)
    odom_writer.writerow([
        "header_stamp_s", "wall_time_s", "x", "y", "z", "qx", "qy", "qz", "qw"])
    odom_stream.flush()

    def note(topic, message):
        now = time.time()
        header_stamp = message.header.stamp.to_sec()
        record = {"header_stamp_s": header_stamp, "wall_time_s": now}
        with lock:
            stats = topic_stats[topic]
            stats["count"] += 1
            if stats["first"] is None:
                stats["first"] = record
            stats["last"] = record

    def odom_callback(message):
        note("/aft_mapped_to_init", message)
        pose = message.pose.pose
        with lock:
            odom_writer.writerow([
                "{:.9f}".format(message.header.stamp.to_sec()),
                "{:.9f}".format(time.time()),
                "{:.9f}".format(pose.position.x),
                "{:.9f}".format(pose.position.y),
                "{:.9f}".format(pose.position.z),
                "{:.9f}".format(pose.orientation.x),
                "{:.9f}".format(pose.orientation.y),
                "{:.9f}".format(pose.orientation.z),
                "{:.9f}".format(pose.orientation.w),
            ])
            odom_stream.flush()

    def cloud_callback(message):
        note("/cloud_registered", message)
        is_nonempty = message.width > 0 and message.height > 0 and bool(message.data)
        with lock:
            if is_nonempty:
                cloud_counts["nonempty"] += 1
                latest_nonempty_cloud["message"] = message
                latest_nonempty_cloud["stamp"] = message.header.stamp
            else:
                cloud_counts["empty"] += 1

    def image_callback(message):
        note("/rgb_img", message)

    def path_callback(message):
        note("/path", message)
        length = len(message.poses)
        with lock:
            if path_lengths["first"] is None:
                path_lengths["first"] = length
            path_lengths["last"] = length

    def snapshot():
        with lock:
            return {
                "topics": json.loads(json.dumps(topic_stats)),
                "path_lengths": dict(path_lengths),
                "cloud_counts": dict(cloud_counts),
                "latest_nonempty_cloud_available": latest_nonempty_cloud["message"] is not None,
            }

    def writer_loop():
        live_path = output_dir / "collector_live.json"
        while not stop_writer.wait(0.5):
            try:
                write_json_atomic(live_path, snapshot())
            except OSError:
                pass
        try:
            write_json_atomic(live_path, snapshot())
        except OSError:
            pass

    rospy.init_node("baseline_collector", anonymous=False)
    subscribers = [
        rospy.Subscriber("/aft_mapped_to_init", Odometry, odom_callback, queue_size=1000),
        rospy.Subscriber("/cloud_registered", PointCloud2, cloud_callback, queue_size=10),
        rospy.Subscriber("/rgb_img", Image, image_callback, queue_size=100),
        rospy.Subscriber("/path", RosPath, path_callback, queue_size=1),
    ]
    writer_thread = threading.Thread(target=writer_loop, daemon=True)
    writer_thread.start()
    (output_dir / "collector_ready").write_text("ready\n", encoding="utf-8")
    failure = None
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        failure = "{}: {}".format(type(error).__name__, error)
    finally:
        stop_writer.set()
        writer_thread.join(timeout=2.0)
        with lock:
            saved_cloud = latest_nonempty_cloud["message"]
            saved_stamp = latest_nonempty_cloud["stamp"]
        cloud_saved = False
        if saved_cloud is not None:
            try:
                with rosbag.Bag(str(output_dir / "last_cloud.bag"), "w") as bag:
                    bag.write("/cloud_registered", saved_cloud, saved_stamp)
                cloud_saved = True
            except Exception as error:
                failure = failure or "cloud bag write failed: {}".format(error)
        odom_stream.close()
        final_report = snapshot()
        final_report.update({
            "odom_csv": "odom.csv",
            "last_cloud_bag": "last_cloud.bag" if cloud_saved else None,
            "last_cloud_is_latest_nonempty_published_frame": cloud_saved,
            "last_cloud_is_global_map": False,
            "failure": failure,
        })
        write_json_atomic(output_dir / "collection.json", final_report)
    return 1 if failure else 0


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Run fastlivo_mapping with an isolated ROS1 master and collect a baseline.")
    parser.add_argument("--bag", help="input ROS1 bag file")
    parser.add_argument("--output", help="new output directory (must not exist)")
    parser.add_argument("--duration", type=float, default=30.0, help="maximum playback seconds (default: 30)")
    parser.add_argument("--rate", type=float, default=0.5, help="rosbag playback rate (default: 0.5)")
    parser.add_argument("--port", type=int, default=11331, help="isolated ROS master port (default: 11331)")
    parser.add_argument("--gdb", action="store_true",
                        help="diagnostic mode; requires --duration 5 and does not record comparable resources")
    parser.add_argument("--_collector-dir", help=argparse.SUPPRESS)
    return parser.parse_args(argv)


def validate_trajectory(path):
    row_count = 0
    invalid_lines = []
    previous_stamp = None
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line_number, line in enumerate(stream, 1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            try:
                if len(fields) != 8:
                    raise ValueError("expected 8 columns")
                values = [float(field) for field in fields]
                if not all(math.isfinite(value) for value in values):
                    raise ValueError("non-finite value")
                if previous_stamp is not None and values[0] <= previous_stamp:
                    raise ValueError("timestamps are not strictly increasing")
                quaternion_norm = math.sqrt(sum(value * value for value in values[4:8]))
                if quaternion_norm <= 1e-12:
                    raise ValueError("zero quaternion")
                previous_stamp = values[0]
                row_count += 1
            except ValueError as error:
                invalid_lines.append({"line": line_number, "reason": str(error)})
    return row_count, invalid_lines


def run_baseline(args):
    if not args.bag or not args.output:
        raise RuntimeError("--bag and --output are required")
    bag_path = Path(args.bag).expanduser().resolve()
    output_dir = Path(args.output).expanduser().resolve()
    if not bag_path.is_file():
        raise RuntimeError("bag file not found: {}".format(bag_path))
    if args.duration <= 0 or not math.isfinite(args.duration):
        raise RuntimeError("duration must be a finite positive number")
    if args.gdb and args.duration != 5.0:
        raise RuntimeError("--gdb diagnostic mode requires --duration 5")
    if args.rate <= 0 or not math.isfinite(args.rate):
        raise RuntimeError("rate must be a finite positive number")
    if args.port < 1 or args.port > 65535:
        raise RuntimeError("port must be between 1 and 65535")
    if output_dir.exists():
        raise RuntimeError("output directory must be new: {}".format(output_dir))
    if os.path.commonpath([str(output_dir), str(WORKSPACE_ROOT.resolve())]) != str(
            WORKSPACE_ROOT.resolve()):
        raise RuntimeError("output directory must be inside the workspace: {}".format(
            WORKSPACE_ROOT))
    log_root = (PACKAGE_ROOT / "Log").resolve()
    if os.path.commonpath([str(output_dir), str(log_root)]) == str(log_root):
        raise RuntimeError("output directory cannot be inside the mapper Log directory")
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    output_dir.mkdir()

    stamp = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    seq_name = "P0_baseline_{}_{}".format(stamp, uuid.uuid4().hex[:8])
    result_path = PACKAGE_ROOT / "Log" / "result" / (seq_name + ".txt")
    if result_path.exists():
        raise RuntimeError("generated sequence name unexpectedly already exists")

    summary = {
        "run_status": "failed",
        "bag": str(bag_path),
        "output_dir": str(output_dir),
        "requested_duration_s": args.duration,
        "diagnostic_mode": bool(args.gdb),
        "resource_metrics_comparable": not args.gdb,
        "mapper_launcher": "gdb" if args.gdb else "direct executable",
        "playback_rate": args.rate,
        "ros_master_port": args.port,
        "sequence_name": seq_name,
        "trajectory_exists": False,
        "trajectory_rows": 0,
        "exit_codes": {},
        "shutdown": {},
        "errors": [],
        "launch_difference": (
            "Directly starts devel/lib/livo_atlas/fastlivo_mapping. This omits the launch file's "
            "republish node and RViz; it loads the two YAML files and applies output overrides explicitly."),
        "last_cloud_note": (
            "last_cloud.bag contains one latest non-empty /cloud_registered frame, not a global map."),
    }
    procs = {"roscore": None, "mapper": None, "collector": None, "rosbag_play": None}
    logs = {}
    sampler = None
    guard = SourceLogGuard(output_dir)
    mapper_started = False
    output_created = True
    try:
        running_mappers = find_running_mappers()
        if running_mappers:
            raise RuntimeError("refusing to run while fastlivo_mapping is already running: {}".format(
                running_mappers))
        reject_busy_port(args.port)
        env = sourced_ros_environment(output_dir)
        master_uri = "http://127.0.0.1:{}".format(args.port)
        env["ROS_MASTER_URI"] = master_uri
        env["ROS_IP"] = "127.0.0.1"
        env.pop("ROS_HOSTNAME", None)
        rosparam = find_program("rosparam", env)
        roscore = find_program("roscore", env)
        rosbag = find_program("rosbag", env)
        gdb = find_program("gdb", env) if args.gdb else None
        mapper_bin = WORKSPACE_ROOT / "devel" / "lib" / "livo_atlas" / "fastlivo_mapping"
        if not mapper_bin.is_file() or not os.access(str(mapper_bin), os.X_OK):
            raise RuntimeError("built mapper executable not found or not executable: {}".format(mapper_bin))
        for label, filename in (
                ("roscore", "roscore.log"),
                ("mapper", "mapper.log"),
                ("collector", "collector.log"),
                ("rosbag_play", "bag_play.log")):
            logs[label] = output_dir / filename

        guard.capture()
        if find_running_mappers():
            raise RuntimeError("a fastlivo_mapping process appeared before baseline startup")

        procs["roscore"], master_log = launch_process(
            [roscore, "-p", str(args.port)], env, WORKSPACE_ROOT, logs["roscore"])
        logs["roscore_stream"] = master_log
        wait_for_master(master_uri, procs["roscore"])

        rosparam_log = output_dir / "rosparam.log"
        run_ros_command([rosparam, "load", str(PACKAGE_ROOT / "config" / "avia.yaml")],
                        env, rosparam_log)
        run_ros_command([rosparam, "load", str(PACKAGE_ROOT / "config" / "camera_pinhole.yaml"),
                         "/laserMapping"], env, rosparam_log)
        overrides = (
            ("/evo/pose_output_en", "true"),
            ("/evo/seq_name", seq_name),
            ("/pcd_save/pcd_save_en", "false"),
            ("/pcd_save/colmap_output_en", "false"),
            ("/image_save/img_save_en", "false"),
            ("/use_sim_time", "false"),
        )
        for name, value in overrides:
            run_ros_command([rosparam, "set", name, value], env, rosparam_log)
        run_ros_command([rosparam, "dump", str(output_dir / "params.yaml")], env, rosparam_log)

        if find_running_mappers():
            raise RuntimeError("refusing to start a second fastlivo_mapping process")
        mapper_command = [str(mapper_bin)]
        if args.gdb:
            mapper_command = [
                gdb,
                "--batch",
                "-ex", "set pagination off",
                "-ex", "handle SIGINT nostop noprint pass",
                "-ex", "run",
                "-ex", "thread apply all bt",
                "--args", str(mapper_bin),
            ]
        procs["mapper"], mapper_log = launch_process(
            mapper_command, env, WORKSPACE_ROOT, logs["mapper"])
        logs["mapper_stream"] = mapper_log
        mapper_started = True
        if not args.gdb:
            sampler = ResourceSampler(procs["mapper"].pid, output_dir / "resource_samples.csv")
            sampler.start()

        procs["collector"], collector_log = launch_process(
            [sys.executable, str(Path(__file__).resolve()), "--_collector-dir", str(output_dir)],
            env, WORKSPACE_ROOT, logs["collector"])
        logs["collector_stream"] = collector_log
        wait_for_nodes_and_topics(master_uri, procs["mapper"], procs["collector"])

        play_command = [
            rosbag, "play", "--quiet", "--rate", str(args.rate), "--duration",
            str(args.duration), "--wait-for-subscribers", str(bag_path),
        ]
        playback_started = time.monotonic()
        procs["rosbag_play"], play_log = launch_process(
            play_command, env, WORKSPACE_ROOT, logs["rosbag_play"])
        logs["rosbag_play_stream"] = play_log
        playback_deadline = playback_started + args.duration / args.rate + 30.0
        while procs["rosbag_play"].poll() is None:
            if time.monotonic() > playback_deadline:
                raise RuntimeError("rosbag play exceeded its playback timeout")
            if procs["mapper"].poll() is not None:
                raise RuntimeError("mapper exited during bag playback (code {})".format(
                    procs["mapper"].returncode))
            if procs["roscore"].poll() is not None:
                raise RuntimeError("roscore exited during bag playback")
            time.sleep(0.25)
        play_code = procs["rosbag_play"].wait()
        summary["playback_wall_seconds"] = time.monotonic() - playback_started
        summary["exit_codes"]["rosbag_play"] = play_code
        if play_code != 0:
            raise RuntimeError("rosbag play exited with code {}".format(play_code))

        stable_since = None
        previous_counts = None
        stable_deadline = time.monotonic() + 15.0
        while time.monotonic() < stable_deadline:
            if procs["mapper"].poll() is not None:
                raise RuntimeError("mapper exited while waiting for output counters to stabilize")
            live_path = output_dir / "collector_live.json"
            try:
                live = json.loads(live_path.read_text(encoding="utf-8"))
                counts = {name: data["count"] for name, data in live["topics"].items()}
            except (OSError, ValueError, KeyError, TypeError):
                counts = None
            if counts is not None and counts == previous_counts:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= 2.0:
                    break
            else:
                previous_counts = counts
                stable_since = None
            time.sleep(0.25)
        else:
            summary["errors"].append("output counters did not stabilize within 15 seconds")

        summary["run_status"] = "complete" if not summary["errors"] else "failed"
    except KeyboardInterrupt:
        summary["errors"].append("interrupted by user")
    except Exception as error:
        summary["errors"].append("{}: {}".format(type(error).__name__, error))
        with (output_dir / "runner.log").open("a", encoding="utf-8") as stream:
            traceback.print_exc(file=stream)
    finally:
        stop_process(procs["rosbag_play"], "rosbag_play", summary["shutdown"])
        stop_process(procs["mapper"], "mapper", summary["shutdown"],
                     signal_group=not args.gdb)
        if sampler is not None:
            sampler.stop()
            summary["resource_peak_rss_bytes"] = sampler.peak_rss_bytes or None
            summary["resource_peak_vmhwm_bytes"] = sampler.peak_vmhwm_bytes or None
            summary["resource_sample_count"] = sampler.samples
            summary["resource_samples_file"] = "resource_samples.csv"
        stop_process(procs["collector"], "collector", summary["shutdown"])
        stop_process(procs["roscore"], "roscore", summary["shutdown"])
        for label, process in procs.items():
            summary["exit_codes"][label] = None if process is None else process.poll()
        if mapper_started and not args.gdb and summary["exit_codes"].get("mapper") != 0:
            summary["errors"].append("mapper exited with code {}".format(
                summary["exit_codes"].get("mapper")))
        for stream_key in ("rosbag_play_stream", "mapper_stream", "collector_stream", "roscore_stream"):
            stream = logs.get(stream_key)
            if stream is not None:
                stream.close()

        if mapper_started:
            run_log_dir = output_dir / "mapper_source_logs"
            run_log_dir.mkdir(parents=True, exist_ok=True)
            copied_logs = []
            for name in SOURCE_LOG_NAMES:
                source_path = PACKAGE_ROOT / "Log" / name
                if source_path.is_file():
                    try:
                        shutil.copy2(str(source_path), str(run_log_dir / name))
                        copied_logs.append(name)
                    except OSError as error:
                        summary["errors"].append(
                            "mapper source log copy failed for {}: {}".format(name, error))
            summary["mapper_source_logs"] = copied_logs
            if result_path.exists():
                try:
                    shutil.copy2(str(result_path), str(output_dir / "trajectory.txt"))
                    summary["trajectory_exists"] = True
                    (summary["trajectory_rows"], invalid_lines) = validate_trajectory(
                        output_dir / "trajectory.txt")
                    summary["trajectory_valid"] = (
                        summary["trajectory_rows"] > 0 and not invalid_lines)
                    summary["trajectory_invalid_lines"] = invalid_lines
                except OSError as error:
                    summary["errors"].append("trajectory copy failed: {}".format(error))
                try:
                    result_path.unlink()
                    summary["source_trajectory_removed"] = True
                except OSError as error:
                    summary["errors"].append("generated source trajectory cleanup failed: {}".format(error))
            try:
                guard.restore()
                summary["source_logs_restored"] = True
            except OSError as error:
                summary["source_logs_restored"] = False
                summary["errors"].append("source log restore failed: {}".format(error))

        try:
            collection_path = output_dir / "collection.json"
            if collection_path.is_file():
                collection = json.loads(collection_path.read_text(encoding="utf-8"))
                summary["topics"] = collection.get("topics", {})
                summary["path_lengths"] = collection.get("path_lengths", {})
                summary["cloud_counts"] = collection.get("cloud_counts", {})
                summary["last_cloud_saved"] = collection.get(
                    "last_cloud_is_latest_nonempty_published_frame", False)
                summary["collection_failure"] = collection.get("failure")
                if collection.get("failure"):
                    summary["errors"].append("collector failed: {}".format(collection["failure"]))
                if collection.get("topics", {}).get("/aft_mapped_to_init", {}).get("count", 0) == 0:
                    summary["errors"].append("no odometry messages were collected")
                if collection.get("topics", {}).get("/rgb_img", {}).get("count", 0) == 0:
                    summary["errors"].append("no image messages were collected")
                if collection.get("cloud_counts", {}).get("nonempty", 0) == 0:
                    summary["errors"].append("no non-empty registered point cloud was collected")
            else:
                summary["errors"].append("collector did not produce collection.json")
        except (OSError, ValueError) as error:
            summary["errors"].append("collection summary read failed: {}".format(error))

        if mapper_started and not summary["trajectory_exists"]:
            summary["errors"].append("mapper did not produce its requested trajectory file")
        if mapper_started and summary["trajectory_rows"] == 0:
            summary["errors"].append("mapper trajectory contains no pose rows")
        if mapper_started and not summary.get("trajectory_valid", False):
            summary["errors"].append("mapper trajectory failed format or value validation")
        collector_code = summary["exit_codes"].get("collector")
        if collector_code not in (None, 0):
            summary["errors"].append("collector exited with code {}".format(collector_code))

        if summary["errors"]:
            summary["run_status"] = "failed"
        if args.gdb:
            summary["run_status"] = "diagnostic"
            try:
                mapper_log_text = logs["mapper"].read_text(
                    encoding="utf-8", errors="replace")
                signal_match = re.search(
                    r"(?:Program (?:received|terminated with) signal|"
                    r"Thread\s+\d+(?:\s+\"[^\"]*\")?\s+received signal)\s+"
                    r"(SIG[A-Z0-9]+)",
                    mapper_log_text)
                summary["diagnostic_signal"] = (
                    signal_match.group(1) if signal_match else None)
            except OSError as error:
                summary["diagnostic_log_read_error"] = str(error)
        if output_created:
            try:
                write_json_atomic(output_dir / "summary.json", summary)
            except OSError:
                pass
    return 0 if summary["run_status"] == "complete" else 1


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args._collector_dir:
        try:
            return collector_main(args._collector_dir)
        except Exception:
            traceback.print_exc()
            return 1
    try:
        return run_baseline(args)
    except Exception as error:
        print("run_baseline: {}".format(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
