#!/usr/bin/env python3
"""Check that invalid extrinsics fail before the mapper is constructed."""

import argparse
import hashlib
import os
import subprocess
import sys
from pathlib import Path

import run_baseline


PACKAGE_ROOT = run_baseline.PACKAGE_ROOT
WORKSPACE_ROOT = run_baseline.WORKSPACE_ROOT
MAPPER_BINARY = WORKSPACE_ROOT / "devel" / "lib" / "livo_atlas" / "fastlivo_mapping"
CASES = (
    ("missing_camera_rotation", "extrin_calib/Rcl",
     ("delete", "/extrin_calib/Rcl")),
    ("short_imu_translation", "extrin_calib/extrinsic_T",
     ("set", "/extrin_calib/extrinsic_T", "[0.04165, 0.02326]")),
    ("reflected_imu_rotation", "extrin_calib/extrinsic_R",
     ("set", "/extrin_calib/extrinsic_R",
      "[-1, 0, 0, 0, 1, 0, 0, 0, 1]")),
)


def source_log_hashes():
    hashes = {}
    log_dir = PACKAGE_ROOT / "Log"
    for name in run_baseline.SOURCE_LOG_NAMES:
        path = log_dir / name
        if path.is_file():
            hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        else:
            hashes[name] = None
    return hashes


def read_startup_diagnostics(mapper_log):
    try:
        return mapper_log.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def run_case(name, parameter_name, mutation, env, rosparam, output_dir,
             rosparam_log, shutdown):
    run_baseline.run_ros_command(
        [rosparam, "load", str(PACKAGE_ROOT / "config" / "avia.yaml")],
        env, rosparam_log)
    run_baseline.run_ros_command(
        [rosparam, "load", str(PACKAGE_ROOT / "config" / "camera_pinhole.yaml"),
         "/laserMapping"], env, rosparam_log)
    run_baseline.run_ros_command([rosparam, *mutation], env, rosparam_log)

    mapper_log = output_dir / (name + ".log")
    process, log_stream = run_baseline.launch_process(
        [str(MAPPER_BINARY)], env, WORKSPACE_ROOT, mapper_log)
    timed_out = False
    try:
        try:
            return_code = process.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            timed_out = True
            return_code = process.poll()
    finally:
        try:
            run_baseline.stop_process(process, name, shutdown)
            return_code = process.poll()
        finally:
            log_stream.close()

    diagnostics = read_startup_diagnostics(mapper_log)
    return {
        "name": name,
        "parameter": parameter_name,
        "return_code": return_code,
        "timed_out": timed_out,
        "diagnostic_contains_parameter": parameter_name in diagnostics,
        "log": str(mapper_log.relative_to(output_dir)),
    }


def run_checks(output_dir, port):
    if not MAPPER_BINARY.is_file() or not os.access(str(MAPPER_BINARY), os.X_OK):
        raise RuntimeError("built mapper executable not found or not executable: {}".format(
            MAPPER_BINARY))
    running_mappers = run_baseline.find_running_mappers()
    if running_mappers:
        raise RuntimeError("refusing to run while fastlivo_mapping is already running: {}".format(
            running_mappers))
    run_baseline.reject_busy_port(port)

    env = run_baseline.sourced_ros_environment(output_dir)
    master_uri = "http://127.0.0.1:{}".format(port)
    env["ROS_MASTER_URI"] = master_uri
    env["ROS_IP"] = "127.0.0.1"
    env.pop("ROS_HOSTNAME", None)
    rosparam = run_baseline.find_program("rosparam", env)
    roscore = run_baseline.find_program("roscore", env)

    summary = {
        "run_status": "failed",
        "ros_master_port": port,
        "source_log_hashes_before": source_log_hashes(),
        "cases": [],
        "shutdown": {},
        "errors": [],
    }
    processes = {"roscore": None}
    streams = []
    source_log_guard = run_baseline.SourceLogGuard(output_dir)
    source_log_guard_captured = False
    try:
        source_log_guard_captured = True
        source_log_guard.capture()
        master_log = output_dir / "roscore.log"
        processes["roscore"], master_stream = run_baseline.launch_process(
            [roscore, "-p", str(port)], env, WORKSPACE_ROOT, master_log)
        streams.append(master_stream)
        run_baseline.wait_for_master(master_uri, processes["roscore"])

        rosparam_log = output_dir / "rosparam.log"
        for name, parameter_name, mutation in CASES:
            if run_baseline.find_running_mappers():
                raise RuntimeError("refusing to start a second fastlivo_mapping process")
            result = run_case(name, parameter_name, mutation, env, rosparam,
                              output_dir, rosparam_log, summary["shutdown"])
            summary["cases"].append(result)
            if result["return_code"] != 1:
                summary["errors"].append(
                    "{} returned {}, expected exit code 1".format(
                        name, result["return_code"]))
            if result["timed_out"]:
                summary["errors"].append("{} did not fail during startup".format(name))
            if not result["diagnostic_contains_parameter"]:
                summary["errors"].append(
                    "{} diagnostic did not contain {}".format(name, parameter_name))

        summary["run_status"] = "complete" if not summary["errors"] else "failed"
    except KeyboardInterrupt:
        summary["errors"].append("interrupted by user")
    except Exception as error:
        summary["errors"].append("{}: {}".format(type(error).__name__, error))
    finally:
        run_baseline.stop_process(processes["roscore"], "roscore", summary["shutdown"])
        for stream in streams:
            stream.close()
        summary["source_log_hashes_after"] = source_log_hashes()
        summary["source_logs_unchanged"] = (
            summary["source_log_hashes_before"] == summary["source_log_hashes_after"])
        if not summary["source_logs_unchanged"]:
            summary["errors"].append("mapper source log hashes changed")
            summary["run_status"] = "failed"
        if source_log_guard_captured:
            try:
                source_log_guard.restore()
                summary["source_logs_restored"] = True
            except OSError as error:
                summary["source_logs_restored"] = False
                summary["errors"].append("source log restore failed: {}".format(error))
                summary["run_status"] = "failed"
        run_baseline.write_json_atomic(output_dir / "summary.json", summary)
    return 0 if summary["run_status"] == "complete" else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", default="validation/P1-02a/startup-check",
        help="new output directory inside the workspace")
    parser.add_argument("--port", type=int, default=11333)
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    output_dir = Path(args.output).expanduser()
    if not output_dir.is_absolute():
        output_dir = WORKSPACE_ROOT / output_dir
    output_dir = output_dir.resolve()
    if output_dir.exists():
        print("output directory must be new: {}".format(output_dir), file=sys.stderr)
        return 2
    if os.path.commonpath([str(output_dir), str(WORKSPACE_ROOT.resolve())]) != str(
            WORKSPACE_ROOT.resolve()):
        print("output directory must be inside the workspace", file=sys.stderr)
        return 2
    log_root = (PACKAGE_ROOT / "Log").resolve()
    if os.path.commonpath([str(output_dir), str(log_root)]) == str(log_root):
        print("output directory cannot be inside the mapper Log directory", file=sys.stderr)
        return 2
    if args.port < 1 or args.port > 65535:
        print("port must be between 1 and 65535", file=sys.stderr)
        return 2

    output_dir.mkdir(parents=True)
    try:
        return run_checks(output_dir, args.port)
    except Exception as error:
        print("check_extrinsics_startup: {}".format(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
