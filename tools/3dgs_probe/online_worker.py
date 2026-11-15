#!/usr/bin/env python3
"""Serve one bounded online Gaussian session over a private Unix socket."""

import argparse
import importlib.metadata
import os
from pathlib import Path
import resource
import select
import socket
import stat
import sys

import torch

from online_model import OnlineGaussianModel, checked_cuda_device
from online_protocol import decode_view, read_message, write_message


def create_model():
    device = checked_cuda_device()
    model = OnlineGaussianModel(device)
    free, total = torch.cuda.mem_get_info(device)
    return model, {
        "python": sys.version.split()[0],
        "torch": torch.__version__,
        "gsplat": importlib.metadata.version("gsplat"),
        "cuda": torch.version.cuda or "unknown",
        "device": torch.cuda.get_device_name(device),
        "gpu_free_bytes": str(free),
        "gpu_total_bytes": str(total),
    }


def _base(kind, session):
    return {"schema": 1, "type": kind, "session_id": session}


def _identity(header):
    return tuple(header[name] for name in
                 ("job_id", "generation", "submap_id", "pose_version"))


def _read_when_ready(sock, transfer_timeout):
    # A paused dataset can leave the connection idle indefinitely. Once the
    # first byte arrives, the whole message has a bounded transfer deadline.
    while not select.select([sock], [], [], 1)[0]:
        pass
    return read_message(sock, transfer_timeout)


def serve_connection(sock, model_factory=create_model, transfer_timeout=5.0):
    """Handle one session; any protocol or model failure closes the session."""
    session = None
    try:
        hello, _ = read_message(sock, transfer_timeout)
        if hello["type"] != "hello":
            raise ValueError("Expected hello")
        session = hello["session_id"]
        try:
            model, metadata = model_factory()
        except Exception as error:
            code = ("extension_unavailable" if "gsplat.csrc" in str(error)
                    else "gpu_unavailable")
            write_message(sock, dict(_base("error", session), code=code), b"",
                          transfer_timeout)
            print("Gaussian worker initialization failed:", error, file=sys.stderr)
            return False
        write_message(sock, dict(_base("hello_ok", session), **metadata), b"",
                      transfer_timeout)
        current = None
        pending = None
        last_job = -1
        last_frame = -1
        last_frontier = 0
        last_revision = 0
        while True:
            header, body = _read_when_ready(sock, transfer_timeout)
            if header["session_id"] != session:
                raise ValueError("Session mismatch")
            kind = header["type"]
            if kind == "stop":
                return True
            if pending is not None:
                if kind != "decision" or _identity(header) != _identity(pending):
                    raise ValueError("Expected the matching candidate decision")
                accept = header["action"] == "accept"
                try:
                    model.decide(accept)
                except Exception as error:
                    raise RuntimeError("Gaussian decision failed: " + str(error)) from error
                reply = dict(_base("commit_ok" if accept else "discard_ok", session),
                             **{key: pending[key] for key in
                                ("job_id", "generation", "submap_id", "pose_version")})
                write_message(sock, reply, b"", transfer_timeout)
                pending = None
                if not accept:
                    last_frame = -1
                continue
            if kind == "reset":
                generation = int(header["generation"])
                version = int(header["pose_version"])
                submap = header.get("submap_id")
                if current is not None and (generation <= current[0] or version < current[1]):
                    raise ValueError("Reset generation or pose version went backwards")
                try:
                    model.reset(generation)
                except Exception as error:
                    raise RuntimeError("Gaussian reset failed: " + str(error)) from error
                current = (generation, version, submap)
                last_frame = -1
                last_frontier = 0
                last_revision = 0
                reply = dict(_base("reset_ok", session), generation=header["generation"],
                             pose_version=header["pose_version"])
                if submap is not None:
                    reply["submap_id"] = submap
                write_message(sock, reply, b"", transfer_timeout)
                continue
            if kind != "view" or current is None:
                raise ValueError("Expected reset or view")
            identity = (int(header["generation"]), int(header["pose_version"]),
                        header["submap_id"])
            job = int(header["job_id"])
            frame = int(header["keyframe_id"])
            frontier = int(header["frame_count"])
            revision = int(header["membership_revision"])
            if (identity != current or job <= last_job or frame <= last_frame or
                    frontier < last_frontier or revision < last_revision or frame >= 512):
                raise ValueError("View identity or ordering is invalid")
            last_job, last_frame = job, frame
            last_frontier, last_revision = frontier, revision
            view = decode_view(header, body)
            try:
                preview, metrics = model.train_candidate(view)
            except Exception as error:
                raise RuntimeError("Gaussian training failed: " + str(error)) from error
            pending = header
            candidate = dict(_base("candidate", session), **{
                key: header[key] for key in
                ("job_id", "generation", "submap_id", "pose_version",
                 "membership_revision", "frame_count", "optimized_frame_count",
                 "width", "height")})
            candidate.update({key: metrics[key] for key in
                              ("steps", "rgb_l1", "depth_l1_m",
                               "supervised_depth_pixels", "gaussian_count")})
            candidate["frame_ids"] = [str(value) for value in metrics["frame_ids"]]
            if "nonzero_gradient" in metrics:
                candidate["nonzero_gradient"] = int(metrics["nonzero_gradient"])
                candidate["parameter_updated"] = int(metrics["parameter_updated"])
                candidate["gpu_peak_allocated_bytes"] = str(torch.cuda.max_memory_allocated())
                candidate["gpu_peak_reserved_bytes"] = str(torch.cuda.max_memory_reserved())
                # Linux reports ru_maxrss in KiB.
                candidate["worker_peak_rss_bytes"] = str(
                    resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024)
            write_message(sock, candidate, preview.tobytes(), transfer_timeout)
    except (ValueError, KeyError) as error:
        code = "protocol_error"
        detail = str(error)
    except (OSError, TimeoutError) as error:
        print("Gaussian worker connection closed:", error, file=sys.stderr)
        return False
    except Exception as error:
        code = "worker_error"
        detail = str(error)
    print("Gaussian worker failed:", detail, file=sys.stderr)
    if session is not None:
        try:
            write_message(sock, dict(_base("error", session), code=code), b"",
                          transfer_timeout)
        except (OSError, TimeoutError, ValueError):
            pass
    return False


def serve(socket_path):
    path = Path(socket_path).absolute()
    parent = path.parent.stat()
    if (not stat.S_ISDIR(parent.st_mode) or parent.st_uid != os.getuid() or
            stat.S_IMODE(parent.st_mode) & 0o077):
        raise ValueError("Socket parent must be a private directory owned by this user")
    if os.path.lexists(path):
        raise FileExistsError("Socket path already exists")
    if len(os.fsencode(path)) >= 108:
        raise ValueError("Unix socket path is too long")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(str(path))
        os.chmod(path, 0o600)
        created = path.lstat()
        try:
            listener.listen(1)
            connection, _ = listener.accept()
            with connection:
                return serve_connection(connection)
        finally:
            try:
                current = path.lstat()
                if (stat.S_ISSOCK(current.st_mode) and current.st_ino == created.st_ino and
                        current.st_dev == created.st_dev):
                    path.unlink()
            except FileNotFoundError:
                pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True, help="New socket in a private directory")
    args = parser.parse_args()
    try:
        return 0 if serve(args.socket) else 1
    except (OSError, ValueError) as error:
        print("Gaussian worker failed:", error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
