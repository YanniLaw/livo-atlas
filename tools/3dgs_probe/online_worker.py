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

from online_model import OnlineGaussianModel
from online_protocol import decode_view, read_message, write_message
from worker_config import (DEFAULT_CONFIG, DEFAULT_SEED_POLICY, DEFAULT_TRANSFER_TIMEOUT_MS,
                           DEFAULT_WORKER_OPTIONS, SEED_POLICIES, load_worker_config)


def create_model(options=DEFAULT_WORKER_OPTIONS):
    model = OnlineGaussianModel(options.device, options=options)
    device = model.device
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


def _base(kind, session, schema=1):
    return {"schema": schema, "type": kind, "session_id": session}


def _identity(header):
    return tuple(header[name] for name in
                 ("job_id", "generation", "submap_id", "pose_version"))


def _read_when_ready(sock, transfer_timeout):
    # A paused dataset can leave the connection idle indefinitely. Once the
    # first byte arrives, the whole message has a bounded transfer deadline.
    while not select.select([sock], [], [], 1)[0]:
        pass
    return read_message(sock, transfer_timeout)


def _serve_submaps(sock, session, directory, model, metadata, transfer_timeout,
                   joint_render_enabled=False, spatial_render_enabled=False, *,
                   seed_policy=DEFAULT_SEED_POLICY, split=None, evaluation_limits=None):
    from gaussian_state import CommitUncertainError
    from submap_worker import GaussianCapacityError, GaussianSubmapWorker

    actor = None
    request = {}
    try:
        actor = GaussianSubmapWorker(session, Path(directory), model, metadata,
                                     joint_render_enabled=joint_render_enabled,
                                     spatial_render_enabled=spatial_render_enabled,
                                     seed_policy=seed_policy, split=split, evaluation_limits=evaluation_limits)
        capabilities = dict(capabilities=["joint_render_v1"]) if joint_render_enabled else {}
        if spatial_render_enabled:
            capabilities['capabilities'].append('joint_render_spatial_v1')
        if split is not None:
            from holdout_split import CAPABILITY

            capabilities.setdefault('capabilities', []).append(CAPABILITY)
            capabilities['split_id'] = split.split_id
        if evaluation_limits is not None:
            from evaluation_protocol import CAPABILITY
            capabilities['capabilities'].append(CAPABILITY)
            capabilities['evaluation_limits'] = dict(evaluation_limits)
        write_message(sock, dict(_base("hello_ok", session, 2), **actor.software, **capabilities,
                                 mode="persistent_submaps", state_schema=2 if split is not None else 1), b"", transfer_timeout)
        while True:
            request, body = _read_when_ready(sock, transfer_timeout)
            reply, output = actor.handle(request, body)
            write_message(sock, reply, output, transfer_timeout)
            if reply["type"] == "stopped":
                return True
    except GaussianCapacityError as error:
        code, detail = "resource_limit", str(error)
    except CommitUncertainError as error:
        code, detail = "commit_uncertain", str(error)
    except (ValueError, KeyError) as error:
        code, detail = "protocol_error", str(error)
    except (OSError, TimeoutError) as error:
        if actor is not None and not actor.failed:
            print("Gaussian submap worker connection closed:", error, file=sys.stderr)
            return False
        code, detail = "storage_error", str(error)
    except Exception as error:
        code, detail = "worker_error", str(error)
    print("Gaussian submap worker failed:", detail, file=sys.stderr)
    # Bound UTF-8 bytes, matching the C++ parser, even for a translated OS error.
    detail = detail.encode("utf-8", errors="replace")[:2048].decode("utf-8", errors="ignore")
    response = dict(_base("error", session, 2), code=code, message=detail)
    response.update({name: request[name] for name in
                     ("job_id", "generation", "pose_version", "submap_id") if name in request})
    try:
        write_message(sock, response, b"", transfer_timeout)
    except (OSError, TimeoutError, ValueError):
        pass
    return False


def serve_connection(sock, model_factory=create_model,
                     transfer_timeout=DEFAULT_TRANSFER_TIMEOUT_MS / 1000, *,
                     seed_policy=DEFAULT_SEED_POLICY):
    """Handle one session; any protocol or model failure closes the session."""
    session = None
    schema = 1
    try:
        hello, hello_body = read_message(sock, transfer_timeout)
        if hello["type"] != "hello":
            raise ValueError("Expected hello")
        session = hello["session_id"]
        schema = hello["schema"]
        split = None
        from holdout_split import CAPABILITY, HoldoutSplit

        if CAPABILITY in hello.get('capabilities', []):
            split = HoldoutSplit(hello_body)
            if split.split_id != hello['split_id']:
                raise ValueError('Hello split_id differs from the frozen manifest')
        if seed_policy not in SEED_POLICIES:
            raise ValueError("Unknown Gaussian seed policy")
        if schema == 1 and seed_policy == "coverage":
            raise ValueError("Coverage seeding requires persistent submaps (schema 2)")
        try:
            model, metadata = model_factory()
        except Exception as error:
            code = ("extension_unavailable" if "gsplat.csrc" in str(error)
                    else "gpu_unavailable")
            write_message(sock, dict(_base("error", session, schema), code=code), b"",
                          transfer_timeout)
            print("Gaussian worker initialization failed:", error, file=sys.stderr)
            return False
        if schema == 2:
            return _serve_submaps(sock, session, hello["state_directory"], model, metadata,
                                  transfer_timeout, 'joint_render_v1' in hello.get('capabilities', []),
                                  'joint_render_spatial_v1' in hello.get('capabilities', []),
                                  seed_policy=seed_policy, split=split,
                                  evaluation_limits=hello.get('evaluation_limits'))
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
            if header["schema"] != schema or header["session_id"] != session:
                raise ValueError("Session or negotiated schema mismatch")
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
                candidate["gpu_peak_allocated_bytes"] = str(torch.cuda.max_memory_allocated(model.device))
                candidate["gpu_peak_reserved_bytes"] = str(torch.cuda.max_memory_reserved(model.device))
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
            write_message(sock, dict(_base("error", session, schema), code=code), b"",
                          transfer_timeout)
        except (OSError, TimeoutError, ValueError):
            pass
    return False


def serve(socket_path, *, seed_policy=DEFAULT_SEED_POLICY,
          transfer_timeout=DEFAULT_TRANSFER_TIMEOUT_MS / 1000, options=DEFAULT_WORKER_OPTIONS):
    if seed_policy not in SEED_POLICIES:
        raise ValueError("Unknown Gaussian seed policy")
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
                return serve_connection(connection, lambda: create_model(options), transfer_timeout,
                                        seed_policy=seed_policy)
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
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help="Gaussian YAML file, defaults to config/gaussian.yaml")
    parser.add_argument("--socket", help="Override gaussian.socket_path with a new private socket")
    parser.add_argument("--seed-policy", choices=SEED_POLICIES,
                        help="Override gaussian.worker.seed_policy (coverage requires persistence)")
    args = parser.parse_args()
    try:
        config = load_worker_config(args.config)
        socket_path = config.socket_path if args.socket is None else args.socket
        if not socket_path:
            raise ValueError("Set gaussian.socket_path in the config or provide --socket")
        return 0 if serve(socket_path, seed_policy=args.seed_policy or config.seed_policy,
                          transfer_timeout=config.transfer_timeout_ms / 1000,
                          options=config.options) else 1
    except (OSError, ValueError) as error:
        print("Gaussian worker failed:", error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
