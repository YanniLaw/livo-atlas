#!/usr/bin/env python3
"""Test peer for exercising the real ROS entry point without a GPU."""

import argparse
import os
from pathlib import Path
import socket
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/3dgs_probe"))
from online_protocol import read_message, write_message  # noqa: E402


def serve(path):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(str(path))
        os.chmod(path, 0o600)
        listener.listen(1)
        created = path.lstat()
        try:
            connection, _ = listener.accept()
            with connection:
                hello, _ = read_message(connection, 20)
                session = hello["session_id"]
                base = {"schema": 1, "session_id": session}
                metadata = {"type": "hello_ok", "python": "test", "torch": "test",
                            "gsplat": "test", "cuda": "test", "device": "test",
                            "gpu_free_bytes": "5000000000", "gpu_total_bytes": "8000000000"}
                write_message(connection, dict(base, **metadata), b"", 5)
                frames = []
                while True:
                    header, body = read_message(connection, 180)
                    kind = header["type"]
                    print(kind, header.get("generation", ""), header.get("job_id", ""),
                          flush=True)
                    if kind == "stop":
                        return
                    if kind == "reset":
                        frames.clear()
                        reply = dict(base, type="reset_ok", generation=header["generation"],
                                     pose_version=header["pose_version"])
                        if "submap_id" in header:
                            reply["submap_id"] = header["submap_id"]
                        write_message(connection, reply, b"", 5)
                    elif kind == "view":
                        frames.append(header["keyframe_id"])
                        frames = frames[-10:]
                        identity = {name: header[name] for name in
                                    ("job_id", "generation", "submap_id", "pose_version",
                                     "membership_revision", "frame_count",
                                     "optimized_frame_count", "width", "height")}
                        candidate = dict(base, type="candidate", **identity, frame_ids=frames,
                                         steps=20, rgb_l1=0.2, depth_l1_m=0.1,
                                         supervised_depth_pixels=1, gaussian_count=1)
                        pixels = header["width"] * header["height"]
                        write_message(connection, candidate, bytes([30, 20, 10]) * pixels, 5)
                    elif kind == "decision":
                        if header["action"] == "discard":
                            frames.clear()
                        reply = dict(base, type=("commit_ok" if header["action"] == "accept"
                                                 else "discard_ok"), **{
                            name: header[name] for name in
                            ("job_id", "generation", "submap_id", "pose_version")})
                        write_message(connection, reply, b"", 5)
                    else:
                        raise ValueError("Unexpected mapper message: " + kind)
        finally:
            try:
                current = path.lstat()
                if (current.st_dev, current.st_ino) == (created.st_dev, created.st_ino):
                    path.unlink()
            except FileNotFoundError:
                pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", type=Path, required=True)
    serve(parser.parse_args().socket)
