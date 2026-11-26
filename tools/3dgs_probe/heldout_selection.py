"""Final-snapshot selection using the same native spatial ranking as online."""

import json
import hashlib
import re
import sys
import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory
import time

from gaussian_map import _require
from gaussian_state import _open_file, _unique_object, _reject_constant, read_state
from heldout_query import verify_evaluation_snapshot
from prepare_holdout_split import file_sha
from spatial_metadata import extract_metadata


class EvaluationSelector:
    def __init__(self, snapshot, policy, executable=None):
        _require(policy in ("temporal", "spatial"), "Unknown evaluation policy")
        self.snapshot, self.policy = snapshot, policy
        self.executable, self.sha256 = None, None
        self.metadata = {}
        self.environment = dict(os.environ)
        self.libraries = {}
        self.runtime_sha256 = None
        if policy == "spatial":
            _require(
                executable is not None, "Spatial scoring requires a selector executable"
            )
            self.executable = Path(executable).resolve(strict=True)
            _require(
                self.executable.is_file()
                and os.access(self.executable, os.X_OK)
                and self.executable.stat().st_size <= 64 * 1024 * 1024,
                "Invalid selector executable",
            )
            self.sha256 = file_sha(self.executable)
            self.libraries = {
                str(path): file_sha(path) for path in self._runtime_libraries()
            }
            records = [("executable", self.sha256)] + [
                (Path(path).name, digest) for path, digest in self.libraries.items()
            ]
            self.runtime_sha256 = hashlib.sha256(
                json.dumps(sorted(records), separators=(",", ":")).encode()
            ).hexdigest()
            for row in snapshot.document["entries"]:
                if "asset" not in row or row["availability"] != "current":
                    continue
                state = read_state(
                    snapshot.directory / "states" / row["asset"]["asset_id"]
                )
                snapshot.split.check_state(state)
                self.metadata[int(row["submap_id"])] = extract_metadata(
                    state, row["content"]
                )
                del state
        self.verify()

    def _runtime_libraries(self):
        # The native bridge is Linux-only. Bind project DSOs as well as the
        # launcher; in a static SDK build the ranking is in the launcher itself.
        process = subprocess.run(
            ["ldd", str(self.executable)],
            env=self.environment,
            timeout=10,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        _require(
            len(process.stdout) <= 64 * 1024,
            "Runtime dependency listing exceeds budget",
        )
        paths = []
        for line in process.stdout.decode(errors="strict").splitlines():
            if not line.strip().startswith("liblivo_atlas"):
                continue
            match = re.fullmatch(
                r"\s*liblivo_atlas\S+\s+=>\s+(.+?)\s+\(0x[0-9a-fA-F]+\)\s*", line
            )
            _require(match is not None, "Selector project dependency is unavailable")
            paths.append(Path(match.group(1)).resolve(strict=True))
        return sorted(set(paths))

    def verify(self):
        verify_evaluation_snapshot(self.snapshot)
        if self.executable is not None:
            _require(
                file_sha(self.executable) == self.sha256,
                "Selector executable changed during scoring",
            )
            paths = self._runtime_libraries()
            _require(
                {str(path) for path in paths} == self.libraries.keys(),
                "Selector runtime dependency resolution changed",
            )
            _require(
                all(
                    file_sha(Path(path)) == digest
                    for path, digest in self.libraries.items()
                ),
                "Selector runtime library changed during scoring",
            )

    def select(self, frame):
        self.verify()
        owner, key = int(frame["submap_id"]), int(frame["keyframe_id"])
        rows = self.snapshot.document["entries"]
        _require(
            0 <= owner < len(rows) and 0 <= key < len(self.snapshot.poses),
            "Unknown selection query",
        )
        result = dict(
            selection_policy=self.policy,
            selected_ids=[],
            neighbor_reason="missing_model",
            selection=None,
        )
        if "asset" not in rows[owner] or rows[owner]["availability"] != "current":
            return result
        result["selected_ids"] = [owner]
        if self.policy == "temporal":
            previous = owner - 1
            if (
                previous >= 0
                and "asset" in rows[previous]
                and rows[previous]["availability"] == "current"
            ):
                result["selected_ids"].insert(0, previous)
                result["neighbor_reason"] = ""
            else:
                result["neighbor_reason"] = (
                    "neighbor_unavailable" if owner else "no_previous_group"
                )
            return result
        metadata = self.metadata[owner]
        _require(
            all(frame[key] == metadata["camera"][key] for key in ("width", "height")),
            "Spatial query dimensions differ from owner calibration",
        )
        camera = dict(
            keyframe_id=frame["keyframe_id"],
            calibration_id=frame["calibration_id"],
            width=frame["width"],
            height=frame["height"],
            K=frame["intrinsics"],
            camera_from_body=frame["camera_from_body"],
            capture_time=frame["image_timestamp"],
        )
        request = dict(
            schema=1,
            camera=camera,
            owner=str(owner),
            parent_from_body=self.snapshot.poses[key].ravel().tolist(),
            candidates=[
                dict(
                    metadata=self.metadata[group],
                    parent_from_submap=rows[group]["binding"]["parent_from_submap"],
                )
                for group in sorted(self.metadata)
                if group < owner
            ],
        )
        started = time.perf_counter()
        with TemporaryDirectory(prefix="livo-spatial-select-") as temporary:
            root = Path(temporary)
            input_file = root / "input.json"
            output_file = root / "output.json"
            payload = json.dumps(request, allow_nan=False, indent=2).encode()
            _require(
                len(payload) <= 512 * 1024, "Spatial selection input exceeds budget"
            )
            input_file.write_bytes(payload)
            # Use a fresh interpreter instead of preexec_fn after CUDA/thread
            # initialization. The hard file-size bound survives exec/fork.
            launch = (
                "import os,resource,sys; "
                "resource.setrlimit(resource.RLIMIT_FSIZE,(65536,65536)); "
                "os.execve(sys.argv[1],sys.argv[1:],dict(os.environ))"
            )
            subprocess.run(
                [
                    sys.executable,
                    "-c",
                    launch,
                    str(self.executable),
                    str(input_file),
                    str(output_file),
                ],
                env=self.environment,
                check=True,
                timeout=10,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            with _open_file(output_file, 64 * 1024) as stream:
                answer = json.loads(
                    stream.read(),
                    object_pairs_hook=_unique_object,
                    parse_constant=_reject_constant,
                )
        self.verify()
        _require(
            isinstance(answer, dict)
            and answer.keys()
            == {
                "schema",
                "selected_id",
                "candidate_count",
                "candidate_pixels",
                "projection_uncertain",
            },
            "Invalid selector receipt",
        )
        _require(
            type(answer["schema"]) is int and answer["schema"] == 1,
            "Invalid selector schema",
        )
        for name, limit in (
            ("candidate_count", 31),
            ("candidate_pixels", frame["width"] * frame["height"]),
            ("projection_uncertain", 1),
        ):
            _require(
                type(answer[name]) is int and 0 <= answer[name] <= limit,
                "Invalid selector counters",
            )
        selected = answer["selected_id"]
        _require(isinstance(selected, str), "Invalid selector ID")
        if selected:
            _require(
                selected in {str(i) for i in self.metadata if i < owner},
                "Selector returned an unavailable group",
            )
            result["selected_ids"].insert(0, int(selected))
        _require(
            bool(selected)
            == bool(answer["candidate_count"])
            == bool(answer["candidate_pixels"]),
            "Inconsistent selector coverage",
        )
        _require(
            not answer["projection_uncertain"]
            or answer["candidate_pixels"] == frame["width"] * frame["height"],
            "Invalid uncertain projection",
        )
        result["neighbor_reason"] = "" if selected else "no_spatial_candidate"
        result["selection"] = {
            name: answer[name]
            for name in ("candidate_count", "candidate_pixels", "projection_uncertain")
        }
        result["selection"]["selection_seconds"] = time.perf_counter() - started
        return result
