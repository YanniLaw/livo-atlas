#!/usr/bin/env python3
"""Validate saved real observations with fresh GPU models and controlled targets.

This is an explicit GPU validation program. Its pose corrections and capacity
sequence are synthetic; the RGB/depth/camera observations come from saved assets.
It creates a separate output and never changes the input map or its states.
"""

import argparse
import copy
import gc
import hashlib
import json
import os
from pathlib import Path
import resource
import uuid
from unittest import mock

import numpy as np
from PIL import Image
import torch

import gaussian_state
from gaussian_state import CommitUncertainError, read_state
from online_model import OnlineGaussianModel, checked_cuda_device
from online_worker import create_model
from submap_worker import GaussianCapacityError, GaussianSubmapWorker
from train import _render_view


_TARGET = ("submap_id", "anchor_keyframe_id", "membership_revision", "frame_count",
           "optimized_frame_count", "pose_version", "parent_frame", "submap_state",
           "parent_from_submap")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parameter_hashes(model):
    return {key: hashlib.sha256(value.detach().cpu().numpy().tobytes()).hexdigest()
            for key, value in model.parameters.items()}


def render(model):
    with torch.no_grad():
        return [value.cpu().numpy().copy() for value in
                _render_view(model.parameters, model.views[-1])]


def same_render(expected, model):
    actual = render(model)
    for i, (before, after) in enumerate(zip(expected, actual)):
        np.testing.assert_allclose(before, after, atol=1e-5 if i == 2 else 1e-6,
                                   rtol=1e-6 if i == 2 else 0, equal_nan=False)
    return [float(np.max(np.abs(a-b))) for a, b in zip(expected, actual)]


def source_context(metadata, frontier):
    result = {key: copy.deepcopy(metadata[key]) for key in _TARGET}
    result.update(frame_count=str(frontier), optimized_frame_count="0",
                  pose_version="0", parent_frame="odom")
    result["primary"] = copy.deepcopy(metadata["primary"])
    result["primary_count"] = len(result["primary"])
    return result


class ControlledOwner:
    """One public worker, monotonic requests and a durable reference journal."""

    def __init__(self, output):
        self.output = output
        output.mkdir(mode=0o700)
        model, software = create_model()
        self.session = uuid.uuid4().hex
        self.actor = GaussianSubmapWorker(self.session, output / "states", model, software)
        self.job = self.generation = self.sequence = self.asset = 0
        self.references = {}
        self.target = None

    def request(self, kind, scoped=True, **fields):
        self.job += 1
        header = {"schema": 2, "type": kind, "session_id": self.session,
                  "job_id": str(self.job), "generation": str(self.generation),
                  "pose_version": self.target["pose_version"] if self.target else "0"}
        if scoped:
            header["submap_id"] = self.target["submap_id"]
        header.update(fields)
        return header

    @staticmethod
    def primary(target):
        return np.asarray([row["submap_from_body"] for row in target["primary"]], dtype="<f8").tobytes()

    def decide(self, candidate, accept=True):
        request = {key: candidate[key] for key in
                   ("schema", "session_id", "job_id", "generation", "pose_version", "submap_id", "operation")}
        request.update(type="decision", action="accept" if accept else "discard")
        reply, _ = self.actor.handle(request, b"")
        require(reply["type"] == ("commit_ok" if accept else "discard_ok"), "Decision failed")

    def activate(self, target, action="empty", asset=None, accept=True):
        previous = self.target
        self.generation += 1
        proposed = copy.deepcopy(target)
        proposed["generation"] = str(self.generation)
        self.target = proposed
        header = self.request("activate", **{key: proposed[key] for key in _TARGET},
                              primary_count=proposed["primary_count"], action=action)
        if action != "empty":
            source = asset if asset else self.actor.content
            header.update({key: source[key] for key in ("source_generation", "content_job_id")})
            if asset:
                header.update(asset_id=asset["asset_id"], manifest_sha256=asset["manifest_sha256"])
        candidate, _ = self.actor.handle(header, self.primary(proposed))
        if action == "empty":
            require(candidate["type"] == "activated", "Empty activation failed")
        else:
            self.decide(candidate, accept)
            if not accept:
                self.target = previous
        return candidate

    def view(self, observation, accept=True):
        target = self.target
        frame = int(observation["keyframe_id"])
        local = np.asarray(target["primary"][frame-int(target["anchor_keyframe_id"])]
                           ["submap_from_body"]).reshape(4, 4)
        height, width = observation["depth_m"].shape
        camera = observation["camera_from_body"]
        intrinsic = observation["K"]
        header = self.request("view", **{key: target[key] for key in _TARGET},
            primary_count=target["primary_count"], keyframe_id=str(frame), calibration_id="0",
            width=width, height=height, fx=float(intrinsic[0, 0]), fy=float(intrinsic[1, 1]),
            cx=float(intrinsic[0, 2]), cy=float(intrinsic[1, 2]),
            camera_from_body=camera.ravel().tolist(),
            camera_from_submap=(camera @ np.linalg.inv(local)).ravel().tolist(),
            **{key: observation[key] for key in
               ("state_timestamp", "cloud_timestamp", "image_timestamp", "inverse_exposure_time")})
        rgb = np.clip(np.rint(observation["rgb"] * 255), 0, 255).astype("u1")
        body = (rgb[..., ::-1].tobytes() + observation["image_mask"].astype("u1").tobytes() +
                observation["depth_mask"].astype("u1").tobytes() +
                observation["depth_m"].astype("<f4").tobytes() + self.primary(target))
        candidate, _ = self.actor.handle(header, body)
        require(candidate["steps"] == 20 and candidate["nonzero_gradient"] and
                candidate["parameter_updated"], "Real observation did not train")
        self.decide(candidate, accept)
        return candidate

    def checkpoint(self):
        self.asset += 1
        reply, _ = self.actor.handle(self.request("checkpoint", asset_id=str(self.asset),
            **{key: self.actor.content[key] for key in ("source_generation", "content_job_id")}), b"")
        read_state(self.output / "states" / reply["asset_id"])
        self.references[reply["submap_id"]] = reply["asset_id"]
        self.sequence += 1
        ids = sorted(self.references.values(), key=int)
        temporary = self.output / "references.tmp.json"
        # This controlled owner journals complete references before its ack.
        # Product map.json publication is separately verified by the C++ tests.
        with temporary.open("xb") as stream:
            stream.write(json.dumps({"sequence": self.sequence, "asset_ids": ids}).encode())
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.output / "references.json")
        gaussian_state._sync_directory(self.output)
        response, _ = self.actor.handle(self.request("map_committed", scoped=False,
            map_sequence=str(self.sequence), asset_ids=ids), b"")
        require(response["type"] == "map_committed_ok", "Durable reference ack failed")
        return reply

    def prune(self, asset):
        reply, _ = self.actor.handle(self.request("prune", scoped=False,
            map_sequence=str(self.sequence), asset_id=asset["asset_id"]), b"")
        require(reply["type"] == "prune_ok", "Prune failed")


def cold_restore(directory, index, device):
    result = {"assets": 0, "matched_previews": 0, "max_rgb8_difference": 0}
    previews = [(path, json.loads(path.read_text())) for path in directory.glob("preview-*.json")]
    for entry in index["entries"]:
        if "asset" not in entry:
            continue
        reference = entry["asset"]
        state = read_state(directory / "states" / reference["asset_id"])
        model = OnlineGaussianModel.from_state(state, device)
        for key, values in state.parameters.items():
            np.testing.assert_array_equal(model.parameters[key].detach().cpu().numpy(), values)
        np.testing.assert_array_equal(model.origin_ids.cpu().numpy(), state.origin_ids)
        rendered = render(model)
        require(all(np.isfinite(values).all() for values in rendered), "Cold rendering is nonfinite")
        rgb8 = np.clip(np.rint(rendered[0] * 255), 0, 255).astype("u1")
        for path, metadata in previews:
            if metadata["submap_id"] != entry["submap_id"]:
                continue
            matches = (metadata["job_id"] == reference["content_job_id"] or
                       metadata.get("operation") == "reuse" and metadata["job_id"] == entry["binding_job_id"])
            if not matches:
                continue
            image = np.asarray(Image.open(path.with_suffix(".png"))).astype(np.int16)
            require(image.shape == rgb8.shape, "Cold preview dimensions differ")
            difference = int(np.max(np.abs(image - rgb8.astype(np.int16))))
            require(difference <= 1, "Cold preview differs from the saved confirmation")
            result["matched_previews"] += 1
            result["max_rgb8_difference"] = max(result["max_rgb8_difference"], difference)
        result["assets"] += 1
        model.reset(0)
        del model, state, rendered
        gc.collect()
        torch.cuda.empty_cache()
    require(result["assets"] >= 2 and result["matched_previews"] >= 1,
            "Cold restore needs historical blocks and a matching saved preview")
    return result


def exercise(directory, index, output):
    entries = [entry for entry in index["entries"] if "asset" in entry][:2]
    require(len(entries) == 2, "Two saved real submaps are required")
    owner = ControlledOwner(output / "lifecycle")
    contexts, checkpoints = [], []
    for entry in entries:
        state = read_state(directory / "states" / entry["asset"]["asset_id"])
        context = source_context(state.metadata, int(index["frame_count"]))
        contexts.append(context)
        owner.activate(context)
        views = state.views if len(contexts) == 1 else state.views[:-1]
        require(len(views) >= 2, "Need multiple real observations in each block")
        for observation in views:
            owner.view(observation)
        checkpoints.append(owner.checkpoint())
        if len(contexts) == 2:
            last_view = copy.deepcopy(state.views[-1])
        del state
    before = parameter_hashes(owner.actor.model)
    renders = render(owner.actor.model)
    owner.view(last_view, accept=False)
    require(parameter_hashes(owner.actor.model) == before, "View rejection changed confirmation")
    rejected_view = same_render(renders, owner.actor.model)
    owner.activate(contexts[0], action="reuse", asset=checkpoints[0], accept=False)
    require(parameter_hashes(owner.actor.model) == before, "Cross-block rejection restored the wrong block")
    rejected_block = same_render(renders, owner.actor.model)
    require(owner.actor.confirmed_binding["generation"] == "2", "Rollback lost the accepted binding")
    rigid = copy.deepcopy(contexts[1])
    rigid.update(pose_version="1", optimized_frame_count=index["frame_count"], parent_frame="map")
    parent = np.asarray(rigid["parent_from_submap"]).reshape(4, 4)
    parent[:3, 3] += [1, -2, .5]
    rigid["parent_from_submap"] = parent.ravel().tolist()
    pointers = [value.data_ptr() for value in owner.actor.model.parameters.values()]
    original_hash = digest(owner.output / "states/2/manifest.json")
    reused = owner.activate(rigid, "reuse")
    require(reused["steps"] == 0 and parameter_hashes(owner.actor.model) == before and
            pointers == [value.data_ptr() for value in owner.actor.model.parameters.values()] and
            digest(owner.output / "states/2/manifest.json") == original_hash,
            "Rigid reuse changed parameters or rewrote the asset")
    changed = copy.deepcopy(rigid)
    retained = set(owner.actor.model.frame_ids)
    outside = next(row for row in changed["primary"][1:] if int(row["keyframe_id"]) not in retained)
    outside["submap_from_body"][3] += .05
    changed["pose_version"] = "4"
    rebuilt = owner.activate(changed, "rebuild")
    require(rebuilt["steps"] == 20 * len(retained), "Rebuild did not train every retained view")
    rebuilt_asset = owner.checkpoint()
    owner.prune(checkpoints[1])
    require(not (owner.output / "states/2").exists() and (owner.output / "states/1").is_dir(),
            "Prune changed a still-referenced asset")
    owner.view(last_view)
    sync = gaussian_state._sync_directory

    def fail_after_publish(path):
        if Path(path) == owner.output / "states/4":
            raise OSError("controlled directory-sync failure")
        return sync(path)

    with mock.patch.object(gaussian_state, "_sync_directory", side_effect=fail_after_publish):
        try:
            owner.checkpoint()
            raise RuntimeError("Uncertain commit was incorrectly accepted")
        except CommitUncertainError:
            pass
    read_state(owner.output / "states/4")
    try:
        owner.prune(rebuilt_asset)
        raise AssertionError("Failed worker continued pruning")
    except RuntimeError as error:
        require("failed" in str(error), "Unexpected post-failure error")
    require(all((owner.output / "states" / str(value)).is_dir() for value in (1, 3, 4)),
            "Uncertain commit did not preserve old and new assets")
    del owner, renders, last_view
    gc.collect()
    torch.cuda.empty_cache()

    capacity = ControlledOwner(output / "capacity")
    state = read_state(directory / "states" / entries[0]["asset"]["asset_id"])
    observation = copy.deepcopy(state.views[0])
    del state
    for i in range(32):
        target = copy.deepcopy(contexts[0])
        target.update(submap_id=str(i), anchor_keyframe_id=str(i), membership_revision="1",
                      frame_count=str(i+1), primary_count=1, submap_state="active",
                      primary=[{"keyframe_id": str(i), "submap_from_body": np.eye(4).ravel().tolist()}])
        capacity.activate(target)
        observation["keyframe_id"] = i
        capacity.view(observation)
        capacity.checkpoint()
    hashes = parameter_hashes(capacity.actor.model)
    target = copy.deepcopy(target)
    target.update(submap_id="32", anchor_keyframe_id="32", frame_count="33",
                  primary=[{"keyframe_id": "32", "submap_from_body": np.eye(4).ravel().tolist()}])
    try:
        capacity.activate(target)
        raise RuntimeError("A 33rd Gaussian content slot was allocated")
    except GaussianCapacityError:
        pass
    require(len(list((capacity.output / "states").iterdir())) == 32 and
            parameter_hashes(capacity.actor.model) == hashes,
            "Capacity rejection released or replaced existing content")
    del capacity
    gc.collect()
    torch.cuda.empty_cache()
    return {"rejected_view_render_differences": rejected_view,
            "rejected_activation_render_differences": rejected_block,
            "rigid_steps": reused["steps"], "rebuild_steps": rebuilt["steps"],
            "changed_nonwindow_frame": outside["keyframe_id"], "pose_version_jump": [1, 4],
            "capacity_assets": 32, "rejected_33rd": True, "uncertain_assets_preserved": [1, 3, 4],
            "capacity_input": "One real observation repeated with synthetic frame/submap IDs"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-directory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.map_directory = args.map_directory.resolve()
    output = args.output.resolve()
    require(output != args.map_directory and args.map_directory not in output.parents and
            output not in args.map_directory.parents, "Output must be separate from the input map")
    output.mkdir(mode=0o700)
    index = json.loads((args.map_directory / "map.json").read_text())
    device = checked_cuda_device()
    torch.set_num_threads(1)
    torch.cuda.reset_peak_memory_stats()
    source_hashes = {row["asset"]["asset_id"]: digest(args.map_directory / "states" /
                     row["asset"]["asset_id"] / "manifest.json") for row in index["entries"] if "asset" in row}
    report = {"cold_restore": cold_restore(args.map_directory, index, device),
              "controlled_real_observations": exercise(args.map_directory, index, output),
              "source_manifest_hashes": source_hashes}
    require(source_hashes == {key: digest(args.map_directory / "states" / key / "manifest.json")
                              for key in source_hashes}, "Input assets were modified")
    report.update(gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated(),
                  gpu_peak_reserved_bytes=torch.cuda.max_memory_reserved(),
                  peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024)
    (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
