"""One GPU working set, a confirmed CPU rollback state and immutable disk assets."""

import copy
import gc
import hashlib
import os
from pathlib import Path
import resource
import shutil
import stat

import numpy as np
import torch

from gaussian_state import read_state, write_state
from online_model import OnlineGaussianModel
from online_protocol import _validate_message, decode_target, decode_view
from submap_protocol import _same_pose
from train import _render_view, masked_losses
from worker_config import DEFAULT_SEED_POLICY, SEED_POLICIES


_MAX_ASSET_BYTES = (512 * 1024) + (1024 * 1024) + (192 * 1024 * 1024)
_IDENTITY = ("job_id", "generation", "pose_version", "submap_id")
_BINDING = ("generation", "pose_version", "submap_id", "membership_revision", "frame_count",
            "optimized_frame_count", "parent_frame", "parent_from_submap")


class GaussianCapacityError(RuntimeError):
    """A new working set cannot be represented within the fixed storage budget."""


def _content_key(content):
    return tuple(content[name] for name in ("submap_id", "source_generation", "content_job_id"))


def _body_poses(context):
    return {int(entry["keyframe_id"]): np.asarray(entry["submap_from_body"], dtype=np.float64).reshape(4, 4)
            for entry in context["primary"]}


def _compatible(state, target):
    metadata = state.metadata
    if (metadata["submap_id"] != target["submap_id"] or
            metadata["anchor_keyframe_id"] != target["anchor_keyframe_id"] or
            int(metadata["frame_count"]) > int(target["frame_count"])):
        return False
    poses = _body_poses(target)
    for entry in metadata["primary"]:
        key = int(entry["keyframe_id"])
        source = np.asarray(entry["submap_from_body"]).reshape(4, 4)
        if key not in poses or not _same_pose(source, poses[key]):
            return False
    return True


def _headers(model, target, refresh_cameras=False):
    poses = _body_poses(target) if refresh_cameras else None
    for view in model.views:
        view.update({name: copy.deepcopy(target[name]) for name in _BINDING})
        view["session_id"] = target["origin_session_id"]
        if refresh_cameras:
            # A new training operation uses the exact target cameras. Reuse
            # alone leaves canonical cameras and all stored arrays unchanged.
            camera = view["camera_from_body"] @ np.linalg.inv(poses[view["keyframe_id"]])
            view["camera_from_parent"] = camera
            view["camera_from_submap"] = camera.ravel().tolist()
    model.generation = int(target["generation"])


def _capture_context(model, target, split=None):
    context = copy.deepcopy(target)
    context['source_cameras'] = [dict(keyframe_id=str(view['keyframe_id']),
        camera_from_body=view['camera_from_body'].ravel().tolist()) for view in model.views]
    if split is not None:
        context['holdout'] = split.proof(model.training_lineage)
    return context


class GaussianSubmapWorker:
    """One serialized session; owner decisions are separate from durable references."""

    def __init__(self, session_id, state_directory, model, software, *, joint_render_enabled=False,
                 spatial_render_enabled=False, seed_policy=DEFAULT_SEED_POLICY, split=None):
        if seed_policy not in SEED_POLICIES:
            raise ValueError("Unknown Gaussian seed policy")
        if spatial_render_enabled and not joint_render_enabled:
            raise ValueError('Spatial rendering requires joint_render_v1')
        path = Path(state_directory)
        if path.name != "states" or not path.is_absolute() or os.path.lexists(path):
            raise ValueError("Expected a new absolute states directory")
        parent = path.parent.stat()
        if not stat.S_ISDIR(parent.st_mode) or parent.st_uid != os.getuid() or parent.st_mode & 0o002:
            raise ValueError("State parent must be owned by this user and not publicly writable")
        if model.parameters is not None or model.candidate_pending or model.views:
            raise ValueError("A fresh GPU model is required")
        path.mkdir(mode=0o700)
        if split is not None:
            with (path.parent / 'split.json').open('xb') as output:
                output.write(split.payload)
        self.session_id = session_id
        self.directory = path
        self.device = model.device
        self._model_options = model.options
        self.model = model
        self.software = dict(software)
        self.joint_render_enabled = joint_render_enabled
        self.spatial_render_enabled = spatial_render_enabled
        self._seed_policy = seed_policy
        self._split = split
        self.confirmed_state = None
        self.confirmed_metadata = None
        self.confirmed_binding = None
        self.content = None
        self.current_target = None
        self.pending = None
        self.assets = {}
        self.references = set()
        self.map_sequence = 0
        self.checkpoint_id = None
        self.pending_checkpoint = None
        self.failed = False
        self._known_submaps = set()
        self._last_job = 0
        self._last_asset = 0
        self._max_generation = -1
        self._max_version = 0
        self._max_frontier = 0

    def _reply(self, kind, request, **fields):
        result = {"schema": 2, "type": kind, "session_id": self.session_id}
        result.update({name: request[name] for name in _IDENTITY if name in request})
        if "operation" in request:
            result["operation"] = request["operation"]
        result.update(fields)
        return result

    def _durable(self):
        return (self.confirmed_state is None or
                self.checkpoint_id is not None and self.checkpoint_id in self.references)

    def _release_model(self):
        if self.model is not None:
            self.model.reset(self.model.generation or 0)
            self.model = None
        gc.collect()
        torch.cuda.synchronize(self.device)

    def _restore_confirmation(self):
        if self._split is not None and self.confirmed_state is not None:
            self._split.check_state(self.confirmed_state)
        self._release_model()
        if self.confirmed_state is None:
            self.model = OnlineGaussianModel(self.device, options=self._model_options)
            if self.current_target is not None:
                self.model.reset(int(self.current_target["generation"]))
        else:
            self.model = OnlineGaussianModel.from_state(self.confirmed_state, self.device,
                                                       options=self._model_options)
            self.current_target = copy.deepcopy(self.confirmed_binding)
            _headers(self.model, self.current_target)

    def _source(self, request):
        expected = {name: request[name] for name in ("submap_id", "source_generation", "content_job_id")}
        if "asset_id" not in request:
            if self.confirmed_state is None or _content_key(expected) != _content_key(self.content):
                raise ValueError("Resident content identity differs from the requested source")
            return self.confirmed_state, expected, self.checkpoint_id
        asset_id = int(request["asset_id"])
        description = self.assets.get(asset_id)
        if (description is None or _content_key(description) != _content_key(expected) or
                description["manifest_sha256"] != request["manifest_sha256"]):
            raise ValueError("Unknown asset or mismatched source identity")
        path = self.directory / str(asset_id)
        state = read_state(path)
        digest = hashlib.sha256((path / "manifest.json").read_bytes()).hexdigest()
        if (digest != description["manifest_sha256"] or state.metadata["origin_session_id"] != self.session_id or
                state.metadata["submap_id"] != expected["submap_id"] or
                state.metadata["generation"] != expected["source_generation"]):
            raise ValueError("Asset manifest differs from its confirmed reference")
        return state, expected, asset_id

    def _track_target(self, target, activate):
        generation = int(target["generation"])
        if (int(target["frame_count"]) < self._max_frontier or int(target["pose_version"]) < self._max_version or
                (activate and generation <= self._max_generation) or
                (not activate and generation != self._max_generation)):
            raise ValueError("Target frontier or generation moved backwards")
        self._max_generation = max(self._max_generation, generation)
        self._max_version = int(target["pose_version"])
        self._max_frontier = int(target["frame_count"])

    def _candidate(self, request, target, operation, preview, metrics):
        result = self._reply("candidate", request, operation=operation)
        if self.spatial_render_enabled:
            from spatial_metadata import extract_metadata
            if operation == 'reuse':
                state = self.pending['state']
                content = dict(session_id=self.session_id, **self.pending['content'])
            else:
                state = self.model.capture_candidate_state(_capture_context(self.model, target, self._split))
                self.pending['state'] = state
                content = dict(session_id=self.session_id, submap_id=request['submap_id'],
                               source_generation=request['generation'], content_job_id=request['job_id'])
            metadata = extract_metadata(state, content)
            self.pending['render_metadata'] = metadata
            result['render_metadata'] = copy.deepcopy(metadata)
        result.update({name: target[name] for name in
                       ("membership_revision", "frame_count", "optimized_frame_count")})
        result.update({name: metrics[name] for name in
                       ("steps", "gaussian_count", "supervised_depth_pixels", "rgb_l1", "depth_l1_m")})
        result.update(frame_ids=[str(key) for key in self.model.frame_ids], width=preview.shape[1],
                      height=preview.shape[0], capture_time=float(self.model.views[-1]["image_timestamp"]),
                      nonzero_gradient=int(metrics["nonzero_gradient"]), parameter_updated=int(metrics["parameter_updated"]),
                      gpu_peak_allocated_bytes=str(torch.cuda.max_memory_allocated(self.device)),
                      gpu_peak_reserved_bytes=str(torch.cuda.max_memory_reserved(self.device)),
                      worker_peak_rss_bytes=str(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024))
        return result, preview.tobytes()

    def _activate(self, request, body):
        target = decode_target(request, body)
        if self._split is not None:
            self._split.check_target(target)
        self._track_target(target, True)
        submap = target["submap_id"]
        if submap not in self._known_submaps and len(self._known_submaps) >= 32:
            raise GaussianCapacityError("All 32 Gaussian content slots are occupied")
        action = request["action"]
        if action == "empty":
            if submap in self._known_submaps:
                raise ValueError("Known content must be reused or rebuilt, not silently reset")
            if not self._durable():
                raise ValueError("Previous confirmation lacks a durable map reference")
            self._release_model()
            self.model = OnlineGaussianModel(self.device, options=self._model_options)
            self.model.reset(int(target["generation"]))
            self.current_target = target
            self.confirmed_state = self.confirmed_binding = self.content = None
            self.confirmed_metadata = None
            self.checkpoint_id = None
            return self._reply("activated", request), b""
        state, content, checkpoint = self._source(request)
        if self._split is not None:
            # Check before releasing the resident model or rebuilding seeds.
            self._split.check_state(state)
        if (state.metadata["anchor_keyframe_id"] != target["anchor_keyframe_id"] or
                not set(_body_poses(state.metadata)) <= _body_poses(target).keys()):
            raise ValueError("Target does not cover the complete source primary membership")
        compatible = _compatible(state, target)
        if action == "reuse" and not compatible:
            raise ValueError("Changed primary body poses require a rebuild")
        same = self.content is not None and _content_key(content) == _content_key(self.content)
        if (not same or action == "rebuild") and not self._durable():
            raise ValueError("Previous confirmation lacks a durable map reference")
        self.pending = {"request": dict(request), "target": target, "operation": action,
                        "state": state if action == "reuse" else None,
                        "content": content, "checkpoint_id": checkpoint}
        if action == "reuse":
            if not same:
                self._release_model()
                self.model = OnlineGaussianModel.from_state(state, self.device,
                                                           options=self._model_options)
            _headers(self.model, target)
            with torch.no_grad():
                view = self.model.views[-1]
                rgb, alpha, depth = _render_view(self.model.parameters, view)
                image_loss, depth_loss, _ = masked_losses(rgb, depth, alpha, view)
                preview = np.clip(np.rint(rgb.cpu().numpy() * 255), 0, 255).astype(np.uint8)
            metrics = {"steps": 0, "gaussian_count": int(self.model.origin_ids.numel()),
                       "supervised_depth_pixels": 0, "rgb_l1": image_loss.item(),
                       "depth_l1_m": depth_loss.item(), "nonzero_gradient": False, "parameter_updated": False}
        else:
            self._release_model()
            self.model = OnlineGaussianModel(self.device, options=self._model_options)
            self.model.reset(int(target["generation"]))
            poses = _body_poses(target)
            if self._split is not None:
                self.model.training_lineage = [int(item['keyframe_id'])
                                               for item in state.metadata['holdout']['lineage']]
            totals = {"supervised_depth_pixels": 0, "nonzero_gradient": False, "parameter_updated": False}
            for index, saved in enumerate(state.views):
                view = dict(saved)
                view.update({name: copy.deepcopy(target[name]) for name in _BINDING})
                view.update(session_id=self.session_id, split="train")
                camera = saved["camera_from_body"] @ np.linalg.inv(poses[saved["keyframe_id"]])
                view["camera_from_parent"] = camera
                view["camera_from_submap"] = camera.ravel().tolist()
                preview, metrics = self.model.train_candidate(view, seed_policy=self._seed_policy)
                totals["supervised_depth_pixels"] += metrics["supervised_depth_pixels"]
                totals["nonzero_gradient"] |= metrics["nonzero_gradient"]
                totals["parameter_updated"] |= metrics["parameter_updated"]
                if index + 1 < len(state.views):
                    self.model.decide(True)
            metrics.update(totals, steps=20 * len(state.views))
        self.current_target = target
        return self._candidate(request, target, action, preview, metrics)

    def _view(self, request, body):
        target = decode_target(request, body, 9 * request["width"] * request["height"])
        view = decode_view(request, body)
        if self._split is not None:
            self._split.check_target(target)
            self._split.check_view(view)
            for retained in self.model.views:
                self._split.check_view(retained)
            if self.model.training_lineage:
                self._split.check_proof(self._split.proof(self.model.training_lineage))
        if self.current_target is None or any(target[key] != self.current_target[key]
                                              for key in ("submap_id", "generation", "pose_version")):
            raise ValueError("View target has not been activated")
        self._track_target(target, False)
        if self.confirmed_state is not None and not _compatible(self.confirmed_state, target):
            raise ValueError("A changed primary member requires activation/rebuild first")
        _headers(self.model, target, refresh_cameras=True)
        preview, metrics = self.model.train_candidate(view, seed_policy=self._seed_policy)
        self.pending = {"request": dict(request), "target": target, "operation": "view"}
        self.current_target = target
        return self._candidate(request, target, "view", preview, metrics)

    def _decision(self, request):
        pending = self.pending
        if (pending is None or any(request[name] != pending["request"][name] for name in _IDENTITY) or
                request["operation"] != pending["operation"]):
            raise ValueError("Decision does not match the pending candidate")
        accept = request["action"] == "accept"
        if accept:
            if pending["operation"] == "reuse":
                self.confirmed_state = pending["state"]
                self.content = dict(pending["content"])
                self.checkpoint_id = pending["checkpoint_id"]
            else:
                self.model.decide(True)
                state = (pending['state'] if self.spatial_render_enabled else
                         self.model.capture_state(_capture_context(self.model, pending['target'], self._split)))
                if self._split is not None:
                    self._split.check_state(state)
                self.confirmed_state = state
                self.content = {"submap_id": request["submap_id"], "source_generation": request["generation"],
                                "content_job_id": request["job_id"]}
                self.checkpoint_id = None
            if self.spatial_render_enabled:
                self.confirmed_metadata = pending['render_metadata']
            self.confirmed_binding = copy.deepcopy(pending["target"])
            # Reuse can advance the binding while preserving this exact asset.
            if self.checkpoint_id is not None:
                self.assets[self.checkpoint_id]['binding_generation'] = pending['target']['generation']
            self._known_submaps.add(request["submap_id"])
            result = self._reply("commit_ok", request, **{name: self.content[name] for name in
                                                         ("source_generation", "content_job_id")},
                                 frame_ids=[str(key) for key in self.model.frame_ids],
                                 gaussian_count=int(self.model.origin_ids.numel()))
        else:
            self._restore_confirmation()
            result = self._reply("discard_ok", request)
        self.pending = None
        return result, b""

    def _checkpoint(self, request):
        if self.confirmed_state is None or _content_key(request) != _content_key(self.content):
            raise ValueError("Checkpoint does not name the confirmed content")
        asset_id = int(request["asset_id"])
        if (asset_id <= self._last_asset or self.pending_checkpoint is not None or
                set(self.assets) - self.references or len(self.assets) >= 33):
            raise ValueError("Checkpoint would overwrite content or create a second staging asset")
        if self.checkpoint_id is not None:
            raise ValueError("Confirmed content already has a checkpoint")
        if shutil.disk_usage(self.directory).free < 2**30 + _MAX_ASSET_BYTES:
            raise OSError("Insufficient disk reserve for a Gaussian asset")
        self._last_asset = asset_id
        path = self.directory / str(asset_id)
        write_state(path, self.confirmed_state)
        verified = read_state(path)
        description = {"submap_id": self.content["submap_id"],
                       "source_generation": self.content["source_generation"],
                       "content_job_id": self.content["content_job_id"], "asset_id": str(asset_id),
                       "manifest_sha256": hashlib.sha256((path / "manifest.json").read_bytes()).hexdigest(),
                       "manifest_bytes": str((path / "manifest.json").stat().st_size),
                       "gaussian_bytes": str((path / "gaussians.npz").stat().st_size),
                       "observation_bytes": str((path / "observations.npz").stat().st_size),
                       "frame_ids": [str(view["keyframe_id"]) for view in verified.views],
                       "gaussian_count": int(verified.origin_ids.size)}
        self.assets[asset_id] = dict(description)
        self.assets[asset_id]['binding_generation'] = self.confirmed_binding['generation']
        if self.spatial_render_enabled:
            # This cache follows the existing checkpoint/prune lifetime. The
            # persisted state schema and checkpoint reply stay unchanged.
            self.assets[asset_id]['render_metadata'] = copy.deepcopy(self.confirmed_metadata)
        self.checkpoint_id = asset_id
        self.pending_checkpoint = asset_id
        return self._reply("checkpoint_ok", request, **description), b""

    def _map_committed(self, request):
        sequence = int(request["map_sequence"])
        references = {int(value) for value in request["asset_ids"]}
        if (sequence <= self.map_sequence or not references <= self.assets.keys() or
                self.pending_checkpoint is not None and self.pending_checkpoint not in references):
            raise ValueError("Map acknowledgement does not reference known successful checkpoints")
        submaps = {self.assets[key]["submap_id"] for key in references}
        if len(submaps) != len(references):
            raise ValueError("A map can reference only one asset per submap")
        self.references = references
        self.map_sequence = sequence
        self.pending_checkpoint = None
        return self._reply("map_committed_ok", request, map_sequence=request["map_sequence"],
                           asset_ids=request["asset_ids"]), b""

    def _prune(self, request):
        asset_id = int(request["asset_id"])
        if (int(request["map_sequence"]) != self.map_sequence or asset_id not in self.assets or
                asset_id in self.references or asset_id == self.pending_checkpoint):
            raise ValueError("Prune requires a durable map that no longer references the asset")
        shutil.rmtree(self.directory / str(asset_id))
        del self.assets[asset_id]
        return self._reply("prune_ok", request, map_sequence=request["map_sequence"],
                           asset_id=request["asset_id"]), b""

    def handle(self, header, body):
        """Process one validated operation; any failure terminates this session."""
        if self.failed:
            raise RuntimeError("Gaussian submap session has failed")
        try:
            _validate_message(header, body)
            if header["schema"] != 2 or header["session_id"] != self.session_id:
                raise ValueError("Schema or session mismatch")
            kind = header["type"]
            if self.pending is not None:
                if kind != "decision":
                    raise ValueError("Expected the pending candidate decision")
                return self._decision(header)
            if kind == "decision" or int(header["job_id"]) <= self._last_job:
                raise ValueError("Operation job ID did not advance")
            self._last_job = int(header["job_id"])
            if kind == 'joint_render':
                if not self.joint_render_enabled:
                    raise ValueError('Joint render capability was not negotiated')
                if header.get('policy') == 'spatial' and not self.spatial_render_enabled:
                    raise ValueError('Spatial render capability was not negotiated')
                from online_render import render_operation
                return render_operation(self, header, body)
            if kind == "activate":
                return self._activate(header, body)
            if kind == "view":
                return self._view(header, body)
            if kind == "checkpoint":
                return self._checkpoint(header)
            if kind == "map_committed":
                return self._map_committed(header)
            if kind == "prune":
                return self._prune(header)
            if kind == "stop":
                incomplete = int(header.get("incomplete", 0))
                if not incomplete and (not self._durable() or self.pending_checkpoint is not None):
                    raise ValueError("Normal stop requires a durable confirmation")
                self._release_model()
                return self._reply("stopped", header, map_sequence=str(self.map_sequence), incomplete=incomplete), b""
            raise ValueError("Unexpected submap worker request")
        except Exception:
            self.failed = True
            raise
