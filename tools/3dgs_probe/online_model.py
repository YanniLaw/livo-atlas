"""Bounded incremental Gaussian model for one submap generation."""

import copy
import importlib
import importlib.metadata
import math
import sys
import time

import numpy as np
import torch

from gaussian_state import GaussianState, _metadata, _pose, _unsigned, validate_state
from train import (_ALPHA_THRESHOLD, _depth_candidates, _evenly_spaced, _render_view,
                   _replacement_seed_count, _reserved_seed_count,
                   create_parameters, initialize_gaussians, masked_losses)
from worker_config import DEFAULT_SEED_POLICY, DEFAULT_WORKER_OPTIONS, SEED_POLICIES

_MAX_VIEWS = 10
_MAX_PER_VIEW = 1000
_MAX_FRAMES = 512


def checked_cuda_device(options=DEFAULT_WORKER_OPTIONS):
    """Require an available GPU and a prebuilt gsplat extension."""
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required")
    device = torch.device(options.device)
    if (device.type != "cuda" or device.index is None or str(device) != options.device or
            device.index >= torch.cuda.device_count()):
        raise RuntimeError(f"CUDA device {options.device} is not available")
    free, _ = torch.cuda.mem_get_info(device)
    if free < options.min_free_gpu_memory_mib * 2**20:
        required_gib = options.min_free_gpu_memory_mib / 1024
        raise RuntimeError(f"At least {required_gib:g} GiB of free GPU memory is required")
    torch.cuda.set_per_process_memory_fraction(options.gpu_memory_fraction, device)
    try:
        importlib.import_module("gsplat.csrc")
    except (ImportError, OSError) as error:
        raise RuntimeError("A loadable prebuilt gsplat.csrc is required") from error
    return device


def _gpu_view(view, device):
    result = dict(view)
    for name in ("rgb", "image_mask", "depth_mask", "depth_m"):
        result[name] = torch.as_tensor(view[name], device=device)
    result["K"] = np.array(view["K"], dtype=np.float64, copy=True)
    result["camera_from_parent"] = np.array(view["camera_from_parent"],
                                            dtype=np.float64, copy=True)
    return result


class OnlineGaussianModel:
    """One fixed-size model; an unaccepted candidate is never reused."""

    def __init__(self, device, *, options=DEFAULT_WORKER_OPTIONS):
        selected_device = checked_cuda_device(options)
        if torch.device(device) != selected_device:
            raise ValueError("The model must use the selected CUDA device")
        self.device = selected_device
        self.options = options
        self.generation = None
        self.views = []
        self.training_lineage = []
        self.parameters = None
        self.origin_ids = None
        self.optimizer = None
        self.candidate_pending = False

    @property
    def frame_ids(self):
        return [view["keyframe_id"] for view in self.views]

    def reset(self, generation):
        if type(generation) is not int or not 0 <= generation <= 2**64 - 1:
            raise ValueError("Invalid model generation")
        self.generation = generation
        self.views = []
        self.training_lineage = []
        self.parameters = None
        self.origin_ids = None
        self.optimizer = None
        self.candidate_pending = False

    def capture_state(self, context):
        """Copy a confirmed working set using independently supplied source poses."""
        return self._capture_state(context, pending=False)

    def capture_candidate_state(self, context):
        """Own one tentative CPU copy without accepting or changing the model."""
        return self._capture_state(context, pending=True)

    def _capture_state(self, context, *, pending):
        if self.parameters is None or self.candidate_pending != pending or not self.views:
            kind = 'pending' if pending else 'confirmed'
            raise ValueError('Only a nonempty ' + kind + ' model can be captured')
        generated = {"schema", "kind", "representation", "optimizer", "software"}
        if not isinstance(context, dict) or generated & context.keys():
            raise ValueError("Capture context cannot override generated metadata")
        metadata = copy.deepcopy({key: value for key, value in context.items()
                                  if key != "source_cameras"})
        metadata.update(schema=2 if 'holdout' in context else 1, kind="livo_atlas.gaussian_state",
                        representation="gsplat-sh0-window-v1", optimizer="reset_on_restore",
                        software={"python": sys.version.split()[0], "numpy": np.__version__,
                                  "torch": str(torch.__version__), "cuda": torch.version.cuda or "unknown",
                                  "gsplat": importlib.metadata.version("gsplat")})
        _metadata(metadata)
        if metadata["generation"] != str(self.generation):
            raise ValueError("Capture generation differs from the model")
        sources = context.get("source_cameras")
        if (not isinstance(sources, list) or len(sources) != len(self.views) or
                any(not isinstance(item, dict) or set(item) != {"keyframe_id", "camera_from_body"}
                    for item in sources) or
                [item["keyframe_id"] for item in sources] != [str(key) for key in self.frame_ids]):
            raise ValueError("Independent source cameras must cover the retained views")
        views = []
        for view, source in zip(self.views, sources):
            for field, key in (("session_id", "origin_session_id"), ("submap_id", "submap_id"),
                               ("generation", "generation"), ("pose_version", "pose_version"),
                               ("parent_frame", "parent_frame"),
                               ("optimized_frame_count", "optimized_frame_count")):
                if view.get(field) != metadata[key]:
                    raise ValueError("Retained view identity differs from capture context: " + field)
            frontier = _unsigned(view.get("frame_count"))
            revision = _unsigned(view.get("membership_revision"))
            if (not view["keyframe_id"] < frontier <= int(metadata["frame_count"]) or
                    frontier < int(metadata["optimized_frame_count"]) or
                    not 1 <= revision <= int(metadata["membership_revision"])):
                raise ValueError("Retained view has an invalid capture frontier")
            if not np.allclose(_pose(view.get("parent_from_submap")),
                               _pose(metadata["parent_from_submap"]), atol=1e-8, rtol=0):
                raise ValueError("Retained view uses a different body anchor")
            camera = _pose(view["camera_from_parent"])
            if "camera_from_submap" in view and not np.allclose(
                    camera, _pose(view["camera_from_submap"]), atol=1e-8, rtol=0):
                raise ValueError("Retained camera headers disagree")
            saved = {name: view[name] for name in ("keyframe_id", "calibration_id", "state_timestamp",
                     "cloud_timestamp", "image_timestamp", "inverse_exposure_time")}
            saved.update(K=view["K"].copy(), camera_from_parent=camera,
                         camera_from_body=_pose(source["camera_from_body"]))
            for name in ("rgb", "image_mask", "depth_mask", "depth_m"):
                saved[name] = view[name].detach().cpu().numpy().copy()
            views.append(saved)
        state = GaussianState(metadata,
                              {key: value.detach().cpu().numpy().copy()
                               for key, value in self.parameters.items()},
                              self.origin_ids.detach().cpu().numpy().astype("<u8"), views)
        validate_state(state)
        return state

    @classmethod
    def from_state(cls, state, device, *, options=DEFAULT_WORKER_OPTIONS):
        """Restore raw parameters into a new model, leaving Adam uninitialized."""
        validate_state(state)
        model = cls(device, options=options)
        metadata = state.metadata
        model.generation = int(metadata["generation"])
        model.training_lineage = [int(item['keyframe_id']) for item in
                                  metadata.get('holdout', {}).get('lineage', [])]
        model.parameters = {}
        for name, value in state.parameters.items():
            tensor = torch.as_tensor(value.copy(), dtype=torch.float32, device=model.device)
            model.parameters[name] = tensor if name == "quats" else torch.nn.Parameter(tensor)
        model.origin_ids = torch.as_tensor(state.origin_ids.astype(np.int64), device=model.device)
        for saved in state.views:
            view = dict(saved)
            view["camera_from_body"] = saved["camera_from_body"].copy()
            for key in ("submap_id", "generation", "pose_version", "parent_frame", "frame_count",
                        "optimized_frame_count", "membership_revision"):
                view[key] = metadata[key]
            view.update(session_id=metadata["origin_session_id"], split="train",
                        parent_from_submap=copy.deepcopy(metadata["parent_from_submap"]),
                        camera_from_submap=saved["camera_from_parent"].ravel().tolist())
            model.views.append(_gpu_view(view, model.device))
        return model

    def render_preview(self, keyframe_id):
        """Render an owned RGB8 preview of a retained, confirmed observation."""
        if self.candidate_pending or self.parameters is None:
            raise ValueError("A confirmed model is required")
        view = next((item for item in self.views if item["keyframe_id"] == keyframe_id), None)
        if view is None:
            raise ValueError("Preview frame is not retained")
        with torch.no_grad():
            rgb, _, _ = _render_view(self.parameters, view)
            return np.clip(np.rint(rgb.cpu().numpy() * 255), 0, 255).astype(np.uint8)

    def _append(self, view, initial):
        next_parameters = create_parameters(initial, self.device)
        count = len(initial["means"])
        next_origins = torch.full((count,), view["keyframe_id"],
                                  device=self.device, dtype=torch.int64)
        if self.parameters is None:
            self.parameters = next_parameters
            self.origin_ids = next_origins
        else:
            keep = torch.ones_like(self.origin_ids, dtype=torch.bool)
            if len(self.views) == _MAX_VIEWS:
                oldest = self.views.pop(0)["keyframe_id"]
                keep = self.origin_ids != oldest
            combined = {}
            for name, old in self.parameters.items():
                value = torch.cat((old.detach()[keep], next_parameters[name].detach()))
                combined[name] = value if name == "quats" else torch.nn.Parameter(value)
            self.parameters = combined
            self.origin_ids = torch.cat((self.origin_ids[keep], next_origins))
        self.views.append(_gpu_view(view, self.device))
        rates = self.options.learning_rates
        self.optimizer = torch.optim.Adam([
            {"params": [self.parameters["means"]], "lr": rates.means},
            {"params": [self.parameters["log_scales"]], "lr": rates.log_scales},
            {"params": [self.parameters["color_logits"]], "lr": rates.color_logits},
            {"params": [self.parameters["opacity_logits"]], "lr": rates.opacity_logits},
        ])

    def _coverage_prepass(self, view):
        candidates = _depth_candidates(view)
        count = int(candidates.sum())
        evidence = dict(prepass_seconds=None, depth_candidates=count,
                        uncovered_candidates=None, reserved_uncovered=0)
        if self.options.coverage_sampling == 'replace':
            evidence.update(uniform_uncovered=None, replaced_seeds=0)
        if self.options.coverage_uncovered_ratio == 0 or self.parameters is None or count <= _MAX_PER_VIEW:
            return None, evidence

        started = time.perf_counter()
        with torch.no_grad():
            parameters = self.parameters
            if len(self.views) == _MAX_VIEWS:
                # Measure coverage from the Gaussians that will survive insertion.
                keep = self.origin_ids != self.views[0]["keyframe_id"]
                parameters = {name: value[keep] for name, value in parameters.items()}
            _, alpha, _ = _render_view(parameters, view)
            alpha = alpha.cpu().numpy()
        evidence["prepass_seconds"] = time.perf_counter() - started
        if alpha.shape != candidates.shape or not np.isfinite(alpha).all():
            raise RuntimeError("Coverage prepass produced invalid alpha")
        uncovered = candidates & (alpha < _ALPHA_THRESHOLD)
        holes = int(uncovered.sum())
        evidence['uncovered_candidates'] = holes
        if self.options.coverage_sampling == 'replace':
            uniform = _evenly_spaced(np.flatnonzero(candidates), min(count, _MAX_PER_VIEW))
            uniform_holes = int(uncovered.ravel()[uniform].sum())
            evidence.update(uniform_uncovered=uniform_holes,
                replaced_seeds=_replacement_seed_count(count, holes, uniform_holes,
                    _MAX_PER_VIEW, self.options.coverage_uncovered_ratio))
        else:
            evidence['reserved_uncovered'] = _reserved_seed_count(
                count, holes, _MAX_PER_VIEW, self.options.coverage_uncovered_ratio)
        return uncovered, evidence

    def train_candidate(self, view, *, seed_policy=DEFAULT_SEED_POLICY):
        if self.generation is None or self.candidate_pending:
            raise RuntimeError("Model must be reset and have no pending candidate")
        frame_id = view.get("keyframe_id")
        if (type(frame_id) is not int or not 0 <= frame_id < _MAX_FRAMES or
                (self.views and frame_id <= self.frame_ids[-1]) or
                view.get("generation") != str(self.generation)):
            raise ValueError("View identity or generation is invalid")
        if seed_policy not in SEED_POLICIES:
            raise ValueError("Unknown Gaussian seed policy")
        uncovered, selection = None, None
        if seed_policy == "coverage":
            if view["split"] != "train":
                raise ValueError("Coverage prepass requires a training view")
            uncovered, selection = self._coverage_prepass(view)
        # Initialization validates that depth exists before the model mutates.
        initial = initialize_gaussians([view], _MAX_PER_VIEW, uncovered_mask=uncovered,
                                      uncovered_ratio=self.options.coverage_uncovered_ratio,
                                      coverage_sampling=self.options.coverage_sampling)
        try:
            self._append(view, initial)
            before = {name: value.detach().clone()
                      for name, value in self.parameters.items() if name != "quats"}
            by_id = {item["keyframe_id"]: item for item in self.views}
            other_ids = sorted(key for key in by_id if key != frame_id)
            schedule = [frame_id] * 10
            schedule += ([other_ids[i % len(other_ids)] for i in range(10)]
                         if other_ids else [frame_id] * 10)
            depth_samples = 0
            nonzero_gradient = False
            for selected_id in schedule:
                self.optimizer.zero_grad(set_to_none=True)
                rgb, alpha, depth = _render_view(self.parameters, by_id[selected_id])
                image_loss, depth_loss, depth_count = masked_losses(
                    rgb, depth, alpha, by_id[selected_id]
                )
                loss = image_loss + self.options.depth_loss_weight * depth_loss
                if not torch.isfinite(loss).item():
                    raise RuntimeError("Training loss is nonfinite")
                loss.backward()
                for name, parameter in self.parameters.items():
                    if name == "quats":
                        continue
                    if (parameter.grad is None or
                            not torch.isfinite(parameter.grad).all().item()):
                        raise RuntimeError("Training gradient is missing or nonfinite: " + name)
                    nonzero_gradient |= bool(parameter.grad.any().item())
                self.optimizer.step()
                with torch.no_grad():
                    self.parameters["log_scales"].clamp_(math.log(0.001), math.log(0.2))
                if any(not torch.isfinite(value).all().item()
                       for value in self.parameters.values()):
                    raise RuntimeError("Updated Gaussian parameter is nonfinite")
                depth_samples += depth_count
            with torch.no_grad():
                rgb, alpha, depth = _render_view(self.parameters, by_id[frame_id])
                image_loss, depth_loss, _ = masked_losses(rgb, depth, alpha, by_id[frame_id])
                preview = np.clip(np.rint(rgb.cpu().numpy() * 255), 0, 255).astype(np.uint8)
            updated = any(not torch.equal(self.parameters[name].detach(), value)
                          for name, value in before.items())
            metrics = {
                "frame_ids": self.frame_ids,
                "steps": len(schedule),
                "training_frame_ids": schedule,
                "rgb_l1": image_loss.item(),
                "depth_l1_m": depth_loss.item(),
                "supervised_depth_pixels": depth_samples,
                "gaussian_count": int(self.origin_ids.numel()),
                "nonzero_gradient": nonzero_gradient,
                "parameter_updated": updated,
            }
            if selection is not None:
                metrics["seed_selection"] = selection
            if (not math.isfinite(metrics["rgb_l1"]) or
                    not math.isfinite(metrics["depth_l1_m"])):
                raise RuntimeError("Evaluation metric is nonfinite")
            self.candidate_pending = True
            # Window eviction removes observations/Gaussians, but cannot undo
            # their past gradients. Keep every seed and actual loss source.
            self.training_lineage = sorted(set(self.training_lineage) | set(schedule) | {frame_id})
            return preview, metrics
        except Exception:
            self.reset(self.generation)
            raise

    def decide(self, accept):
        if type(accept) is not bool or not self.candidate_pending:
            raise RuntimeError("No candidate is awaiting a decision")
        if accept:
            self.candidate_pending = False
        else:
            self.reset(self.generation)
