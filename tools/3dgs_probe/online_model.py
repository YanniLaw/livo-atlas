"""Bounded incremental Gaussian model for one submap generation."""

import importlib
import math

import numpy as np
import torch

from train import (_render_view, create_parameters, initialize_gaussians,
                   masked_losses)

_MAX_VIEWS = 10
_MAX_PER_VIEW = 1000
_MAX_FRAMES = 512


def checked_cuda_device():
    """Require an available GPU and a prebuilt gsplat extension."""
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required")
    device = torch.device("cuda:0")
    free, _ = torch.cuda.mem_get_info(device)
    if free < 4 * 2**30:
        raise RuntimeError("At least 4 GiB of free GPU memory is required")
    torch.cuda.set_per_process_memory_fraction(0.4, device)
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

    def __init__(self, device):
        if torch.device(device) != checked_cuda_device():
            raise ValueError("The model must use the selected CUDA device")
        self.device = torch.device(device)
        self.generation = None
        self.views = []
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
        self.parameters = None
        self.origin_ids = None
        self.optimizer = None
        self.candidate_pending = False

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
        self.optimizer = torch.optim.Adam([
            {"params": [self.parameters["means"]], "lr": 5e-4},
            {"params": [self.parameters["log_scales"]], "lr": 2e-3},
            {"params": [self.parameters["color_logits"]], "lr": 1e-2},
            {"params": [self.parameters["opacity_logits"]], "lr": 1e-2},
        ])

    def train_candidate(self, view):
        if self.generation is None or self.candidate_pending:
            raise RuntimeError("Model must be reset and have no pending candidate")
        frame_id = view.get("keyframe_id")
        if (type(frame_id) is not int or not 0 <= frame_id < _MAX_FRAMES or
                (self.views and frame_id <= self.frame_ids[-1]) or
                view.get("generation") != str(self.generation)):
            raise ValueError("View identity or generation is invalid")
        # Initialization validates that depth exists before the model mutates.
        initial = initialize_gaussians([view], _MAX_PER_VIEW)
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
                loss = image_loss + 0.05 * depth_loss
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
            if (not math.isfinite(metrics["rgb_l1"]) or
                    not math.isfinite(metrics["depth_l1_m"])):
                raise RuntimeError("Evaluation metric is nonfinite")
            self.candidate_pending = True
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
