"""Read the online worker's settings from the shared Gaussian YAML."""

from dataclasses import dataclass, fields
from pathlib import Path
import re
import sys


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "gaussian.yaml"
DEFAULT_TRANSFER_TIMEOUT_MS = 5000
DEFAULT_SEED_POLICY = "uniform"
SEED_POLICIES = (DEFAULT_SEED_POLICY, "coverage")


@dataclass(frozen=True)
class LearningRates:
    means: float = 0.0005
    log_scales: float = 0.002
    color_logits: float = 0.01
    opacity_logits: float = 0.01


@dataclass(frozen=True)
class WorkerOptions:
    device: str = "cuda:0"
    min_free_gpu_memory_mib: int = 4096
    gpu_memory_fraction: float = 0.4
    depth_loss_weight: float = 0.05
    learning_rates: LearningRates = LearningRates()
    coverage_uncovered_ratio: float = 0.5


DEFAULT_WORKER_OPTIONS = WorkerOptions()


@dataclass(frozen=True)
class WorkerConfig:
    socket_path: str
    transfer_timeout_ms: int
    seed_policy: str
    options: WorkerOptions


def _mapping(value, name):
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value


def _check_keys(mapping, allowed, name):
    unknown = mapping.keys() - allowed
    if unknown:
        raise ValueError(f"Unknown {name} settings: {', '.join(sorted(map(str, unknown)))}")


def _nonnegative_number(value, name):
    if type(value) not in (int, float) or not 0 <= value <= sys.float_info.max:
        raise ValueError(f"{name} must be a finite nonnegative number")
    return float(value)


def validate_coverage_uncovered_ratio(value):
    name = 'gaussian.worker.coverage_uncovered_ratio'
    ratio = _nonnegative_number(value, name)
    if ratio > 1:
        raise ValueError(f'{name} must be in [0, 1]')
    return ratio


def load_worker_config(path):
    try:
        import yaml
    except ImportError as error:
        raise ValueError("PyYAML is required to read the Gaussian worker config") from error

    try:
        document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid YAML in {path}: {error}") from error
    root = _mapping(document, "Gaussian config")
    gaussian = _mapping(root.get("gaussian"), "gaussian")
    socket_path = gaussian.get("socket_path", "")
    if not isinstance(socket_path, str):
        raise ValueError("gaussian.socket_path must be a string")
    timeout = gaussian.get("transfer_timeout_ms", DEFAULT_TRANSFER_TIMEOUT_MS)
    if type(timeout) is not int or not 1 <= timeout <= 2**31 - 1:
        raise ValueError("gaussian.transfer_timeout_ms must be a positive 32-bit integer")

    name = "gaussian.worker"
    worker = _mapping(gaussian.get("worker", {}), name)
    _check_keys(worker, {field.name for field in fields(WorkerOptions)} | {"seed_policy"}, name)
    seed_policy = worker.get("seed_policy", DEFAULT_SEED_POLICY)
    if seed_policy not in SEED_POLICIES:
        raise ValueError(f"{name}.seed_policy must be uniform or coverage")
    defaults = DEFAULT_WORKER_OPTIONS
    device = worker.get("device", defaults.device)
    if (not isinstance(device, str) or not re.fullmatch(r"cuda:(0|[1-9][0-9]{0,2})", device)
            or int(device[5:]) > 127):
        raise ValueError(f"{name}.device must be an explicit CUDA device such as cuda:0")
    free_memory = worker.get("min_free_gpu_memory_mib", defaults.min_free_gpu_memory_mib)
    if type(free_memory) is not int or not 0 <= free_memory <= 2**31 - 1:
        raise ValueError(f"{name}.min_free_gpu_memory_mib must be a nonnegative 32-bit integer")
    fraction = _nonnegative_number(worker.get("gpu_memory_fraction", defaults.gpu_memory_fraction),
                                   f"{name}.gpu_memory_fraction")
    if not 0 < fraction <= 1:
        raise ValueError(f"{name}.gpu_memory_fraction must be in (0, 1]")
    depth_weight = _nonnegative_number(worker.get("depth_loss_weight", defaults.depth_loss_weight),
                                       f"{name}.depth_loss_weight")
    uncovered_ratio = validate_coverage_uncovered_ratio(
        worker.get('coverage_uncovered_ratio', defaults.coverage_uncovered_ratio))
    rate_name = f"{name}.learning_rates"
    rates = _mapping(worker.get("learning_rates", {}), rate_name)
    rate_keys = [field.name for field in fields(LearningRates)]
    _check_keys(rates, set(rate_keys), rate_name)
    learning_rates = LearningRates(**{
        key: _nonnegative_number(rates.get(key, getattr(defaults.learning_rates, key)),
                                 f"{rate_name}.{key}")
        for key in rate_keys
    })
    return WorkerConfig(
        socket_path=socket_path, transfer_timeout_ms=timeout, seed_policy=seed_policy,
        options=WorkerOptions(
            device=device, min_free_gpu_memory_mib=free_memory, gpu_memory_fraction=fraction,
            depth_loss_weight=depth_weight, learning_rates=learning_rates,
            coverage_uncovered_ratio=uncovered_ratio,
        ),
    )
