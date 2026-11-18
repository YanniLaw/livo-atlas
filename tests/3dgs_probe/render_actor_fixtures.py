"""Provision real confirmed assets through the existing worker commands."""
import copy
import dataclasses
import sys

import numpy as np
import torch

from online_model import OnlineGaussianModel
from render_fixtures import ASSET_FIELDS, TARGET_FIELDS
from state_fixtures import pose
from submap_fixtures import SESSION, control_fixture, target_fixture, view_v2_fixture
from submap_worker import GaussianSubmapWorker


METADATA = dict(python=sys.version.split()[0], torch=str(torch.__version__),
                gsplat='1.5.3', cuda='12.1', device='test GPU',
                gpu_free_bytes='5000000000', gpu_total_bytes='8000000000')


def freeze(value):
    if isinstance(value, torch.Tensor):
        array = value.detach().cpu().numpy()
        return (str(array.dtype), array.shape, array.tobytes())
    if isinstance(value, np.ndarray):
        return (str(value.dtype), value.shape, value.tobytes())
    if dataclasses.is_dataclass(value):
        return freeze(vars(value))
    if isinstance(value, dict):
        return tuple((key, freeze(item)) for key, item in sorted(value.items(), key=lambda pair: str(pair[0])))
    if isinstance(value, (list, tuple)):
        return tuple(freeze(item) for item in value)
    if isinstance(value, set):
        return tuple(sorted(value))
    return value


def training_state(actor):
    model = actor.model
    return freeze(dict(parameters=model.parameters, origins=model.origin_ids,
        gradients={key: value.grad for key, value in model.parameters.items()},
        optimizer=model.optimizer.state_dict() if model.optimizer is not None else None,
        model_views=model.views, generation=model.generation, candidate_pending=model.candidate_pending,
        confirmed=actor.confirmed_state, binding=actor.confirmed_binding, current=actor.current_target,
        content=actor.content, pending=actor.pending, assets=actor.assets, references=actor.references,
        checkpoint=actor.checkpoint_id, pending_checkpoint=actor.pending_checkpoint,
        map_sequence=actor.map_sequence, known=actor._known_submaps,
        max_generation=actor._max_generation, max_version=actor._max_version, max_frontier=actor._max_frontier))


class RenderActor:
    def __init__(self, root, device, enabled=True):
        kwargs = dict(joint_render_enabled=True) if enabled else {}
        self.actor = GaussianSubmapWorker(SESSION, root / 'states', OnlineGaussianModel(device),
                                         METADATA, **kwargs)
        self.job_id = 0
        self.generation = 0
        self.frontier = 0
        self.sequence = 0
        self.references = {}
        self.targets = {}
        self.target_bodies = {}
        self.target = None
        self.target_body = None
        self.assets = {}

    def job(self):
        self.job_id += 1
        return self.job_id

    def activate(self, submap, first, count, asset=None, version=0):
        self.generation += 1
        header, body = target_fixture(submap=submap, first=first, count=count,
            generation=self.generation, version=version, job=self.job(),
            action='reuse' if asset else 'empty')
        self.frontier = max(self.frontier, first + count)
        header.update(frame_count=str(self.frontier), optimized_frame_count=str(self.frontier if version else 0),
                      parent_from_submap=pose(first, 10 if version else 0).ravel().tolist())
        if asset:
            header.update({key: asset[key] for key in ('asset_id', 'manifest_sha256',
                                                       'source_generation', 'content_job_id')})
        reply, _ = self.actor.handle(header, body)
        self.target, self.target_body = header, body
        self.targets[submap], self.target_bodies[submap] = copy.deepcopy(header), body
        if reply['type'] == 'candidate':
            self.decide(reply)
        return reply

    def decide(self, candidate):
        header = {key: candidate[key] for key in ('job_id', 'generation', 'pose_version', 'submap_id', 'operation')}
        return self.actor.handle(dict(schema=2, type='decision', session_id=SESSION,
                                      action='accept', **header), b'')[0]

    def view(self, frame, accept=True):
        target = self.target
        header, body = view_v2_fixture(frame, target['primary_count'], int(target['anchor_keyframe_id']),
            int(target['submap_id']), self.generation, int(target['pose_version']), self.job())
        header.update({key: target[key] for key in ('frame_count', 'optimized_frame_count', 'parent_from_submap')})
        reply, rgb = self.actor.handle(header, body[:9 * 48] + self.target_body)
        if accept:
            self.decide(reply)
        return reply, rgb

    def acknowledge(self, version=0):
        self.sequence += 1
        self.actor.handle(control_fixture('map_committed', job=self.job(), generation=self.generation,
            version=version, map_sequence=str(self.sequence),
            asset_ids=sorted((asset['asset_id'] for asset in self.assets.values()), key=int)), b'')

    def checkpoint(self):
        source = self.actor.content
        asset_id = max((int(asset['asset_id']) for asset in self.assets.values()), default=0) + 1
        reply, _ = self.actor.handle(control_fixture('checkpoint', job=self.job(),
            generation=self.generation, version=int(self.target['pose_version']),
            submap=int(source['submap_id']), asset_id=str(asset_id),
            source_generation=source['source_generation'], content_job_id=source['content_job_id']), b'')
        asset = {key: copy.deepcopy(reply[key]) for key in (*ASSET_FIELDS, 'frame_ids', 'gaussian_count')}
        self.assets[int(source['submap_id'])] = asset
        self.acknowledge(int(self.target['pose_version']))
        return asset

    def provision(self):
        self.activate(0, 0, 3)
        self.view(0)
        first = self.checkpoint()
        self.activate(1, 3, 2)
        self.view(3)
        self.checkpoint()
        self.activate(2, 5, 2)
        self.view(5)
        self.view(6)
        self.checkpoint()
        self.activate(0, 0, 3, asset=first)
        self.view(1)
        self.acknowledge()

    def request(self, version=0):
        header = dict(schema=2, type='joint_render', session_id=SESSION,
            job_id=str(self.job()), generation=str(self.generation), pose_version=str(version),
            map_sequence=str(self.sequence), frame_count='7', optimized_frame_count='6' if version else '0',
            parent_frame='map' if version else 'odom', query_keyframe_id='6', query_submap_id='2',
            neighbor_reason='', blocks=[])
        pieces = [pose(6, 10 if version else 0).astype('<f8').tobytes()]
        for submap in (1, 2):
            target = {key: copy.deepcopy(self.targets[submap][key]) for key in TARGET_FIELDS}
            target.update(frame_count='7', optimized_frame_count=header['optimized_frame_count'],
                pose_version=str(version), parent_frame=header['parent_frame'],
                parent_from_submap=pose(3 if submap == 1 else 5, 10 if version else 0).ravel().tolist())
            asset = self.assets[submap]
            header['blocks'].append(dict(target=target,
                content={key: asset[key] for key in ('submap_id', 'source_generation', 'content_job_id')},
                resident=0, asset={key: asset[key] for key in ASSET_FIELDS},
                frame_ids=asset['frame_ids'][:], gaussian_count=asset['gaussian_count']))
            pieces.append(self.target_bodies[submap])
        return header, b''.join(pieces)

    def release(self):
        if self.actor.model is not None:
            self.actor.model.reset(0)
        self.actor.confirmed_state = None
        self.actor.pending = None
