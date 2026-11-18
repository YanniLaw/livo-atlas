"""Read confirmed CPU sources without touching the resident training model."""
import copy
import resource
import time

import numpy as np
import torch

from joint_renderer import image_metrics, infer_blocks


def render_confirmed(actor, request):
    from submap_worker import _compatible

    header = request['header']
    if (int(header['generation']) != actor._max_generation or
            int(header['map_sequence']) != actor.map_sequence or actor.pending_checkpoint is not None or
            int(header['frame_count']) < actor._max_frontier or int(header['pose_version']) < actor._max_version):
        raise ValueError('Render fence differs from the acknowledged worker map')
    started = time.perf_counter()
    blocks, query = [], None
    camera_from_parent = None
    for block in request['blocks']:
        source = block['header']
        content = dict(source['content'])
        if (actor.content is not None and actor.content['submap_id'] == content['submap_id'] and
                actor.content != content):
            raise ValueError('Render source is superseded by the resident confirmation')
        if not source['resident']:
            asset = source['asset']
            known = actor.assets.get(int(asset['asset_id']))
            if (known is None or int(asset['asset_id']) not in actor.references or
                    any(known.get(key) != value for key, value in asset.items())):
                raise ValueError('Render asset differs from its durable reference')
            content.update(asset)
        if header.get('policy') == 'spatial':
            known_metadata = (actor.confirmed_metadata if source['resident'] else
                              known.get('render_metadata'))
            if known_metadata is None or source['render_metadata'] != known_metadata:
                raise ValueError('Spatial render metadata differs from its confirmed source')
        state, _, _ = actor._source(content)
        target = block['target']
        ids = [str(view['keyframe_id']) for view in state.views]
        if (not _compatible(state, target) or ids != source['frame_ids'] or
                len(state.origin_ids) != source['gaussian_count']):
            raise ValueError('Render source coverage or canonical body poses differ')
        if target['submap_id'] == header['query_submap_id']:
            query = copy.deepcopy(next(view for view in state.views
                                      if str(view['keyframe_id']) == header['query_keyframe_id']))
            camera_from_parent = query['camera_from_body'] @ np.linalg.inv(request['parent_from_body'])
        # At most two small parameter sets and one retained query survive a load.
        blocks.append((int(target['submap_id']), {key: value.copy() for key, value in state.parameters.items()},
                       np.asarray(target['parent_from_submap'], dtype=np.float64).reshape(4, 4)))
        del state
    if query is None:
        raise ValueError('Requested training query is missing')
    known_query = max((int(frame) for key in actor.references
                       for frame in actor.assets[key]['frame_ids']), default=-1)
    if actor.confirmed_state is not None:
        known_query = max(known_query, max(int(view['keyframe_id']) for view in actor.confirmed_state.views))
    if int(header['query_keyframe_id']) < known_query:
        raise ValueError('Render query predates a newer known confirmation')
    blocks = [(key, parameters, camera_from_parent @ parent) for key, parameters, parent in blocks]
    height, width = query['rgb'].shape[:2]
    single = infer_blocks([blocks[-1]], query['K'], width, height, actor.device)
    joint = infer_blocks(blocks, query['K'], width, height, actor.device)
    metadata = dict(training_view=1, width=width, height=height,
        calibration_id=str(query['calibration_id']), fx=float(query['K'][0, 0]), fy=float(query['K'][1, 1]),
        cx=float(query['K'][0, 2]), cy=float(query['K'][1, 2]),
        camera_from_body=query['camera_from_body'].ravel().tolist(),
        camera_from_parent=camera_from_parent.ravel().tolist(),
        **{key: float(query[key]) for key in ('image_timestamp', 'state_timestamp',
                                             'cloud_timestamp', 'inverse_exposure_time')})
    single_rgb = np.rint(np.clip(single['rgb'], 0, 1) * 255).astype(np.uint8)
    joint_rgb = np.rint(np.clip(joint['rgb'], 0, 1) * 255).astype(np.uint8)
    single_metrics = image_metrics(single, query)
    joint_metrics = image_metrics(joint, query)
    metadata.update(changed_pixels=int(np.any(single_rgb != joint_rgb, axis=2).sum()),
        single_gaussian_count=single['gaussian_count'], gaussian_count=joint['gaussian_count'],
        worker_seconds=time.perf_counter() - started,
        gpu_peak_allocated_bytes=str(torch.cuda.max_memory_allocated(actor.device)),
        gpu_peak_reserved_bytes=str(torch.cuda.max_memory_reserved(actor.device)),
        gpu_allocated_bytes=str(torch.cuda.memory_allocated(actor.device)),
        gpu_reserved_bytes=str(torch.cuda.memory_reserved(actor.device)),
        worker_peak_rss_bytes=str(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024))
    return dict(rgb=joint_rgb, metadata=metadata, single=single_metrics, joint=joint_metrics)


def render_operation(actor, header, body):
    from joint_render_protocol import decode_render_request, encode_render_reply

    request = decode_render_request(header, body)
    try:
        return encode_render_reply(request, render_confirmed(actor, request))
    except torch.cuda.OutOfMemoryError as error:
        # repr preserves the original diagnostic, including invalid Unicode, as
        # bounded printable text accepted by the OpenCV JSON reader.
        prefix = str(error)
        while len(ascii(prefix).encode('ascii')) > 4096:
            prefix = prefix[:max(1, len(prefix) // 2)]
        reply = copy.deepcopy(header)
        reply.update(type='joint_render_error', code='oom', error=ascii(prefix))
        return reply, b''
