"""Serialized historical heldout scoring of confirmed online sources."""

import copy
import hashlib
import json
from pathlib import Path
import shutil
import time

import numpy as np
import torch

from evaluation_metrics import measure
from evaluation_protocol import decode_evaluation
from evaluate_heldout_snapshot import _json_bytes, _save_samples
from heldout_query import canonical_query
from joint_renderer import infer_blocks
from online_render import confirmed_sources
from render_gaussian_map import _exclusive_write
from gaussian_state import _open_file, _unique_object, _reject_constant
from submap_protocol import _primary_context


class EvaluationBudgetError(ValueError):
    """A scoring reservation failed without changing model state."""


class OnlineEvaluation:
    def __init__(self, parent, split, limits):
        self.directory = Path(parent) / 'evaluations'
        self.split = split
        self.maximum = int(limits['max_output_bytes'])
        self.reserve = int(limits['min_free_disk_bytes'])
        # Fixed all-query upper bound, including paired renders, reference,
        # PNGs, ZIP headers and bounded owner/score reports. No query pruning.
        predicted = 2*1024*1024 + sum(frame['width']*frame['height']*67 + 2*1024*1024 + 128*1024
            for frame in split.frames.values() if frame['query'])
        if predicted > self.maximum or shutil.disk_usage(parent).free < predicted+self.reserve:
            raise ValueError('Online evaluation disk budget cannot cover the frozen query plan')
        self.directory.mkdir(mode=0o700)
        self.completed = set()
        self.capture = None

    def commit_map(self, request, body):
        """Bind the complete pose supplement to the catalogue being acknowledged."""
        if 'evaluation_bindings' not in request:
            raise ValueError('Online evaluation requires complete acknowledged bindings')
        with _open_file(self.directory.parent/'map.json',512*1024) as stream:
            raw = stream.read()
        document = json.loads(raw,object_pairs_hook=_unique_object,parse_constant=_reject_constant)
        if (document.get('schema')!=1 or document.get('kind')!='livo_atlas.gaussian_map' or
                any(document.get(key)!=request[key] for key in ('session_id','map_sequence','pose_version'))):
            raise ValueError('Evaluation catalogue differs from map acknowledgement')
        entries = document.get('entries')
        if not isinstance(entries,list) or len(entries)>512:
            raise ValueError('Evaluation catalogue exceeds its entry budget')
        rows = {row['submap_id']:row for row in entries if 'content' in row}
        proofs = request['evaluation_bindings']
        if len(rows)>32 or [proof['submap_id'] for proof in proofs] != list(rows):
            raise ValueError('Evaluation proof omits confirmed catalogue entries')
        primary, offset = {}, 0
        for proof in proofs:
            key = proof['submap_id']
            binding = rows[key]['binding']
            count = proof['primary_count']
            if binding['primary_count'] != count:
                raise ValueError('Evaluation proof membership differs from catalogue')
            segment = body[offset:offset+128*count]
            primary[key] = _primary_context(dict(binding,session_id=request['session_id']),segment,0)['primary']
            offset += 128*count
        if offset != len(body):
            raise ValueError('Evaluation proof has trailing bytes')
        self.capture = dict(document=document,rows=rows,primary=primary,generation=request['generation'],
                            map_sha256=hashlib.sha256(raw).hexdigest(),
                            primary_sha256=hashlib.sha256(body).hexdigest())

    def verify_sources(self, request):
        capture = self.capture
        header = request['header']
        fields = ('session_id','map_sequence','pose_version','frame_count','optimized_frame_count','parent_frame')
        if (capture is None or any(header[key]!=capture['document'][key] for key in fields) or
                header['generation'] != capture['generation']):
            raise ValueError('Evaluation frontier differs from acknowledged catalogue')
        for block in request['blocks']:
            wire = block['header']
            key = wire['target']['submap_id']
            row = capture['rows'].get(key)
            content = dict(wire['content'],session_id=header['session_id'])
            if (row is None or row['availability']!='current' or wire['target']!=row['binding'] or
                    wire['resident']!=row['resident'] or content!=row['content'] or
                    wire['frame_ids']!=row['trained_frame_ids'] or wire['gaussian_count']!=row['gaussian_count'] or
                    block['target']['primary']!=capture['primary'][key]):
                raise ValueError('Evaluation source differs from acknowledged catalogue binding')
            if not wire['resident'] and dict(wire['asset'],session_id=header['session_id'])!=row['asset']:
                raise ValueError('Evaluation asset differs from acknowledged catalogue')
        return capture


    def operation(self, actor, header, body):
        source = header['sources']
        reply = dict(schema=2, session_id=actor.session_id, job_id=header['job_id'],
                     split_id=header['split_id'], keyframe_id=source['query_keyframe_id'],
                     map_sequence=source['map_sequence'])
        try:
            result = self.evaluate(actor, decode_evaluation(header, body))
            reply.update(type='heldout_evaluation_ok', report_sha256=result[0], report_bytes=str(result[1]))
        except torch.cuda.OutOfMemoryError as error:
            reply.update(type='heldout_evaluation_error', code='oom', message=ascii(str(error))[:2048])
        except EvaluationBudgetError as error:
            reply.update(type='heldout_evaluation_error', code='budget', message=ascii(str(error))[:2048])
        except OSError as error:
            reply.update(type='heldout_evaluation_error', code='storage', message=ascii(str(error))[:2048])
        except ValueError as error:
            reply.update(type='heldout_evaluation_error', code='source', message=ascii(str(error))[:2048])
        return reply, b''

    def evaluate(self, actor, request):
        started = time.perf_counter()
        header, captured = request['header'], request['sources']
        capture = self.verify_sources(captured)
        if header['split_id'] != self.split.split_id:
            raise ValueError('Online query uses a different frozen split')
        query = canonical_query(self.split, request['query'])
        key = query['keyframe_id']
        source = captured['header']
        owner = int(self.split.frames[key]['submap_id'])
        if (key in self.completed or owner != int(source['query_submap_id']) or
                captured['blocks'][-1]['target']['submap_state'] != 'sealed'):
            raise ValueError('Online query was already scored or lacks a sealed owner')
        camera = query['camera_from_body'] @ np.linalg.inv(captured['parent_from_body'])
        query['camera_from_parent'] = camera
        fence = (actor.map_sequence, actor._max_generation)
        blocks, sources = [], []
        for block, state in confirmed_sources(actor, captured):
            self.split.check_state(state)
            self.split.check_target(block['target'])
            params = {name: value.copy() for name, value in state.parameters.items()}
            group = int(block['target']['submap_id'])
            transform = camera @ np.asarray(block['target']['parent_from_submap']).reshape(4, 4)
            blocks.append((group, params, transform))
            proof = state.metadata['holdout']
            sources.append(dict(submap_id=group, content=copy.deepcopy(block['header']['content']),
                asset=copy.deepcopy(block['header'].get('asset')), target=copy.deepcopy(block['target']),
                lineage_sha256=proof['lineage_sha256'],
                training_lineage_ids=[int(item['keyframe_id']) for item in proof['lineage']]))
            del state
        pixels = query['rgb'].shape[0]*query['rgb'].shape[1]
        needed = pixels*(18+20*len(blocks)+3*(1+len(blocks)))+2*1024*1024+128*1024
        used = sum(path.stat().st_size for path in self.directory.rglob('*') if path.is_file())
        if used+needed > self.maximum or shutil.disk_usage(self.directory).free < needed+self.reserve:
            raise EvaluationBudgetError('Online evaluation resource budget was exhausted')
        own = next(block for block in blocks if block[0] == owner)
        height, width = query['rgb'].shape[:2]
        single = infer_blocks([own], query['K'], width, height, actor.device)
        joint = infer_blocks(blocks, query['K'], width, height, actor.device) if len(blocks) == 2 else None
        report = dict(schema=1, kind='livo_atlas.gaussian_online_evaluation', phase='online_snapshot',
            status='succeeded', keyframe_id=key, split_id=self.split.split_id, training_performed=False,
            sealed_at_unix_seconds=header['sealed_at_unix_seconds'],
            dispatched_at_unix_seconds=header['dispatched_at_unix_seconds'],
            as_of={name: source[name] for name in ('session_id','job_id','map_sequence','generation',
                'pose_version','frame_count','optimized_frame_count','parent_frame')},
            acknowledged_map_sha256=capture['map_sha256'],
            primary_bindings_sha256=capture['primary_sha256'],
            selection_policy=source.get('policy','temporal'), selection=source.get('selection'),
            neighbor_reason=source['neighbor_reason'], selected_ids=[x[0] for x in blocks], sources=sources,
            query=dict(calibration_id=query['calibration_id'], width=width, height=height,
                K=query['K'].tolist(), camera_from_body=query['camera_from_body'].ravel().tolist(),
                camera_from_parent=camera.ravel().tolist(),
                parent_from_body=captured['parent_from_body'].ravel().tolist(), capture_time=query['image_timestamp']),
            single=measure(single,query), joint=measure(joint,query) if joint is not None else None)
        if fence != (actor.map_sequence, actor._max_generation):
            raise ValueError('Confirmed evaluation sources changed during inference')
        folder = self.directory / 'views' / str(key)
        try:
            report['artifacts'] = _save_samples(self.directory, dict(query=query,single=single,joint=joint))
            report['evaluation_seconds'] = time.perf_counter()-started
            payload = _json_bytes(report, 2*1024*1024)
            if shutil.disk_usage(self.directory).free < self.reserve+len(payload):
                raise EvaluationBudgetError('Online evaluation disk reserve fell below publication budget')
            actual = sum(path.stat().st_size for path in self.directory.rglob('*') if path.is_file())
            if actual+len(payload) > self.maximum:
                raise EvaluationBudgetError('Online evaluation output budget exceeded before publication')
            _exclusive_write(folder/'report.pending.json',payload)
            if fence != (actor.map_sequence, actor._max_generation):
                raise ValueError('Confirmed evaluation sources changed before publication')
            (folder/'report.pending.json').rename(folder/'report.json')
        except Exception:
            if folder.exists(): shutil.rmtree(folder)
            raise
        self.completed.add(key)
        return hashlib.sha256(payload).hexdigest(), len(payload)
