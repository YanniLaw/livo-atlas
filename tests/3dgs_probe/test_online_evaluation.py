import copy
import json
from pathlib import Path
from unittest import mock

import numpy as np
import torch

from online_evaluation import OnlineEvaluation
from render_actor_fixtures import TARGET_FIELDS, training_state
from test_holdout_worker import HoldoutWorkerTest, observation
from evaluation_protocol import validate_evaluation_message


class OnlineEvaluationTest(HoldoutWorkerTest):
    def request(self, job=3):
        self.train(0)
        actor = self.actor
        actor.map_sequence = 0
        actor.evaluation = OnlineEvaluation(Path(self.temporary.name), self.split,
            dict(max_output_bytes=str(16*1024*1024), min_free_disk_bytes='0'))
        target = copy.deepcopy(actor.current_target)
        target.update(submap_state='sealed', membership_revision='15', primary_count=14)
        matrix = np.asarray(target['parent_from_submap']).reshape(4, 4) @ np.asarray(target['primary'][1]['submap_from_body']).reshape(4, 4)
        source = dict(schema=2, type='gaussian_sources', session_id=actor.session_id, job_id=str(job),
            generation='1', pose_version='0', map_sequence='1', frame_count='14', optimized_frame_count='0',
            parent_frame='odom', query_keyframe_id='1', query_submap_id='0', neighbor_reason='no_previous_group',
            blocks=[dict(target={k:target[k] for k in TARGET_FIELDS}, content=copy.deepcopy(actor.content),
                         resident=1, frame_ids=['0'], gaussian_count=int(actor.model.origin_ids.numel()))])
        header, pixels = observation(1)
        query = {k: header[k] for k in ('width','height','calibration_id','fx','fy','cx','cy',
                 'state_timestamp','cloud_timestamp','image_timestamp','inverse_exposure_time','camera_from_body')}
        query['keyframe_id'] = '1'
        outer = dict(schema=2,type='heldout_evaluate',session_id=actor.session_id,job_id=str(job),
                     split_id=self.split.split_id,sources=source,query=query,
                     sealed_at_unix_seconds=1.,dispatched_at_unix_seconds=2.)
        primary = np.asarray([x['submap_from_body'] for x in target['primary']],dtype='<f8').tobytes()
        body = matrix.astype('<f8').tobytes()+primary+pixels[:9*query['width']*query['height']]
        row = dict(submap_id='0',binding=source['blocks'][0]['target'],availability='current',
                   resident=1,content=dict(actor.content,session_id=actor.session_id),
                   trained_frame_ids=['0'],gaussian_count=source['blocks'][0]['gaussian_count'])
        document = dict(schema=1,kind='livo_atlas.gaussian_map',session_id=actor.session_id,
                        map_sequence='1',pose_version='0',frame_count='14',optimized_frame_count='0',
                        parent_frame='odom',entries=[row])
        (Path(self.temporary.name)/'map.json').write_text(json.dumps(document))
        actor._map_committed(dict(map_sequence='1',asset_ids=[],generation='1',pose_version='0',
            session_id=actor.session_id,evaluation_bindings=[dict(submap_id='0',primary_count=14)]),primary)
        validate_evaluation_message(outer,body)
        return outer,body

    def test_score_uses_independent_reference_without_mutating_training(self):
        header,body = self.request()
        before = training_state(self.actor)
        rng = torch.cuda.get_rng_state().clone()
        reply,raw = self.actor.handle(header,body)
        self.assertEqual(reply['type'],'heldout_evaluation_ok')
        self.assertEqual(raw,b'')
        self.assertEqual(training_state(self.actor),before)
        self.assertTrue(torch.equal(rng,torch.cuda.get_rng_state()))
        self.assertTrue((Path(self.temporary.name)/'evaluations/views/1/report.json').is_file())

    def test_changed_reference_fails_scoring_but_training_continues(self):
        header,body = self.request()
        offset = 128*15
        body = body[:offset]+bytes([99])+body[offset+1:]
        reply,_ = self.actor.handle(header,body)
        self.assertEqual(reply['type'],'heldout_evaluation_error')
        self.assertFalse(self.actor.failed)
        self.train(2)

    def test_exhausted_budget_is_reported_without_poisoning_training(self):
        header, body = self.request()
        before = training_state(self.actor)
        self.actor.evaluation.maximum = 0
        reply, _ = self.actor.handle(header, body)
        self.assertEqual(reply['type'], 'heldout_evaluation_error')
        self.assertEqual(reply['code'], 'budget')
        self.assertEqual(training_state(self.actor), before)
        self.train(2)

    def test_oom_is_scoring_failure_and_preserves_rng(self):
        header, body = self.request()
        before = training_state(self.actor)
        rng = torch.cuda.get_rng_state().clone()
        with mock.patch('online_evaluation.infer_blocks', side_effect=torch.cuda.OutOfMemoryError('controlled')):
            reply, _ = self.actor.handle(header, body)
        self.assertEqual(reply['code'], 'oom')
        self.assertEqual(training_state(self.actor), before)
        self.assertTrue(torch.equal(rng, torch.cuda.get_rng_state()))
        self.train(2)

    def test_claimed_frontier_must_equal_the_acknowledged_catalogue(self):
        header, body = self.request()
        header['sources'].update(pose_version='1', optimized_frame_count='14', parent_frame='map')
        for block in header['sources']['blocks']:
            block['target'].update(pose_version='1', optimized_frame_count='14', parent_frame='map')
        reply, _ = self.actor.handle(header, body)
        self.assertEqual(reply['type'], 'heldout_evaluation_error')
        self.assertEqual(reply['code'], 'source')

    def test_absolute_binding_must_equal_the_acknowledged_catalogue(self):
        header, body = self.request()
        header['sources']['blocks'][-1]['target']['parent_from_submap'][3] += 10
        pose = np.frombuffer(body[:128],dtype='<f8').copy().reshape(4,4)
        pose[0,3] += 10
        body = pose.tobytes()+body[128:]
        reply, _ = self.actor.handle(header, body)
        self.assertEqual(reply['type'], 'heldout_evaluation_error')
        self.assertEqual(reply['code'], 'source')

    def test_report_preserves_software_and_cumulative_inference_evidence(self):
        header, body = self.request()
        software = dict(python='3.10-test',torch='2.3-test',cuda='12.1-test',
                        gsplat='1.5-test',device='test device')
        self.actor.software = dict(software)
        before = training_state(self.actor)
        rng = torch.cuda.get_rng_state().clone()
        peak = torch.cuda.max_memory_allocated(self.actor.device)
        reply, _ = self.actor.handle(header,body)
        self.assertEqual(reply['type'],'heldout_evaluation_ok')
        report = json.loads((Path(self.temporary.name)/'evaluations/views/1/report.json').read_text())
        self.assertEqual(report['software'],dict(software,numpy=np.__version__))
        inference = report['inference']
        self.assertEqual(inference['memory_peak_scope'],'worker_lifetime')
        self.assertIsNone(inference['joint'])
        single = inference['single']
        self.assertGreaterEqual(single['gpu_peak_allocated_bytes'],peak)
        self.assertGreaterEqual(single['gpu_peak_reserved_bytes'],single['gpu_peak_allocated_bytes'])
        self.assertGreater(single['gaussian_count'],0)
        self.assertEqual(set(single['timings']),{'transform_upload_seconds','render_seconds','total_seconds'})
        self.assertTrue(all(value>=0 for value in single['timings'].values()))
        self.assertEqual(training_state(self.actor),before)
        self.assertTrue(torch.equal(rng,torch.cuda.get_rng_state()))
