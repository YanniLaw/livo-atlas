import copy
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np
import torch

from holdout_split import HoldoutSplit, build_manifest, canonical_json, observation_record
from online_model import OnlineGaussianModel, checked_cuda_device
from online_protocol import decode_view
from submap_fixtures import SESSION, control_fixture, target_fixture, view_v2_fixture
from submap_worker import GaussianSubmapWorker
from test_holdout_split import sample_manifest


def observation(key, count=14, job=2):
    header, body = view_v2_fixture(key, count=count, job=job)
    for field in ('state_timestamp', 'cloud_timestamp', 'image_timestamp'):
        header[field] += key
    pixels = 3 * header['width'] * header['height']
    return header, bytes([key] * pixels) + body[pixels:]


class HoldoutWorkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = checked_cuda_device()
        torch.set_num_threads(1)

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='livo-holdout-worker-')
        self.addCleanup(self.temporary.cleanup)
        manifest = sample_manifest(14)
        frames = [observation_record(decode_view(*observation(key)), '0',
                                    'heldout' if key % 8 == 1 else 'train') for key in range(14)]
        self.split = HoldoutSplit(canonical_json(build_manifest(manifest['source'], frames)))
        self.actor = GaussianSubmapWorker(SESSION, Path(self.temporary.name) / 'states',
                                         OnlineGaussianModel(self.device), {}, split=self.split)
        self.actor._activate(*target_fixture(count=14))
        self.next_job = 2

    def train(self, key, accept=True):
        job = self.next_job
        self.next_job += 1
        self.actor._view(*observation(key, job=job))
        self.actor._decision(control_fixture('decision', job=job, submap=0,
                                             operation='view', action='accept' if accept else 'discard'))

    def test_heldout_and_changed_input_do_not_mutate_model(self):
        self.train(0)
        before = {key: value.detach().clone() for key, value in self.actor.model.parameters.items()}
        generation = self.actor.model.generation
        for key, corrupt in ((1, False), (2, True)):
            header, body = observation(key, job=3)
            if corrupt:
                body = bytes([99]) + body[1:]
            with self.assertRaises(ValueError):
                self.actor._view(header, body)
            self.assertEqual(self.actor.model.generation, generation)
            self.assertTrue(all(torch.equal(before[k], v) for k, v in self.actor.model.parameters.items()))

    def test_eviction_preserves_history_and_rejection_rolls_back(self):
        keys = [key for key in range(14) if key % 8 != 1]
        for key in keys[:-1]:
            self.train(key)
        state = self.actor.confirmed_state
        self.split.check_state(state)
        self.assertNotIn(0, self.actor.model.frame_ids)
        self.assertEqual([int(x['keyframe_id']) for x in state.metadata['holdout']['lineage']], keys[:-1])
        proof = copy.deepcopy(state.metadata['holdout'])
        self.train(keys[-1], accept=False)
        self.assertEqual(self.actor.confirmed_state.metadata['holdout'], proof)
        self.assertEqual(self.actor.model.training_lineage, keys[:-1])

    def test_unproven_rebuild_is_rejected_before_model_release(self):
        self.train(0)
        self.actor.confirmed_state.metadata.pop('holdout')
        self.actor.confirmed_state.metadata['schema'] = 1
        model = self.actor.model
        header, body = target_fixture(count=14, generation=2, action='rebuild', job=3)
        header.update(source_generation='1', content_job_id='2')
        with self.assertRaises(ValueError):
            self.actor._activate(header, body)
        self.assertIs(self.actor.model, model)

    def test_rebuild_uses_only_proven_training_observations(self):
        self.train(0)
        self.train(2)
        header, body = target_fixture(count=14, generation=2, action='rebuild', job=4)
        header.update(source_generation='1', content_job_id='3')
        # Storage durability has its own integration coverage. This fixture
        # isolates the rebuild's source observations and resulting lineage.
        with mock.patch.object(self.actor, '_durable', return_value=True):
            self.actor._activate(header, body)
        self.actor._decision(control_fixture('decision', job=4, generation=2, submap=0,
                                             operation='rebuild', action='accept'))
        self.split.check_state(self.actor.confirmed_state)
        self.assertEqual(self.actor.model.training_lineage, [0, 2])
        self.assertEqual(self.actor.model.frame_ids, [0, 2])


if __name__ == '__main__':
    unittest.main()
