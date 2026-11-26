import copy
from pathlib import Path
import tempfile
import unittest

import numpy as np
import torch

from heldout_fixtures import make_heldout_snapshot
from heldout_query import load_evaluation_snapshot, render_query
from online_model import OnlineGaussianModel, checked_cuda_device
from test_online_model import make_view


def equal_state(test, before, after):
    if isinstance(before, torch.Tensor):
        test.assertTrue(torch.equal(before, after))
    elif isinstance(before, dict):
        test.assertEqual(before.keys(), after.keys())
        for key in before:
            equal_state(test, before[key], after[key])
    elif isinstance(before, (list, tuple)):
        test.assertEqual(len(before), len(after))
        for a, b in zip(before, after):
            equal_state(test, a, b)
    else:
        test.assertEqual(before, after)


class HeldoutQueryGpuTest(unittest.TestCase):
    def test_independent_dual_block_inference_preserves_live_optimizer_and_rng(self):
        device = checked_cuda_device()
        torch.set_num_threads(1)
        model = OnlineGaussianModel(device)
        model.reset(1)
        model.train_candidate(make_view(0))
        model.decide(True)
        optimizer = copy.deepcopy(model.optimizer.state_dict())
        parameters = {key: value.detach().clone() for key, value in model.parameters.items()}
        lineage = list(model.training_lineage)
        cpu_rng, cuda_rng = torch.get_rng_state().clone(), torch.cuda.get_rng_state(device).clone()
        numpy_rng = copy.deepcopy(np.random.get_state())
        with tempfile.TemporaryDirectory() as temporary:
            directory, keyframes, _, views = make_heldout_snapshot(Path(temporary))
            snapshot = load_evaluation_snapshot(directory, keyframes)
            before = {path: path.read_bytes() for path in directory.rglob('*') if path.is_file()}
            result = render_query(snapshot, views[1], [0, 2], device)
            self.assertIsNotNone(result['report']['joint'])
            self.assertEqual(result['single']['rgb'].dtype, np.dtype('f4'))
            for path, content in before.items():
                self.assertEqual(path.read_bytes(), content)
        for key, value in parameters.items():
            self.assertTrue(torch.equal(model.parameters[key], value))
        equal_state(self, optimizer, model.optimizer.state_dict())
        self.assertEqual(model.training_lineage, lineage)
        self.assertTrue(torch.equal(cpu_rng, torch.get_rng_state()))
        self.assertTrue(torch.equal(cuda_rng, torch.cuda.get_rng_state(device)))
        for a, b in zip(numpy_rng, np.random.get_state()):
            np.testing.assert_array_equal(a, b)


if __name__ == '__main__':
    unittest.main()
