"""Verify stride, final-chunk alignment and action-free OpenPI observations."""
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import pandas as pd

from dataset.gr00t_open_loop_adapter import F14PolicyAdapter, Modality, PolicyHorizonSpec
from third_party.gr00t_n17_eval import open_loop_eval as evaluation


class OpenLoopTests(unittest.TestCase):
    def test_replan_and_truncate_last_chunk_in_native_units(self):
        length = 23
        truth = (np.arange(length)[:, None] + np.arange(16)[None, :]).astype(np.float32)
        trajectory = pd.DataFrame({'state.joints': list(truth), 'action.joints': list(truth)})
        calls = []

        class Tensor:
            def __init__(self, value):
                self.value = value
            def numpy(self):
                return self.value

        class Dataset:
            def __getitem__(self, index):
                return {'observation.state': Tensor(truth[index]), 'task': 'pick and place',
                    **{f'observation.images.{name}': Tensor(np.full((3, 4, 5), value, np.float32))
                       for name, value in [('front', .1), ('left_wrist', .2), ('right_wrist', .3)]}}

        trajectory.attrs['dataset'] = Dataset()
        modalities = {'state': Modality(['joints'], [0]),
            'action': Modality(['joints'], list(range(50))),
            'video': Modality(['front', 'left_wrist', 'right_wrist'], [0]),
            'language': Modality(['task'], [0])}

        class Loader:
            modality_configs = modalities
            def __getitem__(self, episode):
                return trajectory

        def infer(observation):
            self.assertEqual(set(observation), {'state', 'images', 'prompt'})
            self.assertEqual(observation['images']['front'].shape, (3, 4, 5))
            self.assertAlmostEqual(float(observation['images']['right_wrist'][0, 0, 0]), .3)
            calls.append(int(observation['state'][0]))
            return {'actions': observation['state'][None, :] + np.arange(50, dtype=np.float32)[:, None] + 1}

        adapter = F14PolicyAdapter(SimpleNamespace(infer=infer), list(range(16)), 50)
        with patch.object(evaluation, 'plot_trajectory_results') as plot:
            mse, mae = evaluation.evaluate_single_trajectory(adapter, Loader(), 0, 'F14',
                steps=200, execution_horizon=16)
        self.assertEqual(calls, [0, 16])
        self.assertEqual(float(mse), 1.)
        self.assertEqual(float(mae), 1.)
        np.testing.assert_array_equal(plot.call_args.kwargs['pred_action_across_time'], truth+1)
        np.testing.assert_array_equal(plot.call_args.kwargs['gt_action_across_time'], truth)

    def test_execution_horizon_contract(self):
        config = {'action': Modality(['joints'], list(range(50)))}
        for invalid in (0, 51):
            with self.assertRaises(ValueError):
                PolicyHorizonSpec.from_modality_config(config, invalid)
        config['action'].delta_indices = [0, 2]
        with self.assertRaises(ValueError):
            PolicyHorizonSpec.from_modality_config(config, 1)


if __name__ == '__main__':
    unittest.main()
