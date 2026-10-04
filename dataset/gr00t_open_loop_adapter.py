"""Connect F14/OpenPI to the unchanged GR00T N1.7 evaluation function bodies."""
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np


@dataclass
class Modality:
    modality_keys: list[str]
    delta_indices: list[int]


class PolicyHorizonSpec:
    @staticmethod
    def from_modality_config(config, n_action_steps):
        indices = config['action'].delta_indices
        if indices != list(range(len(indices))):
            raise ValueError('Evaluation requires contiguous actions beginning at zero.')
        if not 1 <= n_action_steps <= len(indices):
            raise ValueError('Execution horizon must be between 1 and the predicted chunk length.')


class F14EpisodeLoader:
    def __init__(self, snapshot, labels, horizon):
        self.snapshot = snapshot
        self.modality_configs = {
            'state': Modality(['joints'], [0]),
            'action': Modality(['joints'], list(range(horizon))),
            'video': Modality(['front', 'left_wrist', 'right_wrist'], [0]),
            'language': Modality(['annotation.human.action.task_description'], [0]),
        }
        self.cache = {}

    def __getitem__(self, episode):
        if episode not in self.cache:
            import json
            import pandas as pd
            import pyarrow.parquet as pq
            from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
            info = json.loads((self.snapshot/'meta/info.json').read_text())
            path = self.snapshot/info['data_path'].format(
                episode_chunk=episode//info['chunks_size'], episode_index=episode)
            table = pq.read_table(path)
            states = np.asarray(table['observation.state'].to_pylist(), dtype=np.float32)
            actions = np.asarray(table['action'].to_pylist(), dtype=np.float32)
            if states.shape != actions.shape or states.shape[1] != 16:
                raise ValueError('Expected matching [episode_length, 16] F14 state/action arrays.')
            trajectory = pd.DataFrame({'state.joints': list(states), 'action.joints': list(actions)})
            trajectory.attrs['dataset'] = LeRobotDataset('local/f14_cube_pickplace',
                root=self.snapshot, episodes=[episode], video_backend='pyav')
            self.cache[episode] = trajectory
        return self.cache[episode]


def extract_step_data(trajectory, step, config, embodiment_tag):
    if 'action' in config:
        raise ValueError('Ground-truth action must not enter inference inputs.')
    sample = trajectory.attrs['dataset'][step]
    state = sample['observation.state'].numpy()
    states = {'joints': state[None, :]}
    # Keep LeRobot's float32 [0,1] pixels; F14Inputs performs the official uint8 conversion.
    images = {key: np.transpose(sample[f'observation.images.{key}'].numpy(), (1, 2, 0))[None, :]
              for key in config['video'].modality_keys}
    return SimpleNamespace(states=states, images=images, text=sample['task'])


def parse_observation_gr00t(obs, config):
    """Map the evaluator's generic modalities to the trained OpenPI F14 inputs."""
    return {'state': np.concatenate([obs[f'state.{key}'][0] for key in config['state'].modality_keys]),
            'images': {key: np.transpose(obs[f'video.{key}'][0], (2, 0, 1))
                       for key in config['video'].modality_keys},
            'prompt': obs[config['language'].modality_keys[0]]}


class F14PolicyAdapter:
    def __init__(self, policy, labels, horizon):
        self.policy, self.labels, self.horizon = policy, labels, horizon

    def get_action(self, observation):
        if set(observation) != {'state', 'images', 'prompt'}:
            raise ValueError('Unexpected inference inputs.')
        prediction = np.asarray(self.policy.infer(observation)['actions'], dtype=np.float32)
        if prediction.shape != (self.horizon, len(self.labels)) or not np.isfinite(prediction).all():
            raise ValueError(f'Invalid predicted actions: {prediction.shape}')
        return {'joints': prediction[None, :]}, {}
