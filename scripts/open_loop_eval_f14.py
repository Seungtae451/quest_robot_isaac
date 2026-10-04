"""Run the GR00T N1.7 open-loop evaluator unchanged with an OpenPI F14 policy."""
import argparse
from datetime import datetime
import json
import logging
import os
from pathlib import Path
import sys
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-path', '--checkpoint', dest='model_path', type=Path, required=True)
    parser.add_argument('--dataset-path', type=Path, default=ROOT/'outputs/openpi_training/f14_cube_pickplace/lerobot/local/f14_cube_pickplace')
    parser.add_argument('--openpi-root', type=Path, default=ROOT.parent/'openpi')
    parser.add_argument('--traj-ids', type=int, nargs='+', default=[0])
    parser.add_argument('--execution-horizon', type=int, default=16)
    parser.add_argument('--steps', type=int, default=200)
    parser.add_argument('--denoising-steps', type=int, default=10, help='OpenPI default is 10; GR00T model default is 4')
    parser.add_argument('--save-plot-path', type=Path, help='Output directory (one traj_<id>.jpeg per episode)')
    parser.add_argument('--device', choices=('cpu', 'gpu'), default='cpu')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--wandb', action='store_true')
    args = parser.parse_args()
    if args.steps < 1 or args.denoising_steps < 1:
        parser.error('steps and denoising-steps must be positive.')
    from scripts.train_openpi_f14 import require_isaac_stopped, setup_nccl_runtime
    if args.device == 'gpu':
        require_isaac_stopped()
        processes = os.popen('ps -eo args').read().splitlines()
        if any('train_openpi_f14.py' in command and 'python' in command for command in processes):
            parser.error('Use CPU during GPU training; stop training before GPU evaluation.')
        setup_nccl_runtime()
    os.environ['JAX_PLATFORMS'] = 'cuda' if args.device == 'gpu' else 'cpu'
    os.environ['XLA_PYTHON_CLIENT_PREALLOCATE'] = 'false'
    os.environ.setdefault('OPENPI_DATA_HOME', str(ROOT/'outputs/openpi_cache'))
    os.environ['HF_HUB_OFFLINE'] = '1'
    os.environ['HF_DATASETS_OFFLINE'] = '1'
    sys.path.insert(0, str(args.openpi_root.resolve()/'src'))
    checkpoint = args.model_path.resolve()
    metadata = json.loads((checkpoint/'_CHECKPOINT_METADATA').read_text())
    if not metadata.get('commit_timestamp_nsecs'):
        raise ValueError('Checkpoint is not fully committed yet.')
    snapshot = args.dataset_path.resolve()
    info = json.loads((snapshot/'meta/info.json').read_text())
    if len(set(args.traj_ids)) != len(args.traj_ids) or any(
            episode < 0 or episode >= info['total_episodes'] for episode in args.traj_ids):
        parser.error('traj-ids must contain unique existing episode indices.')
    import numpy as np
    import jax
    import matplotlib
    matplotlib.use('Agg')
    from openpi.models.pi0_config import Pi0Config
    from openpi.training.config import TrainConfig, AssetsConfig
    from openpi.policies.policy_config import create_trained_policy
    from dataset.openpi_f14 import LeRobotF14DataConfig
    from dataset.gr00t_open_loop_adapter import F14EpisodeLoader, F14PolicyAdapter, PolicyHorizonSpec
    from third_party.gr00t_n17_eval import open_loop_eval as evaluation
    model = Pi0Config(pi05=True, paligemma_variant='gemma_2b_lora', action_expert_variant='gemma_300m_lora')
    config = TrainConfig(name='pi05_f14_lora', model=model,
        data=LeRobotF14DataConfig(repo_id='local/f14_cube_pickplace', assets=AssetsConfig(asset_id='f14_cube_pickplace')),
        seed=args.seed)
    labels = info['features']['action']['names']
    loader = F14EpisodeLoader(snapshot, labels, model.action_horizon)
    PolicyHorizonSpec.from_modality_config(loader.modality_configs, args.execution_horizon)
    logging.basicConfig(level=logging.INFO)
    print(f'Loading checkpoint {checkpoint.name} on {jax.devices()}', flush=True)
    policy = create_trained_policy(config, checkpoint, sample_kwargs={'num_steps': args.denoising_steps})
    policy._rng = jax.random.key(args.seed)
    adapter = F14PolicyAdapter(policy, labels, model.action_horizon)
    output = args.save_plot_path or ROOT/'outputs/openpi_open_loop'/f'{checkpoint.name}_{datetime.now(ZoneInfo("Asia/Seoul")):%Y%m%d_%H%M%S}'
    output.mkdir(parents=True, exist_ok=False)
    upstream_plot = evaluation.plot_trajectory_results
    rows = []
    run = None
    if args.wandb:
        import wandb
        run = wandb.init(project='f14-policy-evaluation', name=output.name,
            config={'checkpoint': str(checkpoint), 'execution_horizon': args.execution_horizon,
                    'steps': args.steps, 'seed': args.seed, 'denoising_steps': args.denoising_steps})
    try:
        for episode in args.traj_ids:
            # Capture arrays for reproducibility; call the original plotting function unchanged.
            def capture_plot(**values):
                np.savez(output/f'traj_{episode}.npz',
                    state=values['state_joints_across_time'], truth=values['gt_action_across_time'],
                    prediction=values['pred_action_across_time'])
                upstream_plot(**values)
            evaluation.plot_trajectory_results = capture_plot
            mse, mae = evaluation.evaluate_single_trajectory(adapter, loader, episode,
                embodiment_tag='F14', steps=args.steps, execution_horizon=args.execution_horizon,
                save_plot_path=str(output/f'traj_{episode}.jpeg'))
            rows.append({'traj_id': episode, 'mse': float(mse), 'mae': float(mae),
                         'actual_steps': min(args.steps, len(loader[episode]))})
            print(f'Trajectory {episode}: MSE={mse}, MAE={mae}', flush=True)
            if run:
                run.log({f'traj_{episode}/mse': float(mse), f'traj_{episode}/mae': float(mae),
                         f'traj_{episode}/plot': wandb.Image(str(output/f'traj_{episode}.jpeg'))})
    finally:
        evaluation.plot_trajectory_results = upstream_plot
        if run:
            run.finish()
    metrics = {'checkpoint': str(checkpoint), 'dataset': str(snapshot), 'seed': args.seed,
        'execution_horizon': args.execution_horizon, 'model_action_horizon': model.action_horizon,
        'denoising_steps': args.denoising_steps, 'device': args.device, 'trajectories': rows,
        'average_mse': float(np.mean([row['mse'] for row in rows])),
        'average_mae': float(np.mean([row['mae'] for row in rows])), 'action_names': labels,
        'units': 'Original unnormalized actions: arm radians and gripper open_ratio, same reduction as GR00T.',
        'scope': 'Recorded observations at every inference point, no robot rollout. These episodes were used for training.',
        'reference': json.loads((ROOT/'third_party/gr00t_n17_eval/provenance.json').read_text())}
    if run:
        metrics['wandb_url'] = run.url
    (output/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
    print(f'Average MSE={metrics["average_mse"]}, MAE={metrics["average_mae"]}\nSaved: {output}', flush=True)


if __name__ == '__main__':
    main()
