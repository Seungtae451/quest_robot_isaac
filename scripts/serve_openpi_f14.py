"""Serve the latest trained F14 checkpoint using OpenPI's official WebSocket server."""
import argparse
import json
import logging
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', default='latest')
    parser.add_argument('--experiment-dir', type=Path, default=ROOT/'outputs/openpi_training/checkpoints/pi05_f14_lora/f14_pi05_100eps')
    parser.add_argument('--openpi-root', type=Path, default=ROOT.parent/'openpi')
    parser.add_argument('--dataset-path', type=Path, default=ROOT/'outputs/openpi_training/f14_cube_pickplace/lerobot/local/f14_cube_pickplace')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--device', choices=('cpu','gpu'), default='gpu')
    parser.add_argument('--denoising-steps', type=int, default=10)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--check-only', action='store_true')
    args=parser.parse_args()
    if args.denoising_steps < 1:
        parser.error('denoising-steps must be positive')
    from dataset.openpi_checkpoint import resolve_checkpoint
    checkpoint=resolve_checkpoint(args.checkpoint,args.experiment_dir)
    info=json.loads((args.dataset_path/'meta/info.json').read_text())
    print(f'Selected completed checkpoint: {checkpoint}',flush=True)
    if args.check_only:
        return
    if args.device=='gpu':
        from scripts.train_openpi_f14 import setup_nccl_runtime
        import subprocess
        commands=subprocess.check_output(['ps','-eo','args'],text=True).splitlines()
        if any('python' in command and 'train_openpi_f14.py' in command for command in commands):
            parser.error('GPU training is still running. Stop/finish training before GPU inference, or use --device cpu.')
        os.environ.setdefault('CUDA_VISIBLE_DEVICES','0')
        setup_nccl_runtime()
    os.environ['JAX_PLATFORMS']='cuda' if args.device=='gpu' else 'cpu'
    os.environ['XLA_PYTHON_CLIENT_PREALLOCATE']='false'
    os.environ.setdefault('OPENPI_DATA_HOME',str(ROOT/'outputs/openpi_cache'))
    os.environ['HF_HUB_OFFLINE']='1'
    os.environ['HF_DATASETS_OFFLINE']='1'
    sys.path.insert(0,str(args.openpi_root/'src'))
    import jax
    from openpi.models.pi0_config import Pi0Config
    from openpi.training.config import TrainConfig,AssetsConfig
    from openpi.policies.policy_config import create_trained_policy
    from openpi.serving.websocket_policy_server import WebsocketPolicyServer
    from dataset.openpi_f14 import LeRobotF14DataConfig
    model=Pi0Config(pi05=True,paligemma_variant='gemma_2b_lora',action_expert_variant='gemma_300m_lora')
    config=TrainConfig(name='pi05_f14_lora',model=model,seed=args.seed,
        data=LeRobotF14DataConfig(repo_id='local/f14_cube_pickplace',assets=AssetsConfig(asset_id='f14_cube_pickplace')))
    logging.basicConfig(level=logging.INFO)
    policy=create_trained_policy(config,checkpoint,sample_kwargs={'num_steps':args.denoising_steps})
    policy._rng=jax.random.key(args.seed)
    metadata={'format':'f14_absolute_joint_actions_v1','checkpoint':str(checkpoint),
        'action_names':info['features']['action']['names'],'action_horizon':model.action_horizon,
        'action_fps':info['fps'],'denoising_steps':args.denoising_steps,'seed':args.seed,
        'dataset':str(args.dataset_path.resolve()),'device':args.device}
    print(f'Policy ready at ws://{args.host}:{args.port}',flush=True)
    WebsocketPolicyServer(policy,host=args.host,port=args.port,metadata=metadata).serve_forever()


if __name__=='__main__':
    main()
