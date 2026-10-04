"""F14-specific pi05 LoRA entry point using the existing OpenPI checkout.

Use OpenPI's interpreter. --prepare-only makes a validated v2.1 snapshot;
--check-only loads one training batch without weights or gradient computation.
The default mode calls upstream JAX training (and downloads pi05 base weights).
"""
import argparse
from functools import partial
import importlib.util
import json
import logging
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import warnings

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# LeRobot's working PyAV backend uses torchvision.VideoReader. Suppress only
# its known deprecation notice, including in spawned data-loader workers.
warnings.filterwarnings(
    'ignore',
    message=r'The video decoding and encoding capabilities of torchvision are deprecated.*',
    category=UserWarning,
    module=r'torchvision\.io\._video_deprecation_warning$',
)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--openpi-root', type=Path, default=ROOT.parent/'openpi')
    p.add_argument('--dataset-root', type=Path, default=ROOT/'datasets/f14_cube_pickplace')
    p.add_argument('--prepared-root', type=Path, default=ROOT/'outputs/openpi_training/f14_cube_pickplace')
    modes = p.add_mutually_exclusive_group()
    modes.add_argument('--prepare-only', action='store_true')
    modes.add_argument('--check-only', action='store_true')
    p.add_argument('--exp-name', default='f14_pi05_100eps')
    p.add_argument('--batch-size', type=int, default=2)
    p.add_argument('--steps', type=int, default=10_000)
    p.add_argument('--fsdp-devices', type=int, default=2)
    p.add_argument('--num-workers', type=int, default=0)
    p.add_argument('--resume', action='store_true')
    p.add_argument('--base-params', default='gs://openpi-assets/checkpoints/pi05_base/params')
    p.add_argument('--wandb', action='store_true')
    return p


def main():
    p=parser();args=p.parse_args()
    logging.basicConfig(level=logging.INFO)
    if min(args.batch_size,args.steps,args.fsdp_devices)<1 or args.num_workers<0:
        p.error('Batch size, steps and FSDP devices must be positive; workers must be nonnegative.')
    if not (args.prepare_only or args.check_only):
        require_isaac_stopped()
        setup_nccl_runtime()
    prepared=args.prepared_root.resolve()
    if args.prepare_only:
        # The checker forces CPU only in its child process, never in training.
        if prepared.exists():
            raise FileExistsError(f'Snapshot already exists: {prepared}. Use --check-only or choose a new --prepared-root.')
        subprocess.run([sys.executable,str(ROOT/'scripts/check_openpi_dataset.py'),
            '--openpi-root',str(args.openpi_root), '--root',str(args.dataset_root),
            '--output',str(prepared),'--max-check-episodes','10'],check=True)
        print('Prepared all episodes and normalization statistics:', prepared, flush=True)
        return
    report_path=prepared/'validation.json'
    if not report_path.is_file():
        raise FileNotFoundError('Run this script with --prepare-only first.')
    report=json.loads(report_path.read_text())
    if not report.get('passed'):
        raise ValueError('The preparation report did not pass.')
    repo_id='local/f14_cube_pickplace'
    snapshot=prepared/'lerobot'/repo_id
    info=json.loads((snapshot/'meta/info.json').read_text())
    if info['total_episodes']!=report['episodes'] or info['total_frames']!=report['frames']:
        raise ValueError('Snapshot disagrees with its validation report.')
    os.environ['HF_LEROBOT_HOME']=str(prepared/'lerobot')
    os.environ.setdefault('OPENPI_DATA_HOME',str(ROOT/'outputs/openpi_cache'))
    # Leave memory for NCCL and the desktop on GPU 1. Reserving 85% up front
    # caused NCCL initialization to fail before even loading the model.
    os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
    os.environ.setdefault('XLA_PYTHON_CLIENT_MEM_FRACTION','0.75')
    if args.check_only:
        os.environ['JAX_PLATFORMS']='cpu'
        os.environ['XLA_PYTHON_CLIENT_PREALLOCATE']='false'
    sys.path.insert(0,str(args.openpi_root.resolve()/'src'))
    from openpi.models.pi0_config import Pi0Config
    from openpi.training import config, data_loader, weight_loaders
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
    from dataset.openpi_f14 import LeRobotF14DataConfig
    # Use the supported PyAV decoder, in this process only. TorchCodec in the
    # pinned checkout cannot load the installed FFmpeg shared libraries.
    data_loader.lerobot_dataset=SimpleNamespace(LeRobotDatasetMetadata=LeRobotDatasetMetadata,
        LeRobotDataset=partial(LeRobotDataset,video_backend='pyav'))
    model=Pi0Config(pi05=True,paligemma_variant='gemma_2b_lora',action_expert_variant='gemma_300m_lora')
    training=config.TrainConfig(name='pi05_f14_lora',exp_name=args.exp_name,
        model=model,freeze_filter=model.get_freeze_filter(),ema_decay=None,
        data=LeRobotF14DataConfig(repo_id=repo_id,
            assets=config.AssetsConfig(assets_dir=str(prepared/'assets'),asset_id='f14_cube_pickplace')),
        weight_loader=weight_loaders.CheckpointWeightLoader(args.base_params),
        assets_base_dir=str(ROOT/'outputs/openpi_training/assets'),
        checkpoint_base_dir=str(ROOT/'outputs/openpi_training/checkpoints'),
        batch_size=args.batch_size,num_workers=args.num_workers,num_train_steps=args.steps,
        fsdp_devices=args.fsdp_devices,wandb_enabled=args.wandb,resume=args.resume,
        log_interval=50,save_interval=1000,keep_period=5000)
    print(json.dumps({'policy':'pi05','finetune':'LoRA','episodes':report['episodes'],
        'frames':report['frames'],'robot_action_dim':16,'model_action_dim':model.action_dim,
        'action_horizon':model.action_horizon,'batch_size':training.batch_size,
        'steps':training.num_train_steps,'fsdp_devices':training.fsdp_devices,
        'gpu_preallocate':os.environ['XLA_PYTHON_CLIENT_PREALLOCATE'],
        'checkpoint_dir':str(training.checkpoint_dir),'base_params':args.base_params},indent=2),flush=True)
    if args.check_only:
        factory=training.data.create(training.assets_dirs,model)
        loader=data_loader.create_torch_data_loader(factory,model_config=model,
            action_horizon=model.action_horizon,batch_size=args.batch_size,num_workers=0,
            num_batches=1,shuffle=False,framework='jax')
        obs,actions=next(iter(loader))
        assert obs.state.shape==(args.batch_size,32) and actions.shape==(args.batch_size,50,32)
        result={'passed':True,'training_steps':0,'weights_loaded':False,
            'state_shape':list(obs.state.shape),'actions_shape':list(actions.shape),
            'images':{k:list(v.shape) for k,v in obs.images.items()},'mode':'pi05 LoRA input check'}
        (prepared/'training_config_check.json').write_text(json.dumps(result,indent=2)+'\n')
        print('TRAIN CONFIG CHECK PASSED',json.dumps(result,indent=2),flush=True)
        return
    import jax
    devices=jax.devices()
    if not devices or any(d.platform!='gpu' for d in devices):
        raise RuntimeError(f'GPU training required; JAX sees {devices}. Remove JAX_PLATFORMS=cpu.')
    if len(devices)%args.fsdp_devices or args.batch_size%len(devices):
        raise ValueError('FSDP devices must divide GPU count, and GPU count must divide batch size.')
    if training.checkpoint_dir.exists() and not args.resume:
        raise FileExistsError('Checkpoint directory exists; use --resume or a new --exp-name.')
    has_saved_steps = training.checkpoint_dir.exists() and any(
        entry.is_dir() and entry.name.isdecimal() for entry in training.checkpoint_dir.iterdir())
    if not (args.resume and has_saved_steps) and args.base_params.startswith('gs://openpi-assets/'):
        # These checkpoints are public. Avoid probing Google credentials on a
        # local workstation when upstream falls back from gsutil to gcsfs.
        from dataclasses import replace
        from dataset.openpi_download import public_checkpoint
        weight_path = public_checkpoint(args.base_params, os.environ['OPENPI_DATA_HOME'])
        training = replace(training, weight_loader=weight_loaders.CheckpointWeightLoader(str(weight_path)))
    spec=importlib.util.spec_from_file_location('f14_upstream_train',args.openpi_root/'scripts/train.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    patch_camera_preview(module)
    patch_sharded_initialization(module)
    original_init_wandb = module.init_wandb
    def init_wandb_with_existing_checkpoint(training_config, *, resuming, **kwargs):
        # An existing local checkpoint need not have an existing W&B run.
        original_init_wandb(training_config,
            resuming=resuming and (training_config.checkpoint_dir/'wandb_id.txt').is_file(), **kwargs)
    module.init_wandb = init_wandb_with_existing_checkpoint
    # Main uses init_logging() on the existing root logging handler.
    module.main(training)


def patch_camera_preview(module):
    """Fix the pinned upstream preview in memory, leaving its checkout intact.

    GPU indexing of a sharded image gathers it via NCCL. Transfer the whole
    batch to host first, and skip previews entirely when W&B is disabled.
    Fail explicitly if an upstream update changes the block we support.
    """
    import inspect
    old = '''    images_to_log = [
        wandb.Image(np.concatenate([np.array(img[i]) for img in batch[0].images.values()], axis=1))
        for i in range(min(5, len(next(iter(batch[0].images.values())))))
    ]
    wandb.log({"camera_views": images_to_log}, step=0)'''
    new = '''    if config.wandb_enabled:
        host_images = jax.device_get(batch[0].images)
        images_to_log = [
            wandb.Image(np.concatenate([img[i] for img in host_images.values()], axis=1))
            for i in range(min(5, len(next(iter(host_images.values())))))
        ]
        wandb.log({"camera_views": images_to_log}, step=0)'''
    source = inspect.getsource(module.main)
    if source.count(old) != 1:
        raise RuntimeError('OpenPI camera preview changed; review the F14 compatibility patch.')
    source = source.replace(old, new)
    metric_log = '            wandb.log(reduced_info, step=step)'
    if source.count(metric_log) != 1:
        raise RuntimeError('OpenPI metric logging changed; review the F14 compatibility patch.')
    source = source.replace(metric_log,
        '            _f14_append_metrics(config, reduced_info, step)\n' + metric_log)
    module._f14_append_metrics = append_training_metrics
    exec(compile(source, str(module.__file__), 'exec'), module.__dict__)


def append_training_metrics(config, metrics, step):
    """Persist scalar history independently of W&B, also across resumes."""
    from datetime import datetime, timezone
    row = {'step': int(step), 'time_utc': datetime.now(timezone.utc).isoformat(),
           **{name: float(value) for name, value in metrics.items()}}
    try:
        with (config.checkpoint_dir/'training_metrics.jsonl').open('a') as stream:
            stream.write(json.dumps(row, allow_nan=False)+'\n')
    except (OSError, ValueError) as exc:
        logging.warning('Could not save training metric history: %s', exc)


def patch_sharded_initialization(module):
    """Shard base weights on upload, avoiding a full copy on every GPU.

    Upstream uses replicated inputs to its FSDP initializer. That transient
    full checkpoint plus the sharded state exceeds 16GB on this workstation.
    Model parameters, optimizer, and training computation remain unchanged.
    """
    import inspect
    source = inspect.getsource(module.init_train_state)
    old = '        in_shardings=replicated_sharding,'
    new = '        in_shardings=(replicated_sharding, sharding.fsdp_sharding(partial_params, mesh)),'
    if source.count(old) != 1:
        raise RuntimeError('OpenPI initializer changed; review the F14 compatibility patch.')
    exec(compile(source.replace(old, new), str(module.__file__), 'exec'), module.__dict__)


def require_isaac_stopped(proc_root=Path('/proc')):
    """Avoid training alongside the receiver that uses both rendering GPUs."""
    for process in proc_root.iterdir():
        if not process.name.isdecimal() or int(process.name) == os.getpid():
            continue
        try:
            argv = (process / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(arg.rsplit(b'/', 1)[-1] == b'run_isaac_teleop.py' for arg in argv):
            raise RuntimeError(f'Isaac teleop receiver is running (PID {process.name}). '
                               'Stop Isaac and Quest with Ctrl+C before GPU training; '
                               'Isaac also uses the second GPU for rendering.')


def setup_nccl_runtime():
    """Preload the isolated Blackwell-compatible NCCL before importing CUDA.

    Torch pins NCCL 2.26.2 in the upstream environment. Its large collectives
    fail even in a model-free JAX test on these RTX 5080s. Keep that environment
    untouched and use the separately installed 2.31.2 library for this process.
    """
    library = ROOT / 'outputs/openpi_runtime/nvidia/nccl/lib/libnccl.so.2'
    if not library.is_file():
        raise FileNotFoundError('Install the isolated NCCL runtime first: uv pip install '
            '--python ../openpi/.venv/bin/python --target outputs/openpi_runtime '
            "'nvidia-nccl-cu12==2.31.2'")
    existing = os.environ.get('LD_PRELOAD', '')
    if str(library) not in existing.replace(':', ' ').split():
        os.environ['LD_PRELOAD'] = str(library) + (':' + existing if existing else '')
        # Preserve -u and other Python interpreter options on the one-time restart.
        os.execvpe(sys.executable, [sys.executable, *sys.orig_argv[1:]], os.environ)
    import ctypes
    runtime = ctypes.CDLL(str(library))
    version = ctypes.c_int()
    if runtime.ncclGetVersion(ctypes.byref(version)) != 0 or version.value != 23102:
        raise RuntimeError(f'Expected isolated NCCL 2.31.2; got version code {version.value}.')
    logging.info('Using isolated NCCL 2.31.2 runtime: %s', library)


if __name__=='__main__':
    main()
