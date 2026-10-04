"""Resolve completed local checkpoints without loading JAX or Isaac."""
import json
from pathlib import Path


def committed(path):
    try:
        metadata = json.loads((path/'_CHECKPOINT_METADATA').read_text())
        return bool(metadata.get('commit_timestamp_nsecs')) and (path/'params').is_dir()
    except (OSError, ValueError):
        return False


def resolve_checkpoint(value, experiment):
    experiment = Path(experiment).resolve()
    if str(value) == 'latest':
        candidates = [path for path in experiment.iterdir()
                      if path.is_dir() and path.name.isdigit() and committed(path)]
        if not candidates:
            raise FileNotFoundError(f'No completed checkpoint in {experiment}')
        return max(candidates, key=lambda path: int(path.name))
    path = Path(value).expanduser().resolve()
    if not committed(path):
        raise ValueError(f'Not a completed OpenPI checkpoint: {path}')
    return path
