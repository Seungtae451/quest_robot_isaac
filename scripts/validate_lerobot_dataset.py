"""Reload a local F14 LeRobot dataset and validate episodes, RGB and labels."""
import argparse
import json
import os
from pathlib import Path

import numpy as np


def main():
    # Keep standalone validation cache local too, including in a sandbox.
    os.environ.setdefault("HF_HOME", str(Path(__file__).resolve().parents[1] / "outputs/hf_recording_cache"))
    os.environ.setdefault("HF_HUB_OFFLINE", "1")
    os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.datasets.utils import dataset_to_policy_features
    from lerobot.configs.types import FeatureType
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo-id", default="local/f14_cube_pickplace")
    args = parser.parse_args()
    ds = LeRobotDataset(args.repo_id, root=args.root, video_backend="pyav")
    features = dataset_to_policy_features(ds.meta.features)
    assert tuple(features["observation.state"].shape) == (16,)
    assert tuple(features["action"].shape) == (16,)
    assert len([v for v in features.values() if v.type == FeatureType.VISUAL]) == 3
    result = {"episodes": ds.meta.total_episodes, "frames": ds.meta.total_frames, "fps": ds.meta.fps,
              "images": {}, "task": ds[0]["task"]}
    for key in ("observation.state", "action"):
        assert tuple(ds[0][key].shape) == (16,)
        assert np.isfinite(ds.hf_dataset[key]).all()
        assert "q01" in ds.meta.stats[key] and "q99" in ds.meta.stats[key]
    for i in range(ds.meta.total_episodes):
        episode = ds.meta.episodes[i]
        start, end = episode["dataset_from_index"], episode["dataset_to_index"]
        assert end - start == episode["length"]
        assert abs(float(ds[start]["timestamp"])) < 1e-6
        assert int(ds[end - 1]["episode_index"]) == i
        for name in ("front", "left_wrist", "right_wrist"):
            key = f"observation.images.{name}"
            image = ds[start][key]
            assert image.ndim == 3 and image.shape[0] == 3
            assert np.isfinite(image.numpy()).all() and float(image.std()) > .001
            result["images"][name] = list(image.shape)
    result["passed"] = True
    result["pi05_data_features_valid"] = True
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
