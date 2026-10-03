"""Check saved F14 data with the installed OpenPI pi05 training loader on CPU.

Run with ../openpi/.venv/bin/python. Original v3 files and OpenPI dependencies
are unchanged. An owned v2.1 compatibility copy, norm stats, and a JSON report
are written to outputs/openpi_check/<run>. No policy weights are downloaded.
"""
import argparse
import copy
from functools import partial
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from types import SimpleNamespace
import urllib.request

import numpy as np
import pyarrow.compute as pc
import pyarrow.parquet as pq

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def fingerprint(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob("*")) if p.is_file()}


def write_lines(path, rows):
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))


def export_v21(source, target):
    """Split Parquet by episode; copy already separate videos without transcoding."""
    from lerobot.common.datasets.utils import create_empty_dataset_info, get_hf_features_from_features
    info = json.loads((source / "meta/info.json").read_text())
    if info["codebase_version"] != "v3.0":
        raise ValueError("This exporter expects the F14 LeRobot v3.0 source")
    rows = [row for path in sorted((source / "meta/episodes").rglob("*.parquet"))
            for row in pq.read_table(path).to_pylist()]
    rows.sort(key=lambda row: row["episode_index"])
    if len(rows) != info["total_episodes"]:
        raise ValueError("Dataset changed or episode metadata is incomplete")
    target.mkdir(parents=True, exist_ok=False)
    (target / "meta").mkdir()
    result = create_empty_dataset_info("v2.1", info["fps"], info["robot_type"], copy.deepcopy(info["features"]), True)
    # datasets 4.x writes a 'List' descriptor which the pinned datasets 3.x
    # cannot parse. Rebuild just the Arrow metadata using official v2 features;
    # all numeric values and dtypes remain the same.
    hf_features = get_hf_features_from_features({key: {**value, "shape": tuple(value["shape"])}
                                               for key, value in info["features"].items()})
    tasks = pq.read_table(source / "meta/tasks.parquet").to_pandas()
    write_lines(target / "meta/tasks.jsonl", [{"task_index": int(row["task_index"]), "task": str(task)}
                                             for task, row in tasks.iterrows()])
    episodes, stats, states, actions = [], [], [], []
    import av
    for index, row in enumerate(rows):
        if row["episode_index"] != index:
            raise ValueError("Episodes must be contiguous from zero")
        source_data = source / info["data_path"].format(chunk_index=row["data/chunk_index"],
                                                       file_index=row["data/file_index"])
        table = pq.read_table(source_data)
        table = table.filter(pc.equal(table["episode_index"], index))
        if len(table) != row["length"]:
            raise ValueError(f"Episode {index} frame count mismatch")
        chunk = index // result["chunks_size"]
        data_path = target / result["data_path"].format(episode_chunk=chunk, episode_index=index)
        data_path.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(table.cast(hf_features.arrow_schema), data_path)
        for key, feature in info["features"].items():
            if feature["dtype"] != "video":
                continue
            prefix = f"videos/{key}/"
            path = source / info["video_path"].format(video_key=key,
                chunk_index=row[prefix + "chunk_index"], file_index=row[prefix + "file_index"])
            with av.open(str(path)) as video:
                frame_count = video.streams.video[0].frames
            if abs(row[prefix + "from_timestamp"]) > 1e-6 or frame_count != len(table):
                raise ValueError("Shared video segments need slicing; this source is not a per-episode F14 video")
            video_path = target / result["video_path"].format(episode_chunk=chunk, video_key=key, episode_index=index)
            video_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, video_path)
        episodes.append({"episode_index": index, "tasks": row["tasks"], "length": len(table)})
        episode_stats = {}
        for key, value in row.items():
            if key.startswith("stats/"):
                _, feature, statistic = key.split("/", 2)
                values = np.asarray(value)
                if "image" in feature and statistic != "count":
                    values = values.reshape(3, 1, 1)
                episode_stats.setdefault(feature, {})[statistic] = values.tolist()
        stats.append({"episode_index": index, "stats": episode_stats})
        states.append(np.asarray(table["observation.state"].to_pylist(), np.float32))
        actions.append(np.asarray(table["action"].to_pylist(), np.float32))
    result.update(total_episodes=len(rows), total_frames=sum(row["length"] for row in rows),
                  total_tasks=info["total_tasks"], total_videos=3 * len(rows),
                  total_chunks=(len(rows) + result["chunks_size"] - 1) // result["chunks_size"],
                  splits={"train": f"0:{len(rows)}"})
    (target / "meta/info.json").write_text(json.dumps(result, indent=2))
    write_lines(target / "meta/episodes.jsonl", episodes)
    write_lines(target / "meta/episodes_stats.jsonl", stats)
    return info, episodes, np.concatenate(states), np.concatenate(actions)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--openpi-root", type=Path, default=ROOT.parent / "openpi")
    parser.add_argument("--root", type=Path, default=ROOT / "datasets/f14_cube_pickplace")
    parser.add_argument("--repo-id", default="local/f14_cube_pickplace")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/openpi_check" / time.strftime("%Y%m%d_%H%M%S"))
    args = parser.parse_args()
    source, output = args.root.resolve(), args.output.resolve()
    if not (args.openpi_root / "src/openpi/training/data_loader.py").is_file():
        raise FileNotFoundError(f"OpenPI checkout missing: {args.openpi_root}")
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(JAX_PLATFORMS="cpu", XLA_PYTHON_CLIENT_PREALLOCATE="false",
        HF_HOME=str(output / "hf_cache"), HF_LEROBOT_HOME=str(output / "lerobot"),
        HF_HUB_OFFLINE="1", HF_DATASETS_OFFLINE="1", OPENPI_DATA_HOME=str(ROOT / "outputs/openpi_cache"))
    sys.path.insert(0, str(args.openpi_root.resolve() / "src"))
    from lerobot.common.datasets.lerobot_dataset import LeRobotDatasetMetadata, LeRobotDataset
    from openpi.models.pi0_config import Pi0Config
    from openpi.shared import normalize
    from openpi.training import config, data_loader
    from dataset.openpi_f14 import LeRobotF14DataConfig, F14Inputs, F14Outputs
    report = {"source": str(source), "openpi_root": str(args.openpi_root.resolve()), "policy": "pi05",
              "weights_loaded": False, "training_steps": 0, "device": "cpu"}
    before = fingerprint(source)
    try:
        LeRobotDatasetMetadata(args.repo_id, root=source)
    except Exception as exc:
        report["unmodified_loader_error"] = f"{type(exc).__name__}: {exc}"
    target = output / "lerobot" / args.repo_id
    info, episodes, states, actions = export_v21(source, target)
    for name, array in (("state", states), ("actions", actions)):
        if array.shape != (info["total_frames"], 16) or not np.isfinite(array).all():
            raise ValueError(f"Invalid {name} vectors")
    # A tiny public tokenizer is required for a REAL pi05 model transform.
    # Cache it in this project; never download the multi-GB model checkpoint.
    tokenizer = ROOT / "outputs/openpi_cache/big_vision/paligemma_tokenizer.model"
    if not tokenizer.is_file():
        tokenizer.parent.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen("https://storage.googleapis.com/big_vision/paligemma_tokenizer.model", timeout=30) as response:
            temporary = tokenizer.with_suffix(".partial")
            temporary.write_bytes(response.read())
            temporary.replace(tokenizer)
    norm_stats = {}
    for name, array in (("state", states), ("actions", actions)):
        tracker = normalize.RunningStats()
        tracker.update(array.astype(np.float64))
        norm_stats[name] = tracker.get_statistics()
    assets = output / "assets/f14_cube_pickplace"
    normalize.save(assets, norm_stats)
    model = Pi0Config(pi05=True)
    factory = LeRobotF14DataConfig(repo_id=args.repo_id,
        assets=config.AssetsConfig(assets_dir=str(output / "assets"), asset_id="f14_cube_pickplace"))
    data_config = factory.create(output / "assets", model)
    # The pinned loader doesn't expose video_backend in DataConfig. Use the
    # official constructor with its supported PyAV argument in this process
    # only; the actual OpenPI loader/transforms and video content are unchanged.
    data_loader.lerobot_dataset = SimpleNamespace(LeRobotDatasetMetadata=LeRobotDatasetMetadata,
        LeRobotDataset=partial(LeRobotDataset, video_backend="pyav"))
    raw = data_loader.create_torch_dataset(data_config, model.action_horizon, model)
    transformed = data_loader.transform_dataset(raw, data_config)
    checked = []
    offset = 0
    for episode in episodes:
        for index in (offset, offset + episode["length"] // 2, offset + episode["length"] - 1):
            sample = raw[index]
            np.testing.assert_allclose(sample["observation.state"], states[index], atol=1e-7)
            expected = actions[np.minimum(np.arange(index, index + model.action_horizon), offset + episode["length"] - 1)]
            np.testing.assert_allclose(sample["action"], expected, atol=1e-7)
            assert sample["prompt"] == episode["tasks"][0]
            value = transformed[index]
            assert value["state"].shape == (32,) and value["actions"].shape == (50, 32)
            assert np.isfinite(value["state"]).all() and np.isfinite(value["actions"]).all()
            assert np.count_nonzero(value["state"][16:]) == 0 and np.count_nonzero(value["actions"][:, 16:]) == 0
            for image in value["image"].values():
                assert image.shape == (224, 224, 3) and image.dtype == np.uint8 and float(image.std()) > 1.
            checked.append(index)
        offset += episode["length"]
    loader = data_loader.create_torch_data_loader(data_config, model_config=model,
        action_horizon=model.action_horizon, batch_size=2, num_workers=0, num_batches=1,
        shuffle=False, framework="jax")
    observation, action_batch = next(iter(loader))
    observation_spec, action_spec = model.inputs_spec(batch_size=2)
    assert observation.state.shape == observation_spec.state.shape
    assert action_batch.shape == action_spec.shape
    for key in observation.images:
        assert observation.images[key].shape == observation_spec.images[key].shape
        assert bool(np.asarray(observation.image_masks[key]).all())
    assert observation.tokenized_prompt.shape == observation_spec.tokenized_prompt.shape
    assert bool(np.asarray(observation.tokenized_prompt_mask).any())
    # Quantile normalize -> policy padding -> denormalize -> physical 16D labels.
    restored = normalize.load(assets)
    from openpi import transforms
    sample = raw[0]
    robot = F14Inputs()(data_config.repack_transforms.inputs[0](sample))
    normalized = transforms.Normalize(restored, use_quantiles=True)(robot)
    padded = transforms.PadStatesAndActions(32)(normalized)
    decoded = F14Outputs()(transforms.Unnormalize(restored, use_quantiles=True)(padded))
    np.testing.assert_allclose(decoded["actions"], actions[:50], atol=1e-6)
    assert before == fingerprint(source), "Source dataset changed during the check"
    report.update(passed=True, source_unchanged=True, source_version=info["codebase_version"],
        compatibility_version="v2.1", compatibility_root=str(target), episodes=len(episodes),
        frames=info["total_frames"], fps=info["fps"], checked_indices=checked,
        model_action_dim=32, robot_action_dim=16, action_horizon=50, video_backend="pyav",
        state_batch_shape=list(observation.state.shape), action_batch_shape=list(action_batch.shape),
        images={k:list(v.shape) for k,v in observation.images.items()},
        tokenized_prompt_shape=list(observation.tokenized_prompt.shape), norm_stats=str(assets / "norm_stats.json"),
        action_roundtrip=True, task=episodes[0]["tasks"][0],
        openpi_commit=subprocess.check_output(["git", "-C", str(args.openpi_root), "rev-parse", "HEAD"], text=True).strip())
    (output / "validation.json").write_text(json.dumps(report, indent=2))
    print("OPENPI PI05 DATA CHECK PASSED\n" + json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
