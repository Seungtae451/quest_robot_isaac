"""Dedicated official LeRobot v3 exporter and lossless pending-episode spool.

This process imports no Isaac/Vuer/Pinocchio and never uploads to the Hub.
Only an explicit save commits an episode. B stops capture; X removes just the
current owned spool, leaving previously saved episodes intact.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import sys
import time
import traceback

import cv2
import numpy as np
import zmq

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from dataset.recording import CAMERAS, features_for


class LiveEncoder:
    """Keep official per-camera encoding parallel, with no silent frame loss.

    LeRobot 0.4.4 normally warns and drops frames on encoder overload. A
    training demonstration must instead fail, retaining its lossless spool.
    B drains and closes the videos; A reuses those prepared results.
    """
    def __init__(self, encoder):
        self.encoder = encoder
        self.counts = {}
        self.results = None

    def start_episode(self, **kwargs):
        self.counts = {key: 0 for key in kwargs["video_keys"]}
        self.results = None
        self.encoder.start_episode(**kwargs)

    def feed_frame(self, key, image):
        self.encoder.feed_frame(key, image)
        if any(self.encoder._dropped_frames.values()):
            raise RuntimeError("Video encoder overloaded; no incomplete episode will be committed")
        self.counts[key] += 1

    def finish_episode(self):
        if self.results is None:
            import av
            self.results = self.encoder.finish_episode()
            for key, (path, _) in self.results.items():
                with av.open(str(path)) as video:
                    frames = video.streams.video[0].frames
                    if not frames:
                        frames = sum(1 for _ in video.decode(video=0))
                if frames != self.counts[key]:
                    raise ValueError(f"Incomplete video {key}: {frames}/{self.counts[key]} frames")
        return self.results

    def close(self):
        self.encoder.close()
        # Prepared videos were not yet moved by save_episode (X or shutdown).
        if self.results is not None:
            for path, _ in self.results.values():
                if path.parent.exists():
                    shutil.rmtree(path.parent)

    def cancel_episode(self):
        self.close()


class Writer:
    def __init__(self, settings):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
        self.dataset_class, self.metadata_class = LeRobotDataset, LeRobotDatasetMetadata
        self.settings = settings
        self.root = Path(settings["root"])
        self.root.parent.mkdir(parents=True, exist_ok=True)
        self.lock = (self.root.parent / f".{self.root.name}.writer.lock").open("a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.lock.close()
            raise RuntimeError("This dataset is already being recorded by another writer")
        self.features = features_for(settings["shapes"])
        self.frames = 0
        self.session_id = None
        self.spool = None
        self.dataset = None
        self.write_root = None
        self.recording = False
        self.failed = False
        self.saved_episodes = 0
        if self.root.exists():
            meta = self.metadata_class(settings["repo_id"], root=self.root)
            self._validate(meta)
            self.saved_episodes = meta.total_episodes
        # Test encoder availability at startup, before recording any data.
        import av
        av.codec.Codec("libx264", "w")

    def _validate(self, meta):
        if meta.fps != self.settings["fps"] or meta.robot_type != "f14_isaac":
            raise ValueError("Existing dataset FPS/robot type mismatch; use a new dataset root")
        for key, feature in self.features.items():
            actual = meta.features.get(key)
            if actual is None or any(actual.get(k) != v for k, v in feature.items()
                                     if k != "shape") or tuple(actual["shape"]) != tuple(feature["shape"]):
                raise ValueError(f"Existing dataset feature mismatch: {key}")

    def start(self, session_id):
        if self.spool is not None:
            raise ValueError("Previous pending episode must be saved or discarded first")
        # Session IDs are locally generated hex UUIDs, never paths from the network.
        if len(session_id) != 32 or any(c not in "0123456789abcdef" for c in session_id):
            raise ValueError("Invalid session ID")
        self.spool = self.root.parent / f".{self.root.name}_pending" / session_id
        self.spool.mkdir(parents=True, exist_ok=False)
        self.session_id = session_id
        self.frames = 0
        self.failed = False
        self.recording = True
        (self.spool / "settings.json").write_text(json.dumps(self.settings, indent=2))
        # Build the official episode buffer NOW, not after A=save. Its video
        # threads encode during capture; the final root remains uncommitted.
        self.write_root = self.root if self.root.exists() else self.spool / "lerobot_commit"
        settings = self.settings
        if self.write_root.exists():
            self.dataset = self.dataset_class(settings["repo_id"], root=self.write_root,
                video_backend="pyav", vcodec="h264", streaming_encoding=True,
                encoder_queue_maxsize=30, encoder_threads=2)
            self._validate(self.dataset.meta)
        else:
            self.dataset = self.dataset_class.create(settings["repo_id"], fps=settings["fps"],
                root=self.write_root, robot_type="f14_isaac", features=self.features, use_videos=True,
                vcodec="h264", video_backend="pyav", metadata_buffer_size=1,
                streaming_encoding=True, encoder_queue_maxsize=30, encoder_threads=2)
        self.dataset._streaming_encoder = LiveEncoder(self.dataset._streaming_encoder)

    def add(self, header, parts):
        if not self.recording or header["session_id"] != self.session_id or header["index"] != self.frames:
            raise ValueError("Out-of-order frame / inactive episode")
        vectors = np.frombuffer(parts[0], dtype=np.float32)
        if vectors.shape != (33,) or not np.isfinite(vectors).all():
            raise ValueError("Invalid frame vectors")
        directory = self.spool / f"{self.frames:06d}"
        directory.mkdir()
        np.save(directory / "vectors.npy", vectors, allow_pickle=False)
        frame = {"observation.state": vectors[:16].copy(), "action": vectors[16:32].copy(),
                 "sim_time": vectors[32:].copy(), "task": self.settings["task"]}
        for name, payload in zip(CAMERAS, parts[1:], strict=True):
            rgb = np.frombuffer(payload, dtype=np.uint8).reshape(self.settings["shapes"][name])
            frame[f"observation.images.{name}"] = rgb
            if not cv2.imwrite(str(directory / f"{name}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                               [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                raise OSError(f"Failed to spool image {name}")
        self.dataset.add_frame(frame)
        self.frames += 1

    def stop(self):
        self.recording = False
        if self.frames and not self.failed:
            self.dataset._streaming_encoder.finish_episode()

    def save(self):
        if self.recording or not self.frames or self.failed:
            raise ValueError("Stop a nonempty, complete episode before saving")
        existing = self.root.exists()
        write_root = self.write_root
        save_started = time.monotonic()
        old_count = self.saved_episodes
        old_files = set()
        if existing:
            # The official resume API writes NEW data/video/episode files. Only
            # small global metadata is replaced, so a metadata snapshot suffices
            # to roll back an interrupted append without copying the recordings.
            shutil.copytree(self.root / "meta", self.spool / "meta_before_save")
            old_files = {p.relative_to(self.root) for section in ("data", "videos")
                         for p in (self.root / section).rglob("*") if p.is_file()}
        try:
            self._save_into(write_root)
            if not existing:
                os.replace(write_root, self.root)
        except Exception:
            self.saved_episodes = old_count
            if existing:
                os.replace(self.root / "meta", self.spool / "meta_failed_save")
                os.replace(self.spool / "meta_before_save", self.root / "meta")
                for section in ("data", "videos"):
                    for path in (self.root / section).rglob("*"):
                        if path.is_file() and path.relative_to(self.root) not in old_files:
                            path.unlink()
            raise
        self.discard()
        print(f"LeRobot commit complete in {time.monotonic() - save_started:.3f}s "
              "(videos encoded during recording)", flush=True)

    def _save_into(self, write_root):
        settings = self.settings
        dataset = self.dataset
        episode_index = dataset.meta.total_episodes
        try:
            dataset.save_episode(parallel_encoding=False)
        finally:
            dataset.finalize()  # v3 Parquet footers are required for reading.
            dataset.stop_image_writer()
            self.dataset = None
        # Verify that the official loader can read the committed episode before
        # acknowledging save or allowing the next recording to start.
        check = self.dataset_class(settings["repo_id"], root=write_root, video_backend="pyav")
        if check.meta.total_episodes != episode_index + 1:
            raise ValueError("Saved episode count mismatch")
        sample = check[check.meta.episodes[episode_index]["dataset_from_index"]]
        for key in self.features:
            if key not in sample:
                raise ValueError(f"Saved feature missing: {key}")
        first_vectors = np.load(self.spool / "000000/vectors.npy", allow_pickle=False)
        np.testing.assert_allclose(sample["observation.state"].numpy(), first_vectors[:16], atol=1e-6)
        np.testing.assert_allclose(sample["action"].numpy(), first_vectors[16:32], atol=1e-6)
        self.saved_episodes = check.meta.total_episodes
        metadata_dir = write_root / "meta/f14_sessions"
        metadata_dir.mkdir(exist_ok=True)
        manifest = {"session_id": self.session_id, "episode_index": episode_index, "frames": self.frames,
                    "task": settings["task"], "action_alignment": "o_t paired with applied drive target at t+1/fps",
                    "arm_units": "radians; rev 2.0.1 semantic signs",
                    "gripper_units": "normalized current F14 protocol; 0 physically closed, 1 open",
                    "image_source": "raw Isaac RGB; no operator overlay or JPEG stream",
                    "lerobot_version": "0.4.4", "dataset_version": "v3.0",
                    "encoding": "official per-camera streaming H264 threads during recording"}
        (metadata_dir / f"{self.session_id}.json").write_text(json.dumps(manifest, indent=2))

    def discard(self):
        self.recording = False
        self.close()
        if self.spool is not None:
            shutil.rmtree(self.spool)
        self.spool = self.session_id = None
        self.frames = 0
        self.failed = False

    def close(self):
        if self.dataset is not None:
            # Before A there are no official Parquet writes; close only stops
            # encoders and releases their owned temporary video directories.
            self.dataset.finalize()
            self.dataset.stop_image_writer()
            self.dataset = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    settings = json.loads(args.config.read_text())
    context = zmq.Context()
    socket = context.socket(zmq.PAIR)
    socket.setsockopt(zmq.LINGER, 1000)
    socket.connect(settings["endpoint"])
    writer = None
    try:
        writer = Writer(settings)
        socket.send_json({"event": "ready", "saved_episodes": writer.saved_episodes})
        while True:
            parts = socket.recv_multipart()
            header = json.loads(parts[0])
            operation = header["op"]
            try:
                if operation == "shutdown":
                    break  # pending spool is deliberately retained
                if operation == "start":
                    writer.start(header["session_id"])
                    socket.send_json({"event": "recording"})
                elif operation == "frame":
                    if not writer.failed:
                        writer.add(header, parts[1:])
                elif operation == "stop":
                    writer.stop()
                    socket.send_json({"event": "review", "frames": writer.frames})
                elif operation == "save":
                    writer.save()
                    socket.send_json({"event": "saved", "saved_episodes": writer.saved_episodes})
                elif operation == "discard":
                    writer.discard()
                    socket.send_json({"event": "discarded", "saved_episodes": writer.saved_episodes})
                else:
                    raise ValueError(f"Unknown writer command: {operation}")
            except Exception as exc:
                writer.failed = True
                traceback.print_exc()
                socket.send_json({"event": "error", "error": str(exc)})
    except Exception as exc:
        traceback.print_exc()
        socket.send_json({"event": "error", "error": str(exc)})
    finally:
        if writer is not None:
            writer.close()
        socket.close()
        context.term()


if __name__ == "__main__":
    main()
