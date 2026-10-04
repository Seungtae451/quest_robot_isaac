"""Nonblocking Isaac-side episode control. LeRobot only runs in a subprocess."""
from collections import deque
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import uuid

import numpy as np
import zmq

from config import recording_config as cfg
from robot.f14_config import ARM_JOINT_NAMES

JOINT_NAMES = ARM_JOINT_NAMES + ["left_gripper.open_ratio", "right_gripper.open_ratio"]
CAMERAS = ("front", "left_wrist", "right_wrist")


def features_for(shapes):
    return {
        "observation.state": {"dtype": "float32", "shape": (16,), "names": JOINT_NAMES},
        "action": {"dtype": "float32", "shape": (16,), "names": JOINT_NAMES},
        "sim_time": {"dtype": "float32", "shape": (1,), "names": ["seconds"]},
        **{f"observation.images.{name}": {"dtype": "video", "shape": tuple(shapes[name]),
                                        "names": ["height", "width", "channels"]} for name in CAMERAS},
    }


class EpisodeRecorder:
    def __init__(self, root, repo_id, task, fps, shapes, writer_python=cfg.WRITER_PYTHON):
        writer_python = Path(writer_python)
        if not writer_python.is_file():
            raise RuntimeError("LeRobot writer Python missing. Run scripts/setup_recording_env.sh first.")
        if not task.strip():
            raise ValueError("Dataset task cannot be empty")
        self.generation = uuid.uuid4().hex
        self.state = "INITIALIZING"
        self.frames = self.saved_episodes = 0
        self.notice = self.error = ""
        self.reset_requested = False
        self._reset_started = self._environment_ready = False
        self.session_id = None
        self.pending_observation = None
        self.shapes = shapes
        self.fps = fps
        self.start_allowed = False
        self.outbox = deque()
        self.started = time.monotonic()
        self.directory = tempfile.TemporaryDirectory(prefix="f14-recorder-")
        directory = Path(self.directory.name)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.setsockopt(zmq.SNDHWM, cfg.QUEUE_FRAMES)
        self.socket.setsockopt(zmq.RCVHWM, 128)
        self.socket.setsockopt(zmq.LINGER, 0)
        endpoint = f"ipc://{directory / 'writer.sock'}"
        self.socket.bind(endpoint)
        settings = {"root": str(Path(root).resolve()), "repo_id": repo_id, "task": task,
                    "fps": fps, "shapes": shapes, "endpoint": endpoint,
                    "features": features_for(shapes)}
        settings_path = directory / "settings.json"
        settings_path.write_text(json.dumps(settings))
        log_dir = cfg.ROOT / "outputs/recording"
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / f"writer_{self.generation}.log"
        self.log = self.log_path.open("w")
        # Do not inherit Kit's PYTHONPATH/LD_LIBRARY_PATH into a normal Python
        # process. The writer owns its environment, socket, codecs and I/O.
        environment = os.environ.copy()
        for key in ("PYTHONPATH", "LD_LIBRARY_PATH", "PYTHONHOME"):
            environment.pop(key, None)
        environment.update(HF_HUB_OFFLINE="1", HF_DATASETS_OFFLINE="1",
                           HF_HOME=str(cfg.ROOT / "outputs/hf_recording_cache"))
        self.process = subprocess.Popen(
            [str(writer_python), str(cfg.ROOT / "scripts/lerobot_writer.py"), "--config", str(settings_path)],
            stdout=self.log, stderr=subprocess.STDOUT, env=environment, cwd=cfg.ROOT,
            start_new_session=True)  # Terminal Ctrl+C must not interrupt a dataset commit.

    @property
    def blocks_teleop(self):
        return self.state != "RECORDING"

    def status(self):
        return {"state": self.state, "generation": self.generation, "frames": self.frames,
                "saved_episodes": self.saved_episodes, "notice": self.notice,
                "error": self.error, "session_id": self.session_id, "start_allowed": bool(self.start_allowed),
                "start_readiness": getattr(self, 'start_readiness', None)}

    def _command(self, operation):
        self.outbox.append([json.dumps({"op": operation, "session_id": self.session_id}).encode()])

    def fail(self, message):
        self.state, self.error = "ERROR", message
        self.pending_observation = None
        print(f"RECORDING ERROR: {message}. Pending data retained; X discards.", flush=True)

    def poll(self):
        while self.outbox:
            try:
                self.socket.send_multipart(self.outbox[0], flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            self.outbox.popleft()
        while True:
            try:
                message = self.socket.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            event = message["event"]
            if event == "ready" and self.state == "INITIALIZING":
                self.state = "READY"
                self.saved_episodes = message.get("saved_episodes", 0)
                print("Recording ready: A=start, B=stop, A=save / X=discard.", flush=True)
            elif event == "recording" and self.state == "STARTING":
                self.state = "RECORDING"
            elif event == "review" and self.state == "STOPPING":
                self.state = "REVIEW"
                self.frames = message["frames"]
            elif event in ("saved", "discarded") and self.state in ("SAVING", "DISCARDING"):
                self.saved_episodes = message.get("saved_episodes", self.saved_episodes)
                self.state = "RESETTING"
                self.reset_requested = not self._reset_started
                if self._environment_ready:
                    self.reset_complete()
                print(f"Episode {event}; HOME return and fresh cubes required before next A.", flush=True)
            elif event == "error":
                self.fail(message["error"])
        if self.process.poll() is not None and self.state != "ERROR":
            self.fail(f"Writer exited; see {self.log_path}")
        if self.state == "INITIALIZING" and time.monotonic() - self.started > 120:
            self.fail(f"Writer startup timed out; see {self.log_path}")

    def button(self, operation):
        self.notice = ""
        if operation == "a" and self.state == "READY":
            if not self.start_allowed:
                self.notice = "Wait for Quest calibration and stable HOME."
                return
            self.session_id = uuid.uuid4().hex
            self.frames = 0
            self.error = ""
            self._reset_started = self._environment_ready = False
            self.state = "STARTING"
            self._command("start")
        elif operation == "b" and self.state in ("STARTING", "RECORDING"):
            self.stop()
        elif operation == "a" and self.state == "REVIEW":
            if self.frames == 0:
                self.notice = "Empty episode: X discards; nothing to save."
                return
            self.state = "SAVING"
            # Final commit overlaps the slow HOME return; RGB encoding already
            # runs in the writer during teleoperation. Next A requires both.
            self.reset_requested = True
            self._command("save")
        elif operation == "x" and self.state in ("REVIEW", "ERROR") and self.session_id:
            if self.process.poll() is not None:
                self.notice = "Writer exited. Pending files retained; restart to recover."
                return
            self.state = "DISCARDING"
            self.error = ""
            self._command("discard")
        else:
            self.notice = {"READY": "A starts recording.", "RECORDING": "B stops recording.",
                           "REVIEW": "A saves; X discards."}.get(self.state, "Please wait.")

    def stop(self, notice=""):
        if self.state in ("STARTING", "RECORDING"):
            self.state, self.notice = "STOPPING", notice
            self.pending_observation = None
            self._command("stop")

    def capture(self, images, measured, applied_action, sim_time):
        """Pair o_t with the applied drive target at t+1/fps.

        Images/state at t are copied at their camera boundary; the action is
        the actual end-of-interval reference, not a distant, unsmoothed IK goal.
        The last unpaired observation is omitted on stop. No frame is invented
        or silently dropped to hide an overloaded writer.
        """
        if self.state != "RECORDING":
            self.pending_observation = None
            return
        if self.pending_observation is not None:
            old_images, old_measured, old_time = self.pending_observation
            try:
                if abs(sim_time - old_time - 1 / self.fps) > 1e-5:
                    raise ValueError("Nonuniform camera timestamps")
                vectors = np.concatenate((old_measured, applied_action, [old_time])).astype(np.float32)
                if vectors.shape != (33,) or not np.isfinite(vectors).all():
                    raise ValueError("Nonfinite or invalid recorded state/action")
                header = {"op": "frame", "session_id": self.session_id, "index": self.frames}
                self.socket.send_multipart([json.dumps(header).encode(), vectors.tobytes(),
                                            *(old_images[name].tobytes() for name in CAMERAS)], flags=zmq.NOBLOCK)
                self.frames += 1
            except (zmq.Again, ValueError) as exc:
                self.fail(f"Episode interrupted without committing: {exc or 'writer queue full'}")
                return
        self.pending_observation = ({k: v.copy() for k, v in images.items()},
                                    np.asarray(measured, dtype=np.float32).copy(), sim_time)

    def reset_started(self):
        self.reset_requested = False
        self._reset_started = True

    def reset_complete(self):
        self._environment_ready = True
        if self.state in ("SAVING", "DISCARDING", "ERROR"):
            return  # Physical reset is complete; wait for the writer outcome.
        self.generation = uuid.uuid4().hex
        self.state, self.reset_requested = "READY", False
        self.frames = 0
        self.session_id = None
        self.pending_observation = None

    def overlay(self):
        hints = {"READY": "HOME HOLD  A: START", "RECORDING": "B: STOP", "REVIEW": "A: SAVE  X: DISCARD",
                 "SAVING": "Finalizing in background / returning HOME",
                 "RESETTING": "Returning HOME / new cube"}
        return f"REC {self.state} frames={self.frames} saved={self.saved_episodes}  {hints.get(self.state, self.error or 'Please wait')}"

    def close(self):
        # Shutdown preserves any unconfirmed episode in a pending spool.
        if self.process.poll() is None:
            self._command("shutdown")
            until = time.monotonic() + 5
            while self.outbox and time.monotonic() < until:
                self.poll()
                time.sleep(.01)
            try:
                self.process.wait(timeout=120 if self.state == "SAVING" else 10)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=10)
        self.socket.close()
        self.context.term()
        self.log.close()
        self.directory.cleanup()
