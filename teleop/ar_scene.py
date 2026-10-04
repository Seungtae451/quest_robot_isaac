"""Read-only, latest-state scene transport, separate from action and RGB data."""
import json
import threading
import time

import numpy as np
import zmq

from config import ar_config

MAX_SCENE_BYTES = 65536


def validate_scene(value):
    if not isinstance(value, dict) or value.get("version") != 1:
        raise ValueError("Unknown scene protocol")
    for key in ("timestamp", "boot_time"):
        if not isinstance(value.get(key), (float, int)) or not np.isfinite(value[key]):
            raise ValueError("Invalid scene timestamp")
    links = value.get("links")
    if not isinstance(links, dict) or not 1 <= len(links) <= 128:
        raise ValueError("Invalid links")
    for pose in list(links.values()) + value.get("cubes", []):
        a = np.asarray(pose, dtype=float)
        if (a.shape != (7,) or not np.isfinite(a).all()
                or abs(np.linalg.norm(a[3:]) - 1.) > .001):
            raise ValueError("Expected xyz + wxyz pose")
    if not all(name in links for name in ("left_dof7_link", "right_dof7_link")):
        raise ValueError("Measured wrists missing")
    return value


class ScenePublisher:
    """A replaceable snapshot; encoding/socket work never runs in physics."""
    def __init__(self, endpoint=ar_config.SCENE_ENDPOINT):
        self.endpoint = endpoint
        self.condition = threading.Condition()
        self.pending = None
        self.stopping = False
        self.error = None
        self.started = threading.Event()
        self.thread = threading.Thread(target=self._run, name="quest-ar-scene", daemon=True)
        self.thread.start()
        if not self.started.wait(5.) or self.error:
            self.close()
            raise RuntimeError(f"Scene publisher startup failed: {self.error}")

    def submit(self, snapshot):
        with self.condition:
            self.pending = snapshot
            self.condition.notify()

    def _run(self):
        context = zmq.Context()
        sock = context.socket(zmq.PUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.SNDHWM, 1)
        try:
            sock.bind(self.endpoint)
            self.started.set()
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.stopping or self.pending is not None)
                    if self.stopping:
                        break
                    snapshot, self.pending = self.pending, None
                payload = json.dumps(snapshot, allow_nan=False, separators=(",", ":")).encode()
                if len(payload) > MAX_SCENE_BYTES:
                    raise ValueError("Scene exceeds transport limit")
                try:
                    sock.send(payload, flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.started.set()
            sock.close(linger=0)
            context.term()

    def close(self):
        with self.condition:
            self.stopping = True
            self.condition.notify()
        self.thread.join(timeout=3.)


class SceneSubscriber:
    def __init__(self, endpoint=ar_config.SCENE_ENDPOINT):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.socket.connect(endpoint)
        self.latest = None

    def receive(self):
        try:
            packet = self.socket.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            return None
        try:
            if len(packet) > MAX_SCENE_BYTES:
                return None
            value = validate_scene(json.loads(packet))
            if not 0 <= time.monotonic() - value['timestamp'] <= ar_config.SCENE_TIMEOUT:
                return None
            self.latest = value
            return value
        except (ValueError, TypeError, KeyError):
            return None

    def close(self):
        self.socket.close(linger=0)
        self.context.term()
