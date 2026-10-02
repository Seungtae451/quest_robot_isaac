"""Latest-frame JPEG transport, independent of both Isaac and TeleVuer imports.

Isaac copies RGB observation tensors at camera rate and composes a dashboard.
A worker with ONE replaceable pending frame owns its ZMQ PUB socket and JPEG
encoder; encoding and socket backpressure therefore cannot stall physics.
Quest SUB uses CONFLATE with a single-part message (header + JPEG), decodes
only the newest frame, then the TeleVuer adapter supplies BGR to render_to_xr.
"""
import struct
import threading
import time

import cv2
import numpy as np
import zmq

from config import teleop_config as cfg

VIDEO_HEADER = struct.Struct("!4sQd")


class VideoPublisher:
    def __init__(self, endpoint: str):
        self.endpoint = endpoint
        self.condition = threading.Condition()
        self.pending = None
        self.stopping = False
        self.started = threading.Event()
        self.error = None
        self.frames = 0
        self.thread = threading.Thread(target=self._run, name="quest-video-publisher", daemon=True)
        self.thread.start()
        if not self.started.wait(5.):
            self.close()
            raise RuntimeError("Video publisher startup timed out")
        if self.error:
            self.close()
            raise RuntimeError(f"Video publisher startup failed: {self.error}")

    def submit(self, rgb: np.ndarray, timestamp: float):
        """Transfer ownership of a newly allocated composite; never mutate it."""
        if self.error:
            return
        with self.condition:
            self.pending = (rgb, timestamp)
            self.condition.notify()

    def _run(self):
        context = zmq.Context()
        sock = context.socket(zmq.PUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.SNDHWM, 1)
        sock.setsockopt(zmq.CONFLATE, 1)
        try:
            sock.bind(self.endpoint)
            self.started.set()
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.stopping or self.pending is not None)
                    if self.stopping:
                        break
                    rgb, timestamp = self.pending
                    self.pending = None
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, cfg.JPEG_QUALITY])
                if not ok:
                    continue
                self.frames += 1
                payload = VIDEO_HEADER.pack(b"F14V", self.frames, timestamp) + encoded.tobytes()
                try:
                    sock.send(payload, flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass  # Drop immediately; never wait for a slow headset.
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


class VideoSubscriber:
    def __init__(self, endpoint: str):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.socket.connect(endpoint)
        self.timestamp = 0.
        self.sequence = 0
        self.frames = 0

    def receive(self):
        """Return one fresh BGR dashboard, or None without waiting."""
        try:
            payload = self.socket.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            return None
        if len(payload) <= VIDEO_HEADER.size:
            return None
        magic, sequence, timestamp = VIDEO_HEADER.unpack_from(payload)
        if magic != b"F14V" or not np.isfinite(timestamp) or not 0 <= time.monotonic() - timestamp <= cfg.VIDEO_TIMEOUT:
            return None
        bgr = cv2.imdecode(np.frombuffer(payload, np.uint8, offset=VIDEO_HEADER.size), cv2.IMREAD_COLOR)
        if bgr is None or bgr.shape != (cfg.QUEST_VIEW_HEIGHT, cfg.QUEST_VIEW_WIDTH, 3):
            return None
        self.sequence, self.timestamp = sequence, timestamp
        self.frames += 1
        return bgr

    @property
    def age(self):
        return time.monotonic() - self.timestamp if self.timestamp else float("inf")

    def close(self):
        self.socket.close(linger=0)
        self.context.term()


def send_image_to_xr(tv, bgr: np.ndarray):
    """Installed TeleVuer's render_to_xr takes BGR and internally converts RGB.

Keeping this boundary explicit prevents a double RGB/BGR swap. A future
wrapper with another API should be adapted here, not throughout the app.
"""
    tv.render_to_xr(np.ascontiguousarray(bgr))


def waiting_image(message="Waiting for Isaac camera frames"):
    image = np.zeros((cfg.QUEST_VIEW_HEIGHT, cfg.QUEST_VIEW_WIDTH, 3), np.uint8)
    cv2.putText(image, message, (55, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    return image
