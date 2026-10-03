"""Acknowledged, deduplicated local episode buttons, separate from arm UDP.

Right A=start/save, right B=stop, left X=discard. Generation rejects delayed
buttons from a previous episode. Retries never repeat a save or a reset.
"""
import json
import socket
import time
import uuid
from collections import deque

REQUEST = b"F14E"
REPLY = b"F14e"
MAX_PACKET = 8192


def encode(magic, payload):
    return magic + json.dumps(payload, allow_nan=False, separators=(",", ":")).encode()


class EpisodeServer:
    def __init__(self, host, port, recorder):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, port))
        self.socket.setblocking(False)
        self.recorder = recorder
        self.seen = {}

    def poll(self):
        while True:
            try:
                packet, peer = self.socket.recvfrom(MAX_PACKET + 1)
            except BlockingIOError:
                break
            try:
                if len(packet) > MAX_PACKET or not packet.startswith(REQUEST):
                    continue
                request = json.loads(packet[4:])
                request_id = request.get("id")
                if request.get("op") != "status":
                    if not isinstance(request_id, str) or len(request_id) > 100:
                        continue
                    if request_id not in self.seen:
                        generation = request.get("generation")
                        if generation == self.recorder.generation:
                            self.recorder.button(request.get("op"))
                        self.seen[request_id] = True
                        if len(self.seen) > 256:
                            self.seen.pop(next(iter(self.seen)))
                response = {**self.recorder.status(), "ack": request_id}
                self.socket.sendto(encode(REPLY, response), peer)
            except (ValueError, TypeError, AttributeError, OSError):
                continue

    def close(self):
        self.socket.close()


class EpisodeClient:
    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.setblocking(False)
        self.destination = host, port
        self.client_id = uuid.uuid4().hex
        self.sequence = 0
        self.pending = None
        self.queued = deque()
        self.status = None
        self.last_status = self.last_query = self.last_send = 0.

    def poll(self):
        now = time.monotonic()
        if self.pending is None and self.queued:
            self.pending = self.queued.popleft()
            self.last_send = 0.
        if now - self.last_query >= .1:
            self.socket.sendto(encode(REQUEST, {"op": "status"}), self.destination)
            self.last_query = now
        if self.pending and now - self.last_send >= .1:
            self.socket.sendto(encode(REQUEST, self.pending), self.destination)
            self.last_send = now
        while True:
            try:
                packet, peer = self.socket.recvfrom(MAX_PACKET + 1)
            except BlockingIOError:
                break
            if peer != self.destination or not packet.startswith(REPLY):
                continue
            try:
                response = json.loads(packet[4:])
                if not isinstance(response.get("state"), str):
                    continue
            except (ValueError, AttributeError):
                continue
            self.status, self.last_status = response, now
            if self.pending and response.get("ack") == self.pending["id"]:
                self.pending = None

    @property
    def available(self):
        return self.status is not None and time.monotonic() - self.last_status < 1.

    @property
    def blocks_teleop(self):
        # Losing a previously connected recording server must not resume arms.
        return self.status is not None and (not self.available or self.status["state"] != "RECORDING")

    @property
    def preparing_episode(self):
        """Collect neutral tracking without permitting any operator motion."""
        return self.available and self.status["state"] in ("READY", "STARTING")

    def button(self, operation):
        if not self.available or len(self.queued) >= 8:
            return False
        self.sequence += 1
        request = {"id": f"{self.client_id}:{self.sequence}", "op": operation,
                   "generation": self.status["generation"]}
        if self.pending:
            self.queued.append(request)
        else:
            self.pending = request
        self.last_send = 0.
        return True

    def close(self):
        self.socket.close()
