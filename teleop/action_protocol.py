"""Local UDP action transport shared by the two independent processes.

80-byte network-endian packet: !4sId16f = F16A, uint32 sequence,
float64 monotonic timestamp (seconds on this host), 16 float32 actions.
Actions: left q[7] radians, right q[7] radians, left/right closure [0,1].
Timestamps reject queued/reordered/replayed packets even across sender restarts.

A read-only F16? query returns F16S + receiver boot timestamp + measured state.
It lets a restarted Quest sender calibrate at the current robot pose and lets a
restarted simulator require a new calibration, without commanding a HOME jump.
It is NOT another action schema or a dataset observation.
"""
from dataclasses import dataclass
import socket
import struct
import time

import numpy as np

from config.teleop_config import COMMAND_TIMEOUT, FEEDBACK_HZ
from robot.f14_config import HOME_ACTION

MAGIC = b"F16A"
HOLD_MAGIC = b"F16H"  # same packet shape; brake arms while allowing gripper commands
PACKET_FMT = "!4sId16f"
PACKET = struct.Struct(PACKET_FMT)
PACKET_SIZE = PACKET.size
QUERY = b"F16?"
STATUS = struct.Struct("!4sd16f")


def validate_action(action) -> np.ndarray:
    action = np.asarray(action, dtype=np.float32)
    if action.shape != (16,) or not np.isfinite(action).all():
        raise ValueError("Action must contain exactly 16 finite float32 values")
    if np.any((action[14:] < 0.) | (action[14:] > 1.)):
        raise ValueError("Gripper closure must lie in [0, 1]")
    return action


@dataclass(frozen=True)
class ActionPacket:
    sequence: int
    timestamp: float
    action: np.ndarray
    hold_arms: bool = False


def pack_action(action, sequence: int, timestamp: float | None = None, *, hold_arms=False) -> bytes:
    action = validate_action(action)
    timestamp = time.monotonic() if timestamp is None else timestamp
    if not np.isfinite(timestamp) or timestamp < 0:
        raise ValueError("Invalid monotonic timestamp")
    return PACKET.pack(HOLD_MAGIC if hold_arms else MAGIC, sequence & 0xffffffff, timestamp, *action)


def unpack_action(payload: bytes) -> ActionPacket:
    if len(payload) != PACKET_SIZE:
        raise ValueError(f"Expected {PACKET_SIZE} bytes, got {len(payload)}")
    magic, sequence, timestamp, *action = PACKET.unpack(payload)
    if magic not in (MAGIC, HOLD_MAGIC) or not np.isfinite(timestamp) or timestamp < 0:
        raise ValueError("Bad action magic or timestamp")
    return ActionPacket(sequence, timestamp, validate_action(action), magic == HOLD_MAGIC)


class ActionReceiver:
    """Nonblocking receive queue drain; rejected packets never refresh HOLD."""

    def __init__(self, host: str, port: int, arm_limits=None):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((host, port))
            self.socket.setblocking(False)
        except BaseException:
            self.socket.close()
            raise
        self.arm_limits = arm_limits
        self.boot_time = time.monotonic()
        self.action = HOME_ACTION.astype(np.float32).copy()
        self.feedback_action = self.action.copy()
        self.latest = None
        self.rejected = 0
        self.accept_after = 0.

    def poll(self) -> bool:
        changed = False
        while True:
            try:
                payload, peer = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            if payload == QUERY:
                try:
                    self.socket.sendto(STATUS.pack(b"F16S", self.boot_time, *self.feedback_action), peer)
                except OSError:
                    pass
                continue
            try:
                packet = unpack_action(payload)
                age = time.monotonic() - packet.timestamp
                if not 0. <= age <= COMMAND_TIMEOUT:
                    raise ValueError("Stale or future command")
                if packet.timestamp <= self.accept_after:
                    raise ValueError("Command belongs to an earlier episode")
                if self.latest is not None and packet.timestamp <= self.latest.timestamp:
                    raise ValueError("Out-of-order/duplicate command")
                if self.arm_limits is not None and not packet.hold_arms:
                    low, high = self.arm_limits[:, 0], self.arm_limits[:, 1]
                    if np.any(packet.action[:14] < low - 1e-5) or np.any(packet.action[:14] > high + 1e-5):
                        raise ValueError("Arm command violates USD joint limits")
            except ValueError:
                self.rejected += 1
                continue
            self.latest = packet
            self.action = packet.action.copy()
            changed = True
        return changed

    @property
    def arms_enabled(self) -> bool:
        return self.state == "ACTIVE" and not self.latest.hold_arms

    def update_feedback(self, measured_action):
        self.feedback_action = validate_action(measured_action).copy()

    def reset_session(self, measured_action):
        """Fence old UDP targets and force Quest to calibrate after episode reset.

        This changes protocol state only; it never writes physical joint state.
        """
        self.boot_time = self.accept_after = time.monotonic()
        self.latest = None
        self.action = validate_action(measured_action).copy()
        self.update_feedback(measured_action)

    @property
    def age(self) -> float:
        return float("inf") if self.latest is None else time.monotonic() - self.latest.timestamp

    @property
    def state(self) -> str:
        return "WAITING" if self.latest is None else ("ACTIVE" if self.age <= COMMAND_TIMEOUT else "HOLD")

    def close(self):
        self.socket.close()


class ActionSender:
    """Send IK goals/arm hold requests and poll the measured robot pose."""

    def __init__(self, host: str, port: int):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.setblocking(False)
        self.destination = (host, port)
        self.sequence = 0
        self.boot_time = None
        self.held_action = HOME_ACTION.copy()
        self.last_feedback = 0.
        self.last_query = 0.

    def poll_feedback(self) -> bool:
        """Return True only on first connection or a different simulator boot."""
        now = time.monotonic()
        if now - self.last_query >= 1. / FEEDBACK_HZ:
            self.socket.sendto(QUERY, self.destination)
            self.last_query = now
        restarted = False
        while True:
            try:
                payload, peer = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            if peer != self.destination or len(payload) != STATUS.size:
                continue
            magic, boot, *held = STATUS.unpack(payload)
            if magic != b"F16S" or not np.isfinite(boot):
                continue
            try:
                held = validate_action(held)
            except ValueError:
                continue
            restarted |= self.boot_time != boot
            self.boot_time, self.held_action, self.last_feedback = boot, held, now
        return restarted

    @property
    def measured_state(self):
        # Keep held_action as a compatibility alias for existing diagnostics.
        return self.held_action

    @property
    def ready(self) -> bool:
        return self.boot_time is not None and time.monotonic() - self.last_feedback <= COMMAND_TIMEOUT

    def send(self, action, *, hold_arms=False):
        self.sequence = (self.sequence + 1) & 0xffffffff
        self.socket.sendto(pack_action(action, self.sequence, hold_arms=hold_arms), self.destination)

    def close(self):
        self.socket.close()
