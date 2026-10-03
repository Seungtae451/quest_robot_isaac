"""Small compatibility boundary for the installed editable TeleVuer package.

Only the Quest process imports this module. Isaac/Kit and Vuer must never be
initialized together: that combination previously prevented WebSocket startup.
The upstream package is not modified. We retain its coordinate conversion,
HTTPS certificate discovery, Vuer event handlers, and image implementation.

Upstream motion_data_ready is a latched 'ever received' flag, not liveness.
Shared monotonic event timestamps below provide real freshness even when the
controllers are stationary. Parent/child reads are protected as one snapshot.
"""
import inspect
import multiprocessing as mp
import os
from pathlib import Path
import socket
import time

import numpy as np
from televuer import TeleVuerWrapper
from televuer.tv_wrapper import (T_ROBOT_OPENXR, T_OPENXR_ROBOT,
    transform_IPunitree_Brobot_world_arm_to_head_then_waist)
from televuer.televuer import TeleVuer
from vuer.schemas import MotionControllers, ImageBackground

from config import teleop_config as cfg
from teleop.xr_pose import valid_pose
from teleop.recording_buttons import RecordingButtons


def certificate_paths():
    """Mirror upstream discovery and fail before allocating shared memory."""
    env_cert, env_key = os.getenv("XR_TELEOP_CERT"), os.getenv("XR_TELEOP_KEY")
    if bool(env_cert) != bool(env_key):
        raise RuntimeError("Set both XR_TELEOP_CERT and XR_TELEOP_KEY, or unset both")
    if env_cert and env_key:
        pair = Path(env_cert).expanduser(), Path(env_key).expanduser()
    else:
        directory = Path.home() / ".config/xr_teleoperate"
        pair = directory / "cert.pem", directory / "key.pem"
        if not all(p.is_file() for p in pair):
            directory = Path(inspect.getfile(TeleVuer)).resolve().parents[2]
            pair = directory / "cert.pem", directory / "key.pem"
    if not all(p.is_file() for p in pair):
        raise FileNotFoundError(f"SSL certificate/key missing: {pair}. See README_TELEOP.md")
    return tuple(str(p) for p in pair)


def detect_host_ip():
    # A UDP connect performs route selection without sending a datagram.
    # --host-ip remains authoritative on machines with multiple LAN adapters.
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("192.0.2.1", 9))
            return probe.getsockname()[0]
    except OSError:
        return socket.gethostbyname(socket.gethostname())


def trigger_encoding(requested="auto"):
    """Identify the implementation once; never infer encoding per sample."""
    if requested != "auto":
        return requested
    source = inspect.getsource(TeleVuerWrapper.get_tele_data)
    if "10.0 - self.tvuer.left_ctrl_triggerValue * 10" in source:
        return "legacy-inverted-10"
    if "left_ctrl_triggerValue=self.tvuer.left_ctrl_triggerValue," in source:
        return "standard"
    raise RuntimeError("Unknown TeleVuer trigger encoding; use --trigger-encoding explicitly")


class _ControllerRateSession:
    """Set the supported MotionControllers fps prop in inherited XR scenes.

    Delegate all other session operations, preserving upstream image handling.
    This applies equally to immersive, ego, and the read-only measurement mode.
    """
    def __init__(self, session):
        self.session = session

    def __getattr__(self, name):
        return getattr(self.session, name)

    def upsert(self, element, *args, **kwargs):
        for item in element if isinstance(element, (list, tuple)) else (element,):
            if isinstance(item, MotionControllers):
                item.fps = cfg.QUEST_INPUT_HZ
            elif isinstance(item, ImageBackground):
                item.distanceToCamera = cfg.QUEST_SCREEN_DISTANCE
                item.height = cfg.QUEST_SCREEN_HEIGHT
        return self.session.upsert(element, *args, **kwargs)


class FreshTeleVuer(TeleVuer):
    def __init__(self, **kwargs):
        # These must exist BEFORE upstream forks its WebSocket child process.
        self.snapshot_lock = mp.RLock()
        self.controller_time = mp.Value("d", 0.)
        self.head_time = mp.Value("d", 0.)
        self.controller_serial = mp.Value("Q", 0)
        self.controller_valid = mp.Value("b", False)
        self.controller_arrivals = mp.Array("d", 1024, lock=False)
        self.head_arrivals = mp.Array("d", 1024, lock=False)
        self.head_serial = mp.Value("Q", 0)
        self.invalid_controller_events = mp.Value("Q", 0)
        self.recording_buttons = RecordingButtons()
        super().__init__(**kwargs)

    async def on_cam_move(self, event, session, fps=60):
        arrived = time.monotonic()
        try:
            pose = np.asarray(event.value["camera"]["matrix"]).reshape(4, 4, order="F")
            if not valid_pose(pose):
                return
        except (KeyError, TypeError, ValueError):
            return
        with self.snapshot_lock:
            await super().on_cam_move(event, session, fps)
            self.head_time.value = time.monotonic()
            self.head_arrivals[self.head_serial.value % 1024] = arrived
            self.head_serial.value += 1

    async def on_controller_move(self, event, session, fps=60):
        arrived = time.monotonic()
        valid = True
        try:
            for side in ("left", "right"):
                pose = np.asarray(event.value[side]).reshape(4, 4, order="F")
                state = event.value[f"{side}State"]
                valid &= valid_pose(pose) and isinstance(state, dict)
                valid &= state.get("connected", True) is not False
                valid &= state.get("tracked", True) is not False
                for key in ("triggerValue", "squeezeValue"):
                    valid &= np.isfinite(float(state[key]))
                thumbstick = np.asarray(state.get("thumbstickValue", [0., 0.]))
                valid &= thumbstick.shape == (2,) and np.isfinite(thumbstick).all()
        except (KeyError, TypeError, ValueError):
            valid = False
        with self.snapshot_lock:
            self.controller_valid.value = valid
            value = event.value if isinstance(event.value, dict) else {}
            press_poses = None
            if valid and valid_pose(self.head_pose):
                head = T_ROBOT_OPENXR @ self.head_pose @ T_OPENXR_ROBOT
                press_poses = [transform_IPunitree_Brobot_world_arm_to_head_then_waist(
                    T_ROBOT_OPENXR @ np.asarray(value[side]).reshape(4,4,order="F") @ T_OPENXR_ROBOT,
                    head, cfg.ARM_REFERENCE_MODE) for side in ("left", "right")]
            self.recording_buttons.update(value.get("leftState", {}), value.get("rightState", {}),
                                          arrived, valid=valid, poses=press_poses)
            if valid:
                await super().on_controller_move(event, session, fps)
                self.controller_time.value = time.monotonic()
                self.controller_arrivals[self.controller_serial.value % 1024] = arrived
                self.controller_serial.value += 1
            else:
                self.invalid_controller_events.value += 1

    async def _until_disconnect(self, stream, session):
        # Vuer 0.0.60 removes ws before the image coroutine's next upsert.
        # That normal disconnect raises an assertion in the installed API.
        # Swallow ONLY that known condition, preserving all genuine failures.
        try:
            await stream(_ControllerRateSession(session))
        except AssertionError:
            if session.CURRENT_WS_ID in self.vuer.ws:
                raise

    async def main_image_monocular_zmq(self, session):
        await self._until_disconnect(super().main_image_monocular_zmq, session)

    async def main_image_monocular_zmq_ego(self, session):
        await self._until_disconnect(super().main_image_monocular_zmq_ego, session)

    async def main_pass_through(self, session):
        await self._until_disconnect(super().main_pass_through, session)


class QuestInterface(TeleVuerWrapper):
    """Use upstream get_tele_data/render/close with a timestamped backend."""
    def __init__(self, display_mode="immersive"):
        cert, key = certificate_paths()
        required = {"arm_reference_mode", "display_mode", "zmq", "binocular", "img_shape"}
        if not required.issubset(inspect.signature(TeleVuerWrapper).parameters):
            raise RuntimeError("Installed TeleVuer signature changed; inspect this adapter")
        # These are the four fields set by the inspected wrapper constructor.
        # Construct its backend subclass directly rather than patching globals
        # or touching the editable installation outside this repository.
        self.use_hand_tracking = False
        self.return_hand_rot_data = False
        self.arm_reference_mode = cfg.ARM_REFERENCE_MODE
        self._snapshot_key = None
        self._snapshot_data = None
        self.last_snapshot_controller_time = 0.
        self.tvuer = FreshTeleVuer(
            use_hand_tracking=False, binocular=False,
            img_shape=(cfg.QUEST_VIEW_HEIGHT, cfg.QUEST_VIEW_WIDTH),
            display_fps=cfg.CAMERA_FPS, display_mode=display_mode,
            zmq=display_mode != "pass-through", webrtc=False,
            cert_file=cert, key_file=key,
        )

    def snapshot(self):
        with self.tvuer.snapshot_lock:
            key = (self.tvuer.controller_serial.value, self.tvuer.head_serial.value)
            if key != self._snapshot_key:
                self._snapshot_data = self.get_tele_data()
                self._snapshot_key = key
            data = self._snapshot_data
            self.last_snapshot_controller_time = self.tvuer.controller_time.value
            now = time.monotonic()
            fresh = (self.tvuer.process.is_alive() and data.motion_data_ready
                     and self.tvuer.controller_valid.value
                     and now - self.tvuer.controller_time.value <= cfg.TRACKING_TIMEOUT
                     and now - self.tvuer.head_time.value <= cfg.TRACKING_TIMEOUT
                     and valid_pose(data.left_wrist_pose) and valid_pose(data.right_wrist_pose))
            return data, bool(fresh), self.tvuer.controller_serial.value

    def print_url(self, host_ip):
        # Vuer 0.0.60 serves its client and WebSocket on the same '/' route.
        port = self.tvuer.vuer.port
        print(f"Quest URL: https://{host_ip}:{port}/?ws=wss://{host_ip}:{port}&grid=False")
        print(f"Hosted client alternative: https://vuer.ai?ws=wss://{host_ip}:{port}&grid=False")
        print(f"Wrapper reference={self.arm_reference_mode}; +X forward, +Y left, +Z up")

    def input_timing(self, window=5.):
        from teleop.input_timing import event_timing
        with self.tvuer.snapshot_lock:
            now = time.monotonic()
            controller_times = self.tvuer.controller_arrivals[:]
            head_times = self.tvuer.head_arrivals[:]
            counters = {"controller_events": self.tvuer.controller_serial.value,
                        "head_events": self.tvuer.head_serial.value,
                        "invalid_controller_events": self.tvuer.invalid_controller_events.value}
        return {"controller": event_timing(controller_times, now, window),
                "head": event_timing(head_times, now, window), **counters}

    def recording_button_events(self, with_poses=False):
        with self.tvuer.snapshot_lock:
            return self.tvuer.recording_buttons.drain(time.monotonic(), with_poses=with_poses)
