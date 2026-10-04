"""HTTPS WebXR AR client + raw world-frame input, independent of TeleVuer.

Only this process imports aiohttp. The thread owns its scene SUB and WSS I/O;
the control thread reads atomic snapshots. Neither imports Isaac/Kit.
"""
import asyncio
from collections import deque
import json
import ssl
import threading
import time
from types import SimpleNamespace

import numpy as np

from config import ar_config, teleop_config as cfg
from teleop.ar_scene import SceneSubscriber
from teleop.ar_world import rigid_matrix, controller_in_robot, attachment_distances
from teleop.recording_buttons import RecordingButtons
from teleop.xr_tls import certificate_paths


class ARInputState:
    """All methods are called under the adapter lock; independently testable."""
    def __init__(self, model_id):
        self.model_id = model_id
        self.buttons = RecordingButtons()
        self.arrivals = deque(maxlen=1024)
        self.head_arrivals = deque(maxlen=1024)
        self.serial = 0
        self.packet_sequence = -1
        self.timestamp = 0.
        self.valid = False
        self.placed = False
        self.space = None
        self.minimum_space = 0
        self.epoch = 0
        self.world = None
        self.invalid_events = 0
        self.data = SimpleNamespace(left_wrist_pose=np.eye(4), right_wrist_pose=np.eye(4),
            left_ctrl_triggerValue=0., right_ctrl_triggerValue=0.,
            left_ctrl_squeezeValue=0., right_ctrl_squeezeValue=0.)

    def invalidate(self, *, reset_space=False):
        self.valid = False
        self.buttons.update({}, {}, time.monotonic(), valid=False)
        self.buttons.consumed = self.buttons.serial.value
        if reset_space:
            self.placed, self.world, self.space = False, None, None
            self.epoch += 1

    def disconnect(self):
        self.invalidate(reset_space=True)
        self.packet_sequence = -1
        self.minimum_space = 0

    def require_replacement(self):
        if self.space is not None:
            self.minimum_space = self.space + 1
        self.invalidate(reset_space=True)

    def update(self, packet, now):
        try:
            if self.timestamp and now - self.timestamp > cfg.TRACKING_TIMEOUT:
                self.invalidate()  # Held buttons cannot become edges after a gap.
            sequence = packet['sequence']
            if type(sequence) is not int or sequence <= self.packet_sequence:
                return False
            self.packet_sequence = sequence
            if packet.get('model_id') != self.model_id or not packet.get('visible', False):
                raise ValueError('XR session not visible or visual model not ready')
            space = packet['space']
            if type(space) is not int or space < self.minimum_space:
                raise ValueError('Invalid XR reference-space revision')
            if self.space is not None and space < self.space:
                raise ValueError('Old reference-space revision')
            if self.space != space:
                self.invalidate(reset_space=True)
                self.space = space
            if not packet.get('placed', False):
                self.invalidate(reset_space=True)
                self.space = space
                return False
            world = rigid_matrix(packet['world'])
            if not np.allclose(world[:3, 2], [0, 1, 0], atol=1e-4):
                raise ValueError('Scene must stay upright at unit scale')
            if self.world is not None and (np.linalg.norm(world[:3, 3] - self.world[:3, 3]) > .05
                    or np.linalg.norm(world[:3, :3] - self.world[:3, :3]) > .15):
                self.require_replacement()
                raise ValueError('Anchor jumped; place world again')
            self.world, self.placed = world, True
            poses, states = [], []
            for side in ('left', 'right'):
                hand = packet[side]
                if not hand.get('tracked', False) or hand.get('emulated', True):
                    raise ValueError('Controller gripSpace is not tracked')
                poses.append(controller_in_robot(hand['matrix'], world))
                state = hand['state']
                for key in ('triggerValue', 'squeezeValue'):
                    value = float(state[key])
                    if not np.isfinite(value) or not 0 <= value <= 1:
                        raise ValueError('Invalid analog input')
                states.append(state)
            self.data = SimpleNamespace(left_wrist_pose=poses[0], right_wrist_pose=poses[1],
                left_ctrl_triggerValue=float(states[0]['triggerValue']),
                right_ctrl_triggerValue=float(states[1]['triggerValue']),
                left_ctrl_squeezeValue=float(states[0]['squeezeValue']),
                right_ctrl_squeezeValue=float(states[1]['squeezeValue']))
            self.buttons.update(states[0], states[1], now, valid=True, poses=poses)
            self.valid = True
            self.timestamp = now
            self.serial += 1
            self.arrivals.append(now)
            if packet.get('head_tracked', False):
                self.head_arrivals.append(now)
            return True
        except (ValueError, TypeError, KeyError, AttributeError, np.linalg.LinAlgError):
            self.invalid_events += 1
            self.invalidate()
            return False


class QuestARInterface:
    def __init__(self, scene_endpoint=ar_config.SCENE_ENDPOINT, port=ar_config.HTTPS_PORT):
        manifest_path = ar_config.ASSET_ROOT / 'manifest.json'
        if not manifest_path.is_file() or not (ar_config.ASSET_ROOT / 'vendor/three/build/three.module.js').is_file():
            raise FileNotFoundError('AR assets missing. Run scripts/export_quest_ar_assets.py as described in README_TELEOP.md')
        self.manifest = json.loads(manifest_path.read_text())
        self.port, self.scene_endpoint = port, scene_endpoint
        self.cert, self.key = certificate_paths()
        self.lock = threading.RLock()
        self.input = ARInputState(self.manifest['model_id'])
        self.scene = None
        self.ui = {'control': 'WAITING FOR ISAAC', 'episode': None}
        self.last_snapshot_controller_time = 0.
        self.started = threading.Event()
        self.stopping = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, name='quest-ar-web', daemon=True)
        self.thread.start()
        if not self.started.wait(10.) or self.error:
            self.close()
            raise RuntimeError(f'Quest AR server failed: {self.error}')

    @property
    def is_alive(self):
        return self.thread.is_alive() and self.error is None

    @property
    def placed(self):
        with self.lock:
            return self.input.placed

    @property
    def control_epoch(self):
        with self.lock:
            return self.input.epoch

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            fresh = (self.is_alive and self.input.valid and self.input.placed
                and now - self.input.timestamp <= cfg.TRACKING_TIMEOUT
                and self.scene is not None
                and 0 <= now - self.scene['timestamp'] <= ar_config.SCENE_TIMEOUT)
            self.last_snapshot_controller_time = self.input.timestamp
            return self.input.data, bool(fresh), self.input.serial

    def attachment(self, poses, boot_time):
        detail = self.attachment_status(poses, boot_time)
        return not detail['reasons'], detail['distances_m']

    def attachment_status(self, poses, boot_time):
        with self.lock:
            reasons, distances = [], None
            if not self.placed:
                reasons.append('PLACE_WORLD')
            if poses is None:
                reasons.append('NO_PRESS_POSE')
            if self.scene is None:
                reasons.append('SCENE_MISSING')
            elif self.scene['boot_time'] != boot_time:
                reasons.append('ISAAC_RESTARTED')
            elif not 0 <= time.monotonic() - self.scene['timestamp'] <= ar_config.SCENE_TIMEOUT:
                reasons.append('SCENE_STALE')
            if not reasons:
                wrists = [self.scene['links'][f'{side}_dof7_link'][:3] for side in ('left', 'right')]
                distances = attachment_distances(poses, wrists).tolist()
                for side, distance in zip(('LEFT', 'RIGHT'), distances):
                    if distance > ar_config.ATTACH_RADIUS + 1e-9:
                        reasons.append(side + '_OUTSIDE_5CM')
            return {'reasons': reasons, 'distances_m': distances, 'radius_m': ar_config.ATTACH_RADIUS}

    def recording_button_events(self, with_poses=False):
        with self.lock:
            return self.input.buttons.drain(time.monotonic(), with_poses=with_poses)

    def input_timing(self, window=5.):
        from teleop.input_timing import event_timing
        with self.lock:
            now = time.monotonic()
            return {'controller': event_timing(list(self.input.arrivals), now, window),
                    'head': event_timing(list(self.input.head_arrivals), now, window),
                    'controller_events': self.input.serial,
                    'head_events': len(self.input.head_arrivals),
                    'invalid_controller_events': self.input.invalid_events}

    def update_status(self, control, episode, attachment=None, start_gate=None, last_start=None):
        with self.lock:
            self.ui = {'control': control, 'episode': episode, 'attachment_m': attachment,
                       'placed': self.input.placed, 'epoch': self.input.epoch,
                       'start_gate': start_gate, 'last_start': last_start}

    def print_url(self, host_ip):
        print(f'Quest AR URL: https://{host_ip}:{self.port}/')
        print('Enter AR -> A places world -> both grips within 5 cm of dof7 -> A starts.')
        print('Fixed XR world, 1 metre = 1 metre; head motion never remaps arm input.')

    def _run(self):
        try:
            asyncio.run(self._serve())
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.started.set()

    async def _serve(self):
        from aiohttp import web
        subscriber = SceneSubscriber(self.scene_endpoint)
        app = web.Application(client_max_size=16384)
        owner = None

        async def index(request):
            return web.FileResponse(ar_config.WEB_ROOT / 'index.html', headers={'Cache-Control': 'no-store'})

        async def websocket(request):
            nonlocal owner
            if owner is not None and not owner.closed:
                raise web.HTTPConflict(text='A Quest controller session is already connected.')
            ws = web.WebSocketResponse(max_msg_size=16384, heartbeat=10)
            await ws.prepare(request)
            owner = ws
            with self.lock:
                self.input.disconnect()
            try:
                await ws.send_json({'kind': 'hello', 'manifest': self.manifest,
                                    'attach_radius': ar_config.ATTACH_RADIUS,
                                    'input_hz': cfg.QUEST_INPUT_HZ})
                async for message in ws:
                    if message.type != web.WSMsgType.TEXT:
                        continue
                    try:
                        value = json.loads(message.data)
                        with self.lock:
                            if value.get('kind') == 'input':
                                self.input.update(value, time.monotonic())
                            elif value.get('kind') in ('reset', 'end'):
                                self.input.require_replacement()
                            elif value.get('kind') == 'pause':
                                self.input.invalidate()
                    except (ValueError, TypeError, AttributeError):
                        with self.lock:
                            self.input.invalidate()
            finally:
                with self.lock:
                    self.input.disconnect()
                if owner is ws:
                    owner = None
            return ws

        app.router.add_get('/', index)
        app.router.add_get('/ws', websocket)
        app.router.add_static('/web/', ar_config.WEB_ROOT)
        app.router.add_static('/assets/', ar_config.ASSET_ROOT)
        runner = web.AppRunner(app, access_log=None)
        try:
            await runner.setup()
            tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            tls.load_cert_chain(self.cert, self.key)
            await web.TCPSite(runner, '0.0.0.0', self.port, ssl_context=tls).start()
            self.started.set()
            while not self.stopping.is_set():
                value = subscriber.receive()
                with self.lock:
                    if value is not None:
                        self.scene = value
                    scene, ui = self.scene, self.ui.copy()
                    ui['placed'] = self.input.placed
                    ui['space'] = self.input.space
                    ui['input_sequence'] = self.input.packet_sequence
                    ui['minimum_space'] = self.input.minimum_space
                if owner is not None and not owner.closed:
                    target = owner
                    age = time.monotonic() - scene['timestamp'] if scene else None
                    try:
                        # Slow/disconnected clients cannot create an unbounded
                        # queue or block physics/control, which run elsewhere.
                        await asyncio.wait_for(target.send_json({'kind': 'state', 'scene': scene,
                            'scene_age': age, 'ui': ui}), timeout=.25)
                    except (TimeoutError, ConnectionError, RuntimeError):
                        await target.close()
                await asyncio.sleep(1. / ar_config.SCENE_HZ)
        finally:
            if owner is not None:
                await owner.close()
            await runner.cleanup()
            subscriber.close()

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=3.)
