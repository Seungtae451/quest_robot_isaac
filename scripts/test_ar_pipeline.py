"""Bounded actual-GPU AR/recording check, with synthetic WebXR grip poses.

No headset or user dataset is touched. Uses isolated ports and a fresh local
dataset under outputs/quest_ar_validation; physical passthrough remains a
separate Quest 3 acceptance check. Run with env_isaaclab Python.
"""
import asyncio
from contextlib import closing
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import aiohttp
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from teleop.action_protocol import ActionSender
from teleop.episode_protocol import EpisodeClient
from teleop.ar_world import world_matrix
from robot.f14_config import HOME_Q, F14_URDF_PATH, GRIPPER_CONTROL_OFFSET
from robot.f14_ik import F14IK
from robot.tool_frame import controller_ee_basis, control_position, rotation_wxyz

PORT = 18013
ACTION_PORT, EPISODE_PORT = 16045, 16046
SCENE_ENDPOINT = 'tcp://127.0.0.1:16557'


class Driver:
    def __init__(self, ws, monitor, episodes, manifest):
        self.ws, self.monitor, self.episodes = ws, monitor, episodes
        self.manifest, self.scene, self.ui = manifest, None, {}
        self.sequence = 0
        self.placed = False
        self.space = 1
        self.world = world_matrix([1., 0., -.5], .7)
        self.neutral = None
        self.rotations = None
        self.reader = asyncio.create_task(self.read())

    async def read(self):
        async for message in self.ws:
            if message.type == aiohttp.WSMsgType.TEXT:
                value = json.loads(message.data)
                if value.get('kind') == 'state':
                    self.scene, self.ui = value['scene'], value['ui']
                    if self.scene and self.neutral is None:
                        self.rotations = [rotation_wxyz(self.scene['links'][f'{side}_dof7_link'][3:]) @ controller_ee_basis(side).T for side in ('right','left')]
                        self.neutral = np.array([control_position(self.scene['links'][f'{side}_dof7_link'])
                                                 for side in ('right', 'left')])

    async def feed(self, seconds, *, positions=None, buttons=(), head=True, visible=True, rotations=None):
        until = time.monotonic()+seconds
        while time.monotonic()<until:
            message = {'kind':'input', 'sequence':self.sequence, 'space':self.space,
                       'world':self.world.flatten(order='F').tolist(), 'placed':self.placed,
                       'model_id':self.manifest['model_id'], 'visible':visible, 'head_tracked':head}
            self.sequence += 1
            for i,side in enumerate(('left','right')):
                pose = np.eye(4)
                if self.rotations is not None: pose[:3,:3] = (rotations if rotations is not None else self.rotations)[i]
                pose[:3,3] = (positions if positions is not None else self.neutral)[i] if self.neutral is not None else [0,0,0]
                pose = self.world @ pose
                message[side] = {'matrix':pose.flatten(order='F').tolist(),'tracked':True,'emulated':False,
                    'state':{'triggerValue':0.,'squeezeValue':0.,
                             'aButton':('a' if side=='right' else 'x') in buttons,
                             'bButton':side=='right' and 'b' in buttons}}
            await self.ws.send_json(message)
            self.monitor.poll_feedback(); self.episodes.poll()
            if self.episodes.available and self.episodes.status['state']=='ERROR':
                raise RuntimeError(self.episodes.status)
            await asyncio.sleep(1/60)

    async def click(self, button, **kwargs):
        await self.feed(.12, **kwargs)
        await self.feed(.12, buttons=(button,), **kwargs)
        await self.feed(.12, **kwargs)

    async def wait(self, state, timeout=45):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            await self.feed(.1)
            if self.episodes.available and self.episodes.status['state']==state:
                return
        raise RuntimeError(f'Waiting for {state}: {self.episodes.status}, ui={self.ui}')

    async def wait_start_ready(self, timeout=10):
        # READY can precede the first fresh HOME heartbeat after reset.
        # Follow the same server-ready cue presented to the headset user.
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            await self.feed(.1)
            if self.episodes.available and self.episodes.status['state']=='READY' and self.ui.get('start_gate',{}).get('allowed'):
                return
        raise RuntimeError(f'Waiting for start gates: {self.ui}')


def stop(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=25)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=10)


async def run():
    output = ROOT/'outputs/quest_ar_validation'/time.strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True)
    env = dict(os.environ, PYTHONUNBUFFERED='1')
    sim = quest = None
    result = {'physical_headset':False, 'dataset_root':str(output/'dataset')}
    with (output/'isaac.log').open('w') as sim_log, (output/'quest.log').open('w') as quest_log:
        try:
            sim = subprocess.Popen([sys.executable,'-u','scripts/run_isaac_teleop.py','--headless','--device','cuda:0',
                '--record','--no-quest-video','--camera-fps','20','--scene-seed','12','--action-port',str(ACTION_PORT),
                '--session-port',str(EPISODE_PORT),'--ar-scene-endpoint',SCENE_ENDPOINT,
                '--video-endpoint','tcp://127.0.0.1:16556','--dataset-root',str(output/'dataset'),
                '--repo-id','local/f14_quest_ar_validation'],cwd=ROOT,env=env,stdout=sim_log,stderr=subprocess.STDOUT)
            with closing(ActionSender('127.0.0.1',ACTION_PORT)) as monitor, closing(EpisodeClient('127.0.0.1',EPISODE_PORT)) as episodes:
                deadline = time.monotonic()+90
                while time.monotonic()<deadline:
                    monitor.poll_feedback(); episodes.poll()
                    if monitor.ready and episodes.available and episodes.status['state']=='READY': break
                    if sim.poll() is not None: raise RuntimeError(f'Isaac exited: {output}/isaac.log')
                    await asyncio.sleep(.05)
                assert monitor.ready and episodes.available, 'Isaac startup timeout'
                quest = subprocess.Popen([sys.executable,'-u','scripts/run_quest_teleop.py','--xr-mode','ar',
                    '--host-ip','127.0.0.1','--ar-port',str(PORT),'--action-port',str(ACTION_PORT),
                    '--session-port',str(EPISODE_PORT),'--ar-scene-endpoint',SCENE_ENDPOINT,
                    '--timing-output',str(output/'timing.jsonl')],cwd=ROOT,env=env,stdout=quest_log,stderr=subprocess.STDOUT)
                async with aiohttp.ClientSession() as session:
                    ws = None
                    for _ in range(100):
                        try:
                            ws = await session.ws_connect(f'https://127.0.0.1:{PORT}/ws',ssl=False)
                            break
                        except aiohttp.ClientError:
                            if quest.poll() is not None: raise RuntimeError(f'Quest exited: {output}/quest.log')
                            await asyncio.sleep(.1)
                    assert ws is not None
                    hello = await ws.receive_json(timeout=5)
                    driver = Driver(ws,monitor,episodes,hello['manifest'])
                    try:
                        async with session.get(f'https://127.0.0.1:{PORT}/',ssl=False) as response:
                            assert response.status==200 and 'F14 Quest AR' in await response.text()
                        for path in ('/assets/robot.glb','/assets/manifest.json','/web/client.js','/web/core.mjs',
                                     '/assets/vendor/three/build/three.module.js'):
                            async with session.get(f'https://127.0.0.1:{PORT}{path}',ssl=False) as response:
                                assert response.status==200,path
                                await response.read()
                        with np.testing.assert_raises(aiohttp.WSServerHandshakeError):
                            await session.ws_connect(f'https://127.0.0.1:{PORT}/ws',ssl=False)
                        await driver.feed(.5)
                        assert driver.scene and driver.neutral is not None
                        assert len(driver.scene['links'])==19 and len(driver.scene['cubes'])==1
                        (output/'scene.json').write_text(json.dumps(driver.scene,indent=2)+'\n')
                        initial_cube = driver.scene['cubes'][0][:3]
                        driver.placed=True
                        await driver.feed(.12,buttons=('a',))  # placement A must be consumed
                        await driver.feed(.3)
                        assert episodes.status['state']=='READY' and episodes.status['frames']==0
                        away=driver.neutral.copy(); away[:,0]+=.08
                        await driver.click('a',positions=away)
                        await driver.feed(.3,positions=away)
                        assert episodes.status['state']=='READY'
                        rejection = driver.ui['last_start']
                        assert not rejection['allowed']
                        assert rejection['reasons'] == ['LEFT_OUTSIDE_4CM','RIGHT_OUTSIDE_4CM'], rejection
                        assert min(rejection['distances_m']) > .05
                        result['start_rejection_reports_press_distance_and_exact_reason'] = rejection
                        np.testing.assert_allclose(monitor.measured_state[:14],HOME_Q,atol=.015)
                        result['placement_a_consumed_and_outside_4cm_rejected']=True
                        import pinocchio as pin
                        await driver.feed(.3)
                        await driver.wait_start_ready()
                        await driver.click('a'); await driver.wait('RECORDING')
                        await driver.feed(.3)
                        np.testing.assert_allclose(monitor.measured_state[:14],HOME_Q,atol=.015)
                        result['recording_a_has_no_start_jump']=True
                        turned=[r.copy() for r in driver.rotations]
                        turned[0]=pin.exp3(np.array([.10,0.,0.]))@turned[0]
                        await driver.feed(2.5,rotations=turned)
                        measured=F14IK(F14_URDF_PATH,ee_offset=GRIPPER_CONTROL_OFFSET).forward_kinematics(monitor.measured_state[:14])
                        error=float(np.linalg.norm(pin.log3(measured[1].rotation.T@(turned[0]@controller_ee_basis('right')))))
                        assert error<.03,error
                        result['rotation_following_error_rad']=error
                        await driver.feed(2.)
                        asymmetric=driver.neutral.copy()
                        asymmetric[0]+=[.025,0.,.025]  # physical LEFT -> robot RIGHT, +world X/Z
                        await driver.feed(1.5,positions=asymmetric,head=False)
                        tips=F14IK(F14_URDF_PATH,ee_offset=GRIPPER_CONTROL_OFFSET).forward_kinematics(monitor.measured_state[:14])
                        delta=np.array([p.translation-driver.neutral[1-i] for i,p in enumerate(tips)])
                        assert np.linalg.norm(delta[0])<.004,delta
                        assert delta[1,0]>.005 and delta[1,2]>.005,delta
                        result['physical_left_moves_robot_right_without_xyz_mirroring_m']=delta.tolist()
                        moved=driver.neutral.copy(); moved[:,2]+=.025
                        await driver.feed(2.,positions=moved,head=False)
                        assert episodes.status['state']=='RECORDING'
                        fk=F14IK(F14_URDF_PATH,ee_offset=GRIPPER_CONTROL_OFFSET).forward_kinematics(monitor.measured_state[:14])
                        dz=[float(p.translation[2]-driver.neutral[1-i,2]) for i,p in enumerate(fk)]
                        assert min(dz)>.001,dz
                        result['head_independent_control_rise_m']=dz
                        await driver.click('b',positions=moved); await driver.wait('REVIEW')
                        assert episodes.status['frames']>10
                        await driver.click('a'); await driver.wait('READY')
                        assert episodes.status['saved_episodes']==1
                        assert driver.ui['placed'] and driver.scene['cubes'][0][:3]!=initial_cube
                        result['save_home_new_cube_world_anchor_retained']=True
                        await driver.wait_start_ready()
                        await driver.click('a'); await driver.wait('RECORDING')
                        await driver.feed(.8); await driver.click('b'); await driver.wait('REVIEW')
                        await driver.click('x'); await driver.wait('READY')
                        assert episodes.status['saved_episodes']==1
                        result['discard_does_not_append_dataset']=True
                        await driver.wait_start_ready()
                        await driver.click('a'); await driver.wait('RECORDING')
                        await driver.feed(.5)
                        await driver.feed(.2,visible=False)
                        await driver.wait('REVIEW')
                        await driver.feed(.4)
                        assert 'ACTIVE' not in driver.ui['control']
                        result['tracking_loss_stops_episode_no_auto_resume']=True
                        await driver.click('x'); await driver.wait('READY')
                        info=json.loads((output/'dataset/meta/info.json').read_text())
                        assert info['total_episodes']==1
                        image_keys=[k for k in info['features'] if k.startswith('observation.images.')]
                        assert len(image_keys)==3 and info['features']['action']['shape']==[16]
                        result['lerobot_camera_features']=image_keys
                        result['total_episodes']=info['total_episodes']
                        result['total_frames']=info['total_frames']
                    finally:
                        await ws.close(); driver.reader.cancel()
                        await asyncio.gather(driver.reader,return_exceptions=True)
        finally:
            stop(quest); stop(sim)
            (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print('QUEST AR PIPELINE PASSED',json.dumps(result,indent=2),flush=True)
    print('Validation files:',output,flush=True)


if __name__=='__main__':
    asyncio.run(run())
