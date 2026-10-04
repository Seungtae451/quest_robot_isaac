"""Closed-loop F14 pick/place evaluation in the unchanged recording environment."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import signal
import time
from zoneinfo import ZoneInfo

import numpy as np

from config import teleop_config as cfg, tabletop_config as tabletop, recording_config
from robot.f14_config import HOME_ACTION, USD_ARM_SIGNS, ARM_JOINT_NAMES
from robot.joint_motion import JointMotionLimiter
from simulation.episode_reset import EpisodeReset
from simulation.inference_task import PlacementSuccess, EvaluationResults, bounded_action

ROOT=Path(__file__).resolve().parents[1]


def run(app, args, stop_requested):
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObject, RigidObjectCfg
    from isaaclab.sensors import ContactSensor, ContactSensorCfg
    from simulation.f14_scene import create_scene, resolve_joint_ids, initialize_home, apply_action, semantic_state
    from simulation.cameras import create_cameras, raw_observation, display_rgb_copies
    from simulation.tabletop import sample_cube_poses, table_surface_height
    from simulation.openpi_client import connect_policy

    if tabletop.CUBE_COUNT!=1:
        raise ValueError('This benchmark requires CUBE_COUNT=1, matching the training task.')
    sim=sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1/cfg.PHYSICS_HZ,device=args.device))
    sim.set_camera_view(eye=[1.8,1.4,1.3],target=[.3,0.,.5])
    client=None
    results=None
    active=None
    cameras={}
    failure_keys=None
    inference_worker=None
    terminal_status='interrupted'
    try:
        # Connect before loading the scene; no Quest sender/receiver or recorder is started.
        metadata={}
        if not args.smoke_test:
            client=connect_policy(args.host,args.port,args.inference_timeout)
            metadata=client.get_server_metadata()
            names=ARM_JOINT_NAMES+['left_gripper.open_ratio','right_gripper.open_ratio']
            if metadata.get('format')!='f14_absolute_joint_actions_v1' or metadata.get('action_names')!=names:
                raise ValueError('Server must expose the trained absolute 16D F14 action convention.')
            if metadata['action_horizon']<args.execution_horizon:
                raise ValueError('execution-horizon exceeds the server model action horizon.')
            if metadata['action_fps']!=cfg.CAMERA_FPS:
                raise ValueError('Server training FPS does not match simulation action/camera FPS.')
        robot=create_scene(scene_seed=args.scene_seed,cube_contact_sensors=True)
        cameras=create_cameras(cfg.CAMERA_FPS)
        cubes=RigidObject(RigidObjectCfg(prim_path='/World/Cubes/Cube_0'))
        # The bottom is static: filter its actual CollisionAPI prim, not its Xform container.
        sensor=ContactSensor(ContactSensorCfg(prim_path='/World/Cubes/Cube_0',update_period=0.,
            filter_prim_paths_expr=['/World/CollectionBox/bottom/geometry/mesh']))
        sim.reset()
        for camera in cameras.values():
            camera.update(0.,force_recompute=True)
        arm_ids,finger_ids=resolve_joint_ids(robot)
        initialize_home(robot,arm_ids,finger_ids)
        home=HOME_ACTION.copy()
        home[14:]=1.  # Same physical OPEN value as the recorded episodes.
        limits=np.sort(robot.data.joint_pos_limits[0,arm_ids].cpu().numpy()*USD_ARM_SIGNS[:,None],axis=1)
        dt=sim.get_physics_dt()
        measured=semantic_state(robot,arm_ids,finger_ids)[0].cpu().numpy()
        motion=JointMotionLimiter(measured[:14],cfg.ARM_MAX_VELOCITY,cfg.ARM_MAX_ACCELERATION,cfg.ARM_MAX_TRACKING_ERROR)
        output=args.output or ROOT/'outputs/openpi_closed_loop'/datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S')
        config={'requested_trials':args.trials,'trial_timeout_sim_seconds':args.trial_timeout,
            'scene_seed':args.scene_seed,'execution_horizon':args.execution_horizon,
            'action_fps':cfg.CAMERA_FPS,'physics_hz':cfg.PHYSICS_HZ,'task':args.task,'server':metadata,
            'success':'Full cube footprint inside box and actual cube/bottom contact force >= threshold.',
            'success_hold_seconds':args.success_hold,'contact_force_threshold_n':args.contact_force_threshold,
            'physics_paused_during_inference':True,'arm_max_velocity':cfg.ARM_MAX_VELOCITY,
            'arm_max_acceleration':cfg.ARM_MAX_ACCELERATION,
            'table_center':tabletop.TABLE_CENTER,'table_size':tabletop.TABLE_SIZE,
            'box_center_xy':tabletop.BOX_CENTER_XY,'cube_size':tabletop.CUBE_SIZE}
        results=EvaluationResults(output,config)
        if args.smoke_test:
            contact_smoke_test(sim,robot,cubes,sensor,arm_ids,finger_ids,home,args)
            (output/'contact_validation.json').write_text(json.dumps({'passed':True,
                'cases':['airborne not success','table contact not success','box bottom contact succeeds']},indent=2)+'\n')
            terminal_status='contact_smoke_passed'
            return
        from concurrent.futures import ThreadPoolExecutor
        from simulation.terminal_failure import TerminalFailureKey, wait_prediction
        failure_keys=TerminalFailureKey()
        inference_worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='f14-inference')
        print('Evaluation controls: f = fail current trial (no Enter); Ctrl+C = stop.',flush=True)
        reset=EpisodeReset()
        goal=home.copy()
        chunk=None
        chunk_index=0
        elapsed=frame_accumulator=0.
        spawn_pose=None
        last_log=time.monotonic()

        def finish_trial(outcome):
            nonlocal active,chunk,reset,goal
            trial_time=elapsed-active['start_sim_time']
            write_trial(results,active,outcome,trial_time,position,quaternion,bottom_force)
            print(f'Trial {len(results.trials)}/{args.trials}: {outcome}, {trial_time:.2f}s',flush=True)
            active=None
            chunk=None
            reset=EpisodeReset()
            goal,_=reset.goal(measured,0.)

        while app.is_running() and not stop_requested():
            if failure_keys.poll(active is not None):
                finish_trial('manual_failure')
            if reset is not None:
                goal,enabled=reset.goal(measured,dt)
                if reset.error:
                    raise RuntimeError(reset.error)
            else:
                enabled=True
            action=goal.copy()
            action[:14]=motion.step(goal[:14],measured[:14],dt,enabled)
            apply_action(robot,action,arm_ids,finger_ids)
            robot.write_data_to_sim()
            sim.step(render=False)
            robot.update(dt)
            cubes.update(dt)
            sensor.update(dt)
            measured=semantic_state(robot,arm_ids,finger_ids)[0].cpu().numpy()
            elapsed+=dt
            position=cubes.data.root_pos_w[0].cpu().numpy()
            quaternion=cubes.data.root_quat_w[0].cpu().numpy()
            force_matrix=sensor.data.force_matrix_w
            if force_matrix is None or force_matrix.shape[-2:]!=(1,3):
                raise RuntimeError('Cube/bottom filtered contact sensor is unavailable.')
            bottom_force=force_matrix.reshape(-1,3)[0].cpu().numpy()
            if active is not None:
                trial_time=elapsed-active['start_sim_time']
                outcome=None
                if success.update(position,quaternion,bottom_force,dt):
                    outcome='success'
                elif position[2]<table_surface_height()-tabletop.CUBE_SIZE-.02:
                    outcome='cube_fell'
                elif trial_time+1e-9>=args.trial_timeout:
                    outcome='timeout'
                if outcome:
                    finish_trial(outcome)
            if reset is not None:
                reset.update(measured,motion.velocity,dt)
                if reset.phase=='RESPAWN':
                    trial_index=len(results.trials)
                    seed=args.scene_seed+trial_index
                    spawn_pose=sample_cube_poses(seed)[0]
                    respawn_cube(cubes,spawn_pose,sim.device)
                    sensor.reset()
                    reset.cubes_respawned()
                elif reset.phase=='COMPLETE':
                    if np.max(np.abs(measured[:14]-home[:14]))>=.015 or np.min(measured[14:])<.97:
                        raise RuntimeError('Robot did not reach the common HOME/open-gripper start state.')
                    reset=None
                    if len(results.trials)>=args.trials:
                        terminal_status='complete'
                        break
                    active={'trial_index':len(results.trials),'scene_seed':args.scene_seed+len(results.trials),
                        'spawn_position':list(spawn_pose[0]),'spawn_quaternion':list(spawn_pose[1]),
                        'initial_state':measured.tolist(),'start_sim_time':elapsed,
                        'inference_seconds':[],'clipped_action_frames':0}
                    success=PlacementSuccess(args.success_hold,args.contact_force_threshold)
                    goal=home.copy()
                    chunk=None
                    chunk_index=0
                    client.reset()
                    print(f'Starting trial {len(results.trials)+1}/{args.trials}; cube={spawn_pose[0]}',flush=True)
            frame_accumulator+=dt
            for camera in cameras.values():
                camera.update(dt)
            if frame_accumulator+1e-9>=1/cfg.CAMERA_FPS:
                frame_accumulator%=1/cfg.CAMERA_FPS
                sim.render()
                if active is not None:
                    if chunk is None or chunk_index>=args.execution_horizon:
                        observation=raw_observation(cameras,semantic_state(robot,arm_ids,finger_ids))
                        images=display_rgb_copies(observation)
                        inputs={'images':{key:np.transpose(image,(2,0,1)) for key,image in images.items()},
                                'state':measured.astype(np.float32),'prompt':args.task}
                        # Physics stays paused while waiting, but f can abort the trial immediately.
                        started=time.monotonic()
                        future=inference_worker.submit(client.infer,inputs)
                        prediction=wait_prediction(future,failure_keys,
                            lambda:stop_requested() or not app.is_running(),
                            lambda:finish_trial('manual_failure'))
                        if prediction is None:
                            continue
                        active['inference_seconds'].append(time.monotonic()-started)
                        chunk=np.asarray(prediction['actions'])
                        if chunk.shape!=(metadata['action_horizon'],16) or not np.isfinite(chunk).all():
                            raise ValueError(f'Invalid model action chunk: {chunk.shape}')
                        chunk_index=0
                    goal,clipped=bounded_action(chunk[chunk_index],limits)
                    active['clipped_action_frames']+=int(clipped)
                    chunk_index+=1
            if time.monotonic()-last_log>=5.:
                report=results.summary('running')
                print(f'Completed={report["completed_trials"]}/{args.trials} success={report["successes"]} '
                      f'phase={"POLICY" if active else reset.phase if reset else "HOLD"}',flush=True)
                last_log=time.monotonic()
    except Exception as exc:
        terminal_status=f'error: {type(exc).__name__}: {exc}'
        raise
    finally:
        if failure_keys:
            failure_keys.close()
        if inference_worker:
            inference_worker.shutdown(wait=False,cancel_futures=True)
        if results:
            if active is not None:
                # Interrupted/infrastructure-error trials are recorded but not counted as task failures.
                write_trial(results,active,'interrupted' if terminal_status=='interrupted' else 'error',
                            elapsed-active['start_sim_time'],position,quaternion,bottom_force)
            summary=results.summary(terminal_status)
            print(json.dumps(summary,indent=2),flush=True)
            print(f'Results: {results.directory}',flush=True)
        if client:
            client.close()
        cameras.clear()
        camera=None
        import gc
        gc.collect()
        sim.clear_all_callbacks()
        # Unsubscribe Lab's headless STOP renderer before stopping the timeline.
        sim.clear_instance()
        sim.stop()


def respawn_cube(cubes,pose,device):
    import torch
    state=cubes.data.root_state_w.clone()
    state[0,:3]=torch.as_tensor(pose[0],device=device)
    state[0,3:7]=torch.as_tensor(pose[1],device=device)
    state[:,7:]=0.
    cubes.write_root_state_to_sim(state)
    cubes.reset()


def write_trial(results,active,outcome,seconds,position,quaternion,force):
    latencies=active['inference_seconds']
    values={key:value for key,value in active.items() if key!='start_sim_time'}
    values.update(outcome=outcome,elapsed_sim_seconds=seconds,final_cube_position=position.tolist(),
                  final_cube_quaternion=quaternion.tolist(),bottom_contact_force_n=force.tolist(),
                  inference_p95_seconds=float(np.percentile(latencies,95)) if latencies else None)
    results.record(**values)


def contact_smoke_test(sim,robot,cubes,sensor,arm_ids,finger_ids,home,args):
    """Exercise the real bottom contact filter; no learned policy or task trials."""
    from simulation.f14_scene import apply_action
    from simulation.tabletop import table_surface_height
    dt=sim.get_physics_dt()
    floor=table_surface_height()+tabletop.BOX_BOTTOM_THICKNESS
    bx,by=tabletop.BOX_CENTER_XY
    cases=[('airborne',(bx,by,floor+.3),2,False),
           ('table',(tabletop.TABLE_CENTER[0]-.12,by+.22,table_surface_height()+tabletop.CUBE_SIZE/2+.004),60,False),
           ('bottom',(bx,by,floor+tabletop.CUBE_SIZE/2+.004),60,True)]
    for name,position,steps,expected in cases:
        respawn_cube(cubes,(position,(1.,0.,0.,0.)),sim.device)
        sensor.reset()
        success=PlacementSuccess(0.,args.contact_force_threshold)
        observed=False
        for _ in range(steps):
            apply_action(robot,home,arm_ids,finger_ids)
            robot.write_data_to_sim()
            sim.step(render=False)
            robot.update(dt)
            cubes.update(dt)
            sensor.update(dt)
            force=sensor.data.force_matrix_w.reshape(-1,3)[0].cpu().numpy()
            observed|=success.update(cubes.data.root_pos_w[0].cpu().numpy(),
                                    cubes.data.root_quat_w[0].cpu().numpy(),force,dt)
        if observed!=expected:
            raise AssertionError(f'Contact smoke test {name}: success={observed}, expected={expected}')
        print(f'CONTACT CHECK PASSED: {name}',flush=True)


def main(argv=None):
    from isaaclab.app import AppLauncher
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8000)
    parser.add_argument('--openpi-root',type=Path,default=ROOT.parent/'openpi')
    parser.add_argument('--trials',type=int,default=100)
    parser.add_argument('--trial-timeout',type=float,default=60.)
    parser.add_argument('--execution-horizon',type=int,default=16)
    parser.add_argument('--scene-seed',type=int,default=0)
    parser.add_argument('--task',default=recording_config.TASK)
    parser.add_argument('--inference-timeout',type=float,default=120.)
    parser.add_argument('--success-hold',type=float,default=0.,help='0 counts first actual bottom contact')
    parser.add_argument('--contact-force-threshold',type=float,default=.001,help='N; cube weight is ~0.196N')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--smoke-test',action='store_true',help='Check the actual contact sensor without policy inference')
    AppLauncher.add_app_launcher_args(parser)
    args=parser.parse_args(argv)
    if (args.trials<1 or not 1<=args.execution_horizon<=50 or args.trial_timeout<=0
            or args.success_hold<0 or args.contact_force_threshold<=0 or args.inference_timeout<=0):
        parser.error('Invalid trial count, horizon, timeout or success threshold.')
    sys_path=args.openpi_root/'packages/openpi-client/src'
    import sys
    sys.path.insert(0,str(sys_path))
    if tabletop.CUBE_SPAWN_REQUIRE_IK:
        from robot.spawn_workspace import ensure_spawn_pool
        ensure_spawn_pool()
    args.enable_cameras=True
    launcher=AppLauncher(args)
    stopped=False
    def stop(signum,frame):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop)
    signal.signal(signal.SIGTERM,stop)
    try:
        run(launcher.app,args,lambda:stopped)
    finally:
        launcher.app.close()
