"""Real Isaac descent and wall-crossing test for collection-only XYZ limits.

Uses no Quest, dataset recorder or learned policy. Writes a separate validation
report. Run with env_isaaclab Python, --headless --device cuda:0.
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    from isaaclab.app import AppLauncher
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps',type=int,default=720)
    parser.add_argument('--differential-ik',action='store_true',help='Validate live Pinocchio + ProxQP references')
    AppLauncher.add_app_launcher_args(parser)
    args=parser.parse_args()
    from robot.spawn_workspace import ensure_spawn_pool
    ensure_spawn_pool()
    launcher=AppLauncher(args)
    sim=None
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.sim.schemas import activate_contact_sensors
        from isaaclab.sensors import ContactSensor,ContactSensorCfg
        import pinocchio as pin
        from config import teleop_config as cfg
        from robot.f14_config import F14_URDF_PATH,HOME_ACTION,USD_ARM_SIGNS,LEFT_EE_DOWN_ROT,RIGHT_EE_DOWN_ROT
        from robot.f14_ik import F14IK
        from robot.joint_motion import JointMotionLimiter
        from robot.reachable_ik import ReachableIK
        from robot.differential_ik import DifferentialIK
        from robot.table_approach import limit_targets
        from simulation.f14_scene import create_scene,resolve_joint_ids,initialize_home,apply_action,semantic_state
        from simulation.table_approach_monitor import TableApproachMonitor
        from simulation.cameras import rigid_link_path
        sim=sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1/cfg.PHYSICS_HZ,device=args.device))
        robot=create_scene(scene_seed=0)
        activate_contact_sensors('/World/F14')
        contact_paths=['/World/Table/geometry/mesh']+[
            f'/World/CollectionBox/{name}/geometry/mesh' for name in ('bottom','front','back','left','right')]
        sensors=[ContactSensor(ContactSensorCfg(prim_path=rigid_link_path(f'{side}_{finger}_gripper_link'),
                    update_period=0.,filter_prim_paths_expr=contact_paths))
                 for side in ('left','right') for finger in ('l','r')]
        sim.reset()
        arm_ids,finger_ids=resolve_joint_ids(robot)
        initialize_home(robot,arm_ids,finger_ids)
        # Regression for the real bug: OPEN fingers overlap the box margin at
        # HOME. They must be able to escape above the rim, without contact.
        open_home=HOME_ACTION.copy();open_home[14:]=1.
        dt=sim.get_physics_dt()
        for _ in range(60):
            apply_action(robot,open_home,arm_ids,finger_ids);robot.write_data_to_sim()
            sim.step(render=False);robot.update(dt)
        monitor=TableApproachMonitor(robot)
        ik=F14IK(F14_URDF_PATH)
        measured=semantic_state(robot,arm_ids,finger_ids)[0].cpu().numpy()
        initial_wrist=np.array([p.translation for p in ik.forward_kinematics(measured[:14])])
        lift_targets=list(ik.forward_kinematics(measured[:14]))
        for pose in lift_targets:
            pose.translation[2]+=.04
        lift_motion=JointMotionLimiter(measured[:14],cfg.ARM_MAX_VELOCITY,cfg.ARM_MAX_ACCELERATION,
                                       cfg.ARM_MAX_TRACKING_ERROR,allow_tracking_retreat=True)
        if args.differential_ik:
            print('QP START',measured[:14].tolist(),flush=True)
        follow=DifferentialIK(ik) if args.differential_ik else ReachableIK(ik)
        solve_times=[];qp_status_counts={};rotation_errors=[]

        def solve_targets(safe,measured,feedback):
            started=time.perf_counter()
            if args.differential_ik:
                # Match live Quest's fixed rotation even if measured PhysX
                # HOME has a small initial tracking error.
                for target,rotation in zip(safe,(LEFT_EE_DOWN_ROT,RIGHT_EE_DOWN_ROT)):
                    target.rotation=rotation.copy()
                previous_reference=None if follow.reference is None else follow.reference.copy()
                previous_velocity=follow.velocity.copy()
                result=follow.solve(*safe,measured[:14],dt=dt,feedback=feedback)
                debug_output=ROOT/'outputs/proxqp_check/first_rejected.json'
                if 'PROXQP_MAX_ITER_REACHED' in result.diagnostics['arm_status'] and not debug_output.exists():
                    debug_output.parent.mkdir(parents=True,exist_ok=True)
                    debug_output.write_text(json.dumps({'reference':(measured[:14] if previous_reference is None else previous_reference).tolist(),
                        'velocity':previous_velocity.tolist(),'measured':measured[:14].tolist(),
                        'targets':[p.homogeneous.tolist() for p in safe],'feedback':feedback,'diagnostics':result.diagnostics},indent=2))
                for status in result.diagnostics['arm_status']:
                    qp_status_counts[status]=qp_status_counts.get(status,0)+1
                rotation_errors.append(result.diagnostics['rotation_error_deg'])
            else:
                result=follow.solve(*safe,measured[:14],max_iter=cfg.IK_MAX_ITER,eps=cfg.IK_EPS,
                    dt=cfg.IK_DT,damping=cfg.IK_DAMPING,
                    limit_margin=cfg.IK_LIMIT_MARGIN,limit_gain=cfg.IK_LIMIT_GAIN)
            solve_times.append((time.perf_counter()-started)*1000)
            return result
        lift_force=np.zeros((len(sensors),len(contact_paths)));lift_failures=0
        for _ in range(300):
            data=monitor.sample(dt,True);data['boot_time']=1.
            safe,_,ready=limit_targets(lift_targets,data,1.,time.monotonic())
            assert ready
            result=solve_targets(safe,measured,data)
            lift_failures+=int(not result.success)
            action=open_home.copy()
            action[:14]=lift_motion.step(result.q,measured[:14],dt,result.success,
                                        reference_mode=args.differential_ik)
            apply_action(robot,action,arm_ids,finger_ids);robot.write_data_to_sim()
            sim.step(render=False);robot.update(dt)
            measured=semantic_state(robot,arm_ids,finger_ids)[0].cpu().numpy()
            for index,sensor in enumerate(sensors):
                sensor.update(dt,force_recompute=True)
                lift_force[index]=np.maximum(lift_force[index],sensor.data.force_matrix_w.norm(dim=-1).reshape(-1).cpu().numpy())
        lift_distance=np.array([p.translation for p in ik.forward_kinematics(measured[:14])])-initial_wrist
        if args.differential_ik:
            print('QP LIFT',lift_distance.tolist(),qp_status_counts,result.diagnostics,flush=True)
        assert np.all(lift_distance[:,2]>.035),f'Open HOME remained trapped: {lift_distance}'
        assert np.all(lift_force<.001),f'Lift hit table/box: {lift_force}'
        home=HOME_ACTION.copy();home[14:]=0.  # Narrow jaws for table and box-side approach.
        for _ in range(240):
            action=home.copy();action[:14]=lift_motion.step(home[:14],measured[:14],dt)
            apply_action(robot,action,arm_ids,finger_ids);robot.write_data_to_sim()
            sim.step(render=False);robot.update(dt)
            measured=semantic_state(robot,arm_ids,finger_ids)[0].cpu().numpy()
        monitor=TableApproachMonitor(robot)
        ik=F14IK(F14_URDF_PATH)
        follow=DifferentialIK(ik) if args.differential_ik else ReachableIK(ik)
        measured=semantic_state(robot,arm_ids,finger_ids)[0].cpu().numpy()
        targets=list(ik.forward_kinematics(measured[:14]))
        for target in targets:
            target.translation[2]-=.10  # Deliberately request below the tabletop.
        motion=JointMotionLimiter(measured[:14],cfg.ARM_MAX_VELOCITY,cfg.ARM_MAX_ACCELERATION,
                                  cfg.ARM_MAX_TRACKING_ERROR,allow_tracking_retreat=True)
        clearance=[];limited_count=failures=0
        maximum_force=np.zeros((len(sensors),len(contact_paths)))
        lateral_start=None;lateral_limited=0
        for step in range(args.steps*2):
            if step==args.steps:
                lateral_start=np.asarray([arm['wrist_position'] for arm in monitor.sample(dt,True)['arms']])
                targets=list(ik.forward_kinematics(measured[:14]))
                for target in targets:
                    target.translation[1]=0.  # Endpoint across the near box wall.
            feedback=monitor.sample(dt,True);feedback['boot_time']=1.
            safe,limited,ready=limit_targets(targets,feedback,1.,time.monotonic())
            assert ready
            limited_count+=int(any(limited))
            if step>=args.steps:
                lateral_limited+=int(any(limited))
            result=solve_targets(safe,measured,feedback)
            q,ok=result.q,result.success
            failures+=int(not ok)
            action=home.copy()
            action[:14]=motion.step(q if ok else measured[:14],measured[:14],dt,ok,
                                   reference_mode=args.differential_ik)
            apply_action(robot,action,arm_ids,finger_ids);robot.write_data_to_sim()
            sim.step(render=False);robot.update(dt)
            for index,sensor in enumerate(sensors):
                sensor.update(dt,force_recompute=True)
                force=sensor.data.force_matrix_w
                assert force is not None,'Table contact filter was not initialized'
                maximum_force[index]=np.maximum(maximum_force[index],force.norm(dim=-1).reshape(-1).cpu().numpy())
            measured=semantic_state(robot,arm_ids,finger_ids)[0].cpu().numpy()
            clearance.append([arm['clearance'] for arm in feedback['arms']])
        minimum=np.min(clearance,axis=0)
        final=monitor.sample(dt,True)
        assert np.all(minimum>0.),f'Collider reached table: {minimum}'
        assert np.all(maximum_force<.001),f'Gripper pressed a protected solid: {maximum_force} N'
        assert limited_count>0
        assert np.all(np.asarray(clearance)[args.steps-1]<np.asarray(clearance)[0]-.015),'No actual downward approach was tested'
        lateral_end=np.asarray([arm['wrist_position'] for arm in final['arms']])
        assert np.all(np.abs(lateral_end[:,1]-lateral_start[:,1])>.01),'No actual box approach was tested'
        assert lateral_limited>0
        assert np.all(np.abs(lateral_end[:,1])>.09),'Hand crossed a box wall'
        assert all(arm['box_clearance']>0 for arm in final['arms'])
        disabled=monitor.sample(dt,False)
        disabled['boot_time']=1.
        unmodified,flags,ready=limit_targets(targets,disabled,1.,time.monotonic())
        assert ready and not any(flags)
        for actual,raw in zip(unmodified,targets):
            np.testing.assert_array_equal(actual.translation,raw.translation)
        output=ROOT/('outputs/proxqp_check' if args.differential_ik else 'outputs/table_box_assist_check')
        output.mkdir(parents=True,exist_ok=True)
        report={'passed':True,'physics_steps':args.steps*2,'limited_steps':limited_count,'ik_failures':failures,
                'open_home_lift_mm':(lift_distance*1000).tolist(),'open_home_lift_ik_failures':lift_failures,
                'open_home_lift_contact_force_n':lift_force.tolist(),
                'box_limited_steps':lateral_limited,'box_approach_distance_mm':(np.abs(lateral_end[:,1]-lateral_start[:,1])*1000).tolist(),
                'minimum_clearance_mm':(minimum*1000).tolist(),'contact_filters':contact_paths,
                'maximum_contact_force_n':maximum_force.tolist(),'final':final,
                'inactive_targets_unchanged':True,'control':'Pinocchio + ProxQP' if args.differential_ik else 'XYZ limit before IK; joint drive limiter',
                'solver_ms_p50_p95_max':np.percentile(solve_times,[50,95,100]).tolist(),
                'solver_slow_samples':[(i,float(value)) for i,value in enumerate(solve_times) if value>1000/cfg.CONTROL_HZ],
                'qp_status_counts':qp_status_counts,'maximum_reference_rotation_error_deg':np.max(rotation_errors,axis=0).tolist() if rotation_errors else None}
        (output/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
        print('TABLE ASSIST GPU CHECK PASSED',json.dumps(report,indent=2),flush=True)
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        if sim:
            sim.clear_all_callbacks();sim.clear_instance();sim.stop()
        launcher.app.close()


if __name__=='__main__':
    main()
