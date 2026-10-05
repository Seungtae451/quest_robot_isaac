"""One-centimetre/10-degree attachment and world-frame rotation following."""
import threading
import pytest
import time
from types import SimpleNamespace
import numpy as np
import pinocchio as pin
from robot.f14_config import F14_URDF_PATH, HOME_Q, GRIPPER_CONTROL_OFFSET
from robot.f14_ik import F14IK
from robot.tool_frame import control_position, controller_ee_basis, controller_alignment_errors
from robot.differential_ik import DifferentialIK
from config import teleop_config as cfg
from teleop.quest_ar_adapter import QuestARInterface
from teleop.xr_pose import WorldPoseMapper


def test_world_rotation_delta_is_preserved_without_start_jump():
    initial=np.eye(4);initial[:3,:3]=pin.exp3(np.array([.3,.1,-.5]))
    anchor=pin.SE3(pin.exp3(np.array([-.2,.4,.1])),np.array([.4,.2,.5]))
    mapper=WorldPoseMapper(initial,anchor,False,side='left')
    np.testing.assert_allclose(mapper.target(initial).homogeneous,anchor.homogeneous)
    turn=pin.exp3(np.array([.12,-.09,.05]));current=initial.copy()
    current[:3,:3]=turn@initial[:3,:3];current[:3,3]+=[.03,-.02,.04]
    target=mapper.target(current)
    np.testing.assert_allclose(target.rotation,turn@anchor.rotation,atol=1e-12)
    np.testing.assert_allclose(target.translation,anchor.translation+[.03,-.02,.04])


def test_center_fk_and_directed_axis_start_thresholds():
    wrist_ik=F14IK(F14_URDF_PATH);tcp_ik=F14IK(F14_URDF_PATH,ee_offset=GRIPPER_CONTROL_OFFSET)
    wrists=wrist_ik.forward_kinematics(HOME_Q);centers=tcp_ik.forward_kinematics(HOME_Q)
    interface=QuestARInterface.__new__(QuestARInterface)
    interface.lock=threading.RLock();interface.input=SimpleNamespace(placed=True)
    links={}
    for side,wrist,center in zip(('left','right'),wrists,centers):
        quat=pin.Quaternion(wrist.rotation)
        pose=np.r_[wrist.translation,quat.w,quat.x,quat.y,quat.z]
        links[side+'_dof7_link']=pose.tolist()
        np.testing.assert_allclose(control_position(pose),center.translation,atol=1e-12)
    interface.scene={'boot_time':1.,'timestamp':time.monotonic(),'links':links}
    poses=np.stack([c.homogeneous for c in centers])
    for i,side in enumerate(('left','right')):
        poses[i,:3,:3]=centers[i].rotation @ controller_ee_basis(side).T
    assert not interface.attachment_status(poses,1.)['reasons']
    poses[0,0,3]+=.009
    poses[0,:3,:3]=centers[0].rotation@pin.exp3(np.array([0,np.deg2rad(9.9),0]))@controller_ee_basis('left').T
    assert not interface.attachment_status(poses,1.)['reasons']
    poses[0,:3,:3]=centers[0].rotation@pin.exp3(np.array([0,np.deg2rad(10.1),0]))@controller_ee_basis('left').T
    assert not interface.attachment_status(poses,1.)['reasons']
    poses[0,0,3]+=.032
    assert interface.attachment_status(poses,1.)['reasons']==['LEFT_OUTSIDE_4CM']


def test_rotation_qp_tracks_a_rotating_center_without_exceeding_joint_limits():
    ik=F14IK(F14_URDF_PATH,ee_offset=GRIPPER_CONTROL_OFFSET)
    qp=DifferentialIK(ik,rotating_geometry=True)
    targets=list(ik.forward_kinematics(HOME_Q))
    targets[0].rotation=pin.exp3(np.array([.10,0.,0.]))@targets[0].rotation
    q=HOME_Q.copy();last_velocity=np.zeros(14)
    for _ in range(900):
        result=qp.solve(*targets,q,dt=1/60)
        velocity=(result.q-q)*60
        assert np.max(np.abs(velocity))<=cfg.ARM_MAX_VELOCITY+1e-6
        assert np.max(np.abs(velocity-last_velocity))<=cfg.ARM_MAX_ACCELERATION/60+1e-6
        assert np.all(result.q>=qp.lower) and np.all(result.q<=qp.upper)
        q=result.q;last_velocity=velocity
    for actual,target in zip(ik.forward_kinematics(q),targets):
        assert np.linalg.norm(actual.translation-target.translation)<.0003
        assert np.linalg.norm(pin.log3(actual.rotation.T@target.rotation))<.0003


def test_full_pose_rejects_rotating_gripper_through_table():
    ik=F14IK(F14_URDF_PATH,ee_offset=GRIPPER_CONTROL_OFFSET)
    qp=DifferentialIK(ik,rotating_geometry=True)
    before=pin.SE3(np.eye(3),np.array([0.,0.,.03]))
    after=pin.SE3(pin.exp3(np.array([.5,0.,0.])),before.translation.copy())
    corners=np.array([[x,y,z] for x in (-.01,.01) for y in (-.1,.1) for z in (-.01,.01)])
    geometry=(None,[{'bounds':[[-1.,-1.,-1.],[1.,1.,0.]]}],[corners])
    assert not qp._valid_step(before,after,after,geometry)
    before.translation[2]=.2;after.translation[2]=.2
    assert qp._valid_step(before,after,after,geometry)


def test_crossed_controller_axis_mapping():
    for side in ('left','right'):
        basis=controller_ee_basis(side)
        np.testing.assert_allclose(basis.T@basis,np.eye(3))
        assert np.linalg.det(basis)==1
        ee=np.eye(3);controller=basis.T
        np.testing.assert_allclose(controller_alignment_errors(controller,ee,side),[0.,0.])
        sign=1 if side=='left' else -1
        np.testing.assert_allclose(sign*controller[:,1],ee[:,0])
        np.testing.assert_allclose(-sign*controller[:,0],ee[:,2])


def test_translation_preserves_world_direction_for_both_arms():
    for side in ('left','right'):
        start=np.eye(4);anchor=pin.SE3(np.eye(3),np.array([.4,.2,.6]))
        mapper=WorldPoseMapper(start,anchor,False,side=side)
        moved=start.copy();moved[:3,3]=[.02,.03,.04]
        actual=mapper.target(moved)
        np.testing.assert_allclose(actual.translation,anchor.translation+[.02,.03,.04])
        np.testing.assert_allclose(actual.rotation,anchor.rotation)


def test_ar_packet_to_live_mapper_matches_displayed_controller_direction():
    from teleop.ar_world import world_matrix, controller_in_robot
    for yaw in (0.,np.pi,.7):
        world=world_matrix([1.,.7,-.5],yaw)
        neutral=np.eye(4);neutral[:3,3]=[.4,.2,.6]
        moved=neutral.copy();moved[:3,3]+=[.02,.03,.04]
        start=controller_in_robot(world@neutral,world)
        current=controller_in_robot(world@moved,world)
        for side in ('left','right'):
            mapper=WorldPoseMapper(start,pin.SE3(np.eye(3),neutral[:3,3]),False,side=side)
            target=mapper.target(current)
            np.testing.assert_allclose(target.translation-neutral[:3,3],[.02,.03,.04],atol=1e-12)
            # The robot's DISPLAYED displacement must equal raw XR grip displacement.
            display_delta=world[:3,:3]@(target.translation-neutral[:3,3])
            hand_delta=(world@moved)[:3,3]-(world@neutral)[:3,3]
            np.testing.assert_allclose(display_delta,hand_delta,atol=1e-12)


@pytest.mark.parametrize('dt',[1/60,1/30])
def test_fast_ik_exceeds_former_hidden_velocity_cap_and_converges(dt):
    ik=F14IK(F14_URDF_PATH,ee_offset=GRIPPER_CONTROL_OFFSET)
    qp=DifferentialIK(ik,rotating_geometry=True)
    goal=HOME_Q.copy();goal[:7]+=[.15,.12,-.1,.1,.08,0.,0.]
    targets=ik.forward_kinematics(goal)
    q=HOME_Q.copy();previous=np.zeros(14);maximum=0.
    for _ in range(round(2/dt)):
        result=qp.solve(*targets,q,dt=dt)
        velocity=(result.q-q)/dt
        assert np.max(np.abs(velocity))<=cfg.ARM_MAX_VELOCITY+1e-6
        assert np.max(np.abs(velocity-previous))<=cfg.ARM_MAX_ACCELERATION*dt+1e-6
        assert np.all(result.q>=qp.lower) and np.all(result.q<=qp.upper)
        maximum=max(maximum,float(np.max(np.abs(velocity))))
        q=result.q;previous=velocity
    assert maximum>.2  # old .05rad/.5s lag horizon capped outward speed near .1rad/s
    for actual,target in zip(ik.forward_kinematics(q),targets):
        assert np.linalg.norm(actual.translation-target.translation)<cfg.IK_EPS
        assert np.linalg.norm(pin.log3(actual.rotation.T@target.rotation))<cfg.IK_EPS
