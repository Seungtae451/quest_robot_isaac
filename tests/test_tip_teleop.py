"""Position-only grasp-tip control with opposite hands and rotating obstacles."""
from types import SimpleNamespace
import itertools
import numpy as np
import pinocchio as pin

from config import teleop_config as cfg
from robot.f14_config import F14_URDF_PATH, HOME_Q, GRIPPER_TIP_OFFSET
from robot.f14_ik import F14IK
from robot.differential_ik import DifferentialIK
from robot.tool_frame import tip_position
from teleop.xr_pose import front_facing_controls, TranslationPoseMapper
from robot.table_approach import limit_targets


def setup():
    ik=F14IK(F14_URDF_PATH,ee_offset=GRIPPER_TIP_OFFSET)
    return ik,DifferentialIK(ik,position_only=True)


def test_tip_frame_jacobian_matches_finite_difference_and_measured_pose():
    ik,_=setup(); wrists=F14IK(F14_URDF_PATH).forward_kinematics(HOME_Q)
    tips=ik.forward_kinematics(HOME_Q)
    for wrist,tip in zip(wrists,tips):
        expected=wrist.translation+wrist.rotation@GRIPPER_TIP_OFFSET
        np.testing.assert_allclose(tip.translation,expected,atol=1e-12)
        quaternion=pin.Quaternion(wrist.rotation)
        np.testing.assert_allclose(tip_position(np.r_[wrist.translation,quaternion.w,quaternion.x,quaternion.y,quaternion.z]),expected)
    full=ik.arm_to_full_q(HOME_Q)
    jac=pin.computeFrameJacobian(ik.model,ik.data,full,ik.left_ee_frame,pin.ReferenceFrame.LOCAL)
    for index in range(7):
        moved=HOME_Q.copy(); moved[index]+=1e-7
        numerical=(ik.forward_kinematics(moved)[0].translation-tips[0].translation)/1e-7
        np.testing.assert_allclose(numerical,tips[0].rotation@jac[:3,ik.arm_v_indices[index]],atol=1e-7)


def test_crossed_hands_preserve_xyz_sign_and_cross_gripper_values():
    left,right=np.eye(4),np.eye(4); left[:3,3]=[1,2,3];right[:3,3]=[4,5,6]
    physical=SimpleNamespace(left_wrist_pose=left,right_wrist_pose=right,
        left_ctrl_triggerValue=.1,right_ctrl_triggerValue=.8,
        left_ctrl_squeezeValue=.2,right_ctrl_squeezeValue=.9)
    robot=front_facing_controls(physical)
    assert robot.left_ctrl_triggerValue==.8 and robot.right_ctrl_squeezeValue==.2
    mapper=TranslationPoseMapper(robot.left_wrist_pose,pin.SE3.Identity(),False,side='left')
    moved=right.copy();moved[:3,3]+=[.03,-.04,.05];moved[:3,:3]=pin.exp3(np.array([.7,.5,-.4]))
    np.testing.assert_allclose(mapper.target(moved).translation,[.03,-.04,.05])
    np.testing.assert_array_equal(mapper.target(moved).rotation,np.eye(3))
    np.testing.assert_array_equal(robot.right_wrist_pose,left)


def test_xyz_qp_ignores_target_rotation_and_does_not_fix_downward():
    ik,c=setup(); _,other=setup(); home=ik.forward_kinematics(HOME_Q)
    targets=[p.copy() for p in home]; targets[0].translation += [.04,.02,.03]
    rotated=[p.copy() for p in targets]
    for p in rotated:p.rotation=pin.exp3(np.array([1.,-.7,.4]))
    q=q2=HOME_Q.copy()
    previous=np.zeros(14)
    for _ in range(300):
        r=c.solve(*targets,q,dt=1/60);r2=other.solve(*rotated,q2,dt=1/60)
        np.testing.assert_allclose(r.q,r2.q,atol=1e-10)
        velocity=(r.q-q)*60
        assert np.max(np.abs(velocity-previous))<=cfg.ARM_MAX_ACCELERATION/60+1e-6
        q,q2,previous=r.q,r2.q,velocity
    actual=ik.forward_kinematics(q)
    assert max(r.remaining_mm)<.2
    assert np.linalg.norm(pin.log3(home[0].rotation.T@actual[0].rotation))>.02
    idle= c.__class__(ik,position_only=True)
    np.testing.assert_allclose(idle.solve(*home,HOME_Q,dt=1/60).q,HOME_Q,atol=1e-12)


def test_passive_rotation_cannot_sweep_finger_into_table():
    _,controller=setup()
    local=np.array(list(itertools.product([-.01,.01],[-.1,.1],[-.01,.01])))
    geometry=(np.array([[local.min(0),local.max(0)]]),
        [{'name':'table','bounds':[[-1,-1,-.1],[1,1,0]]}],[local])
    before=pin.SE3(np.eye(3),np.array([0.,0.,.03]))
    after=pin.SE3(pin.exp3(np.array([.5,0.,0.])),before.translation.copy())
    assert not controller._valid_step(before,after,before,geometry)
    lifted=pin.SE3(after.rotation,np.array([0.,0.,.2]))
    assert controller._valid_step(before,lifted,before,geometry)


def test_old_wrist_feedback_cannot_silently_drive_tip_guard():
    targets=[pin.SE3.Identity(),pin.SE3.Identity()]
    feedback={'enabled':True,'timestamp':10.,'boot_time':1.,'surface_z':0.,
        'table_xy':[-1,1,-1,1],'arms':[{},{}]}
    assert not limit_targets(targets,feedback,1.,10.,tcp_only=True)[2]


def test_position_qp_respects_rotating_finger_table_envelope():
    ik,controller=setup();home=ik.forward_kinematics(HOME_Q)
    # A short fingertip box extends 1 cm below the TCP. Put the synthetic
    # table 35 mm below HOME so this guard check is independent of HOME edits.
    # Express the measured envelope in each actual wrist/TCP orientation.
    world_corners=np.array(list(itertools.product([-.01,.01],[-.01,.01],[-.01,.01])))
    surface=min(p.translation[2] for p in home)-.035
    feedback={'enabled':True,'obstacles':[{'name':'table','bounds':[[-2,-2,-.1],[2,2,surface]]}],
        'arms':[{'offset_bounds':[world_corners.min(0),world_corners.max(0)],
                 'part_local_points':[(world_corners@pose.rotation).tolist()]} for pose in home]}
    targets=[p.copy() for p in home]
    for p in targets:p.translation += [.02,0.,-.08]
    q=HOME_Q.copy()
    for _ in range(450):
        result=controller.solve(*targets,q,dt=1/60,feedback=feedback);q=result.q
        for arm,pose in zip(feedback['arms'],ik.forward_kinematics(q)):
            points=np.array(arm['part_local_points'][0])@pose.rotation.T+pose.translation
            assert points[:,2].min()>=surface+.002-2e-7
    assert min(p.translation[2] for p in ik.forward_kinematics(q))<min(p.translation[2] for p in home)
