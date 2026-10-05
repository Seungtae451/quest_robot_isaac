"""Simulator-independent grasp-tip point in measured wrist link frames."""
import numpy as np
from robot.f14_config import GRIPPER_TIP_OFFSET, GRIPPER_CONTROL_OFFSET


def rotation_wxyz(quaternion):
    q=np.asarray(quaternion,float)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-12:
        raise ValueError('Invalid link quaternion')
    w,x,y,z=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def tip_position(link_pose):
    p=np.asarray(link_pose,float)
    if p.shape!=(7,) or not np.isfinite(p).all():
        raise ValueError('Expected xyz + wxyz link pose')
    return p[:3]+rotation_wxyz(p[3:])@GRIPPER_TIP_OFFSET


def control_position(link_pose):
    p=np.asarray(link_pose,float)
    return p[:3]+rotation_wxyz(p[3:])@GRIPPER_CONTROL_OFFSET


def alignment_axis_errors(controller_rotation, ee_rotation):
    """Directed X/Y column angles in the same fixed world frame, degrees."""
    c,e=np.asarray(controller_rotation),np.asarray(ee_rotation)
    return np.rad2deg(np.arccos(np.clip(np.sum(c[:,:2]*e[:,:2],axis=0),-1.,1.)))


def controller_ee_basis(robot_side):
    """Columns of EE axes in its opposite physical controller frame."""
    if robot_side == 'left':  # physical right: +Y -> X, +X -> -Z
        return np.array([[0.,0.,-1.],[1.,0.,0.],[0.,-1.,0.]])
    if robot_side == 'right':  # physical left: -Y -> X, +X -> Z
        return np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    raise ValueError('Unknown robot side')


def controller_alignment_errors(controller_rotation, ee_rotation, robot_side):
    mapped = np.asarray(controller_rotation) @ controller_ee_basis(robot_side)
    dots = np.sum(mapped[:,[0,2]] * np.asarray(ee_rotation)[:,[0,2]],axis=0)
    return np.rad2deg(np.arccos(np.clip(dots,-1.,1.)))
