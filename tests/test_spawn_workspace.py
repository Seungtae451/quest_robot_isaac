"""Actual IK spawn points, grasp-frame offsets, cache validity and visibility."""
import numpy as np

from config import tabletop_config as cfg
from config import teleop_config
from robot.f14_config import F14_URDF_PATH, HOME_Q, LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT
from robot.f14_ik import F14IK
from robot.spawn_workspace import ensure_spawn_pool, workspace_signature, grasp_wrist_position, cube_is_in_camera
from simulation.tabletop import sample_cube_poses, table_surface_height


def test_spawn_has_no_unverified_position_jitter():
    pool=ensure_spawn_pool()
    accepted={(p['x'],p['y']) for p in pool['candidates']}
    for seed in range(256):
        for pos,_ in sample_cube_poses(seed):
            assert pos[:2] in accepted
            assert cube_is_in_camera(*pos[:2],table_surface_height())
            assert abs(pos[1]-cfg.BOX_CENTER_XY[1])>=cfg.BOX_SIZE[1]/2+cfg.CUBE_GRIPPER_BOX_CLEARANCE


def test_saved_waypoints_reach_cube_center_and_box_with_fixed_orientation():
    pool=ensure_spawn_pool();ik=F14IK(F14_URDF_PATH)
    for name,side in (('left',0),('right',1)):
        points=[p for p in pool['candidates'] if p['side']==name]
        for index in np.linspace(0,len(points)-1,12,dtype=int):
            p=points[index]
            for q in p['route_q']:
                assert np.isfinite(q).all()
                actual=ik.forward_kinematics(q)[side]
                rotation=(LEFT_EE_DOWN_ROT,RIGHT_EE_DOWN_ROT)[side]
                np.testing.assert_allclose(actual.rotation,rotation,atol=teleop_config.IK_EPS)
            grasp=ik.forward_kinematics(p['route_q'][1])[side]
            actual_center=grasp.translation+grasp.rotation@np.asarray(cfg.GRASP_LOCAL_OFFSET)
            np.testing.assert_allclose(actual_center,[p['x'],p['y'],table_surface_height()+cfg.CUBE_SIZE/2],atol=teleop_config.IK_EPS)
            release=ik.forward_kinematics(p['route_q'][4])[side]
            actual_center=release.translation+release.rotation@np.asarray(cfg.GRASP_LOCAL_OFFSET)
            np.testing.assert_allclose(actual_center,[*cfg.BOX_CENTER_XY,table_surface_height()+cfg.BOX_SIZE[2]+cfg.CUBE_SIZE/2+cfg.BOX_RELEASE_GAP],atol=teleop_config.IK_EPS)


def test_recheck_route_with_actual_100_iteration_solver():
    from scripts.build_ik_spawn_pool import verify_route
    ik=F14IK(F14_URDF_PATH);pool=ensure_spawn_pool()
    for side,name in enumerate(('left','right')):
        points=[p for p in pool['candidates'] if p['side']==name]
        for index in np.linspace(0,len(points)-1,6,dtype=int):
            p=points[index];candidate,reason=verify_route(ik,p['x'],p['y'],side)
            assert candidate is not None,reason
            assert candidate['max_joint_step_rad']<=cfg.IK_SPAWN_MAX_JOINT_STEP


def test_cache_signature_tracks_geometry_camera_and_solver(monkeypatch):
    original=workspace_signature()
    with monkeypatch.context() as m:
        m.setattr(cfg,'TABLE_CENTER',(.46,0,.28))
        assert workspace_signature()!=original
    with monkeypatch.context() as m:
        m.setattr(teleop_config,'BODY_CAMERA_FOCAL_LENGTH',18.)
        assert workspace_signature()!=original
    with monkeypatch.context() as m:
        m.setattr(teleop_config,'IK_MAX_ITER',50)
        assert workspace_signature()!=original
    with monkeypatch.context() as m:
        m.setattr(cfg,'CUBE_COUNT',8)
        assert workspace_signature()==original  # number doesn't change a point's IK


def test_camera_rejects_offscreen_location():
    assert not cube_is_in_camera(.4,2.,table_surface_height())
    assert not cube_is_in_camera(-1.,0.,table_surface_height())
