"""World mapping, grip attachment, XR reset and button isolation regressions."""
import copy
from contextlib import closing
import json
import socket
import time

import numpy as np
import pytest

from teleop.ar_world import world_matrix, rigid_matrix, controller_in_robot, attachment_distances
from teleop.quest_ar_adapter import ARInputState, QuestARInterface
from teleop.ar_scene import ScenePublisher, SceneSubscriber, validate_scene
from robot.f14_config import HOME_Q, F14_URDF_PATH
from robot.f14_ik import F14IK
from teleop.xr_pose import DownwardPoseMapper
from teleop.ar_start import start_decision, start_message


def packet(sequence=0, *, placed=True, buttons=(), space=1, offset=0.):
    world = world_matrix([1., 0., -.5], .7)
    value = {'kind': 'input', 'sequence': sequence, 'space': space, 'placed': placed,
             'model_id': 'test', 'visible': True, 'world': world.flatten(order='F').tolist()}
    for side, y in (('left', .18), ('right', -.18)):
        grip = np.eye(4)
        grip[:3, 3] = [.4+offset, y, .57]
        grip = world @ grip
        value[side] = {'matrix': grip.flatten(order='F').tolist(), 'tracked': True, 'emulated': False,
            'state': {'triggerValue': .3, 'squeezeValue': .4,
                      'aButton': ('a' if side == 'right' else 'x') in buttons,
                      'bButton': side == 'right' and 'b' in buttons}}
    return value


@pytest.mark.parametrize('yaw', [0., .7, np.pi, -np.pi/2])
def test_world_mapping_metric_invariant_when_viewer_walks(yaw):
    world = world_matrix([1., 0., -.5], yaw)
    pose = np.eye(4); pose[:3, 3] = [.4, .18, .57]
    moved = pose.copy(); moved[:3, 3] += [.08, -.03, .10]
    first = controller_in_robot(world @ pose, world)
    current = controller_in_robot(world @ moved, world)
    np.testing.assert_allclose(current[:3, 3] - first[:3, 3], [.08, -.03, .10], atol=1e-12)
    # No viewer pose is an argument to this operation.
    np.testing.assert_allclose(world[:3, 2], [0, 1, 0])


def test_world_transform_rejects_scale_reflection_and_nonfinite():
    for diagonal in ([2., 1., 1., 1.], [-1., 1., 1., 1.], [np.nan, 1., 1., 1.]):
        with pytest.raises(ValueError):
            rigid_matrix(np.diag(diagonal))


def test_a_place_does_not_start_and_short_press_pose_is_exact():
    state = ARInputState('test')
    state.update(packet(0, placed=False), 10.)
    state.update(packet(1, buttons=('a',)), 10.01)
    assert state.buttons.drain(10.01) == []  # A used for placement, not START
    state.update(packet(2), 10.02)
    state.update(packet(3, buttons=('a',), offset=.03), 10.03)
    state.update(packet(4, offset=.04), 10.04)
    events = state.buttons.drain(10.04, with_poses=True)
    assert [e[0] for e in events] == ['a']
    np.testing.assert_allclose(events[0][1][0, :3, 3], [.43, .18, .57])
    state.update(packet(5, buttons=('a',)), 10.05)
    state.update(packet(6, buttons=('a',)), 10.06)
    assert state.buttons.drain(10.06) == ['a']


def test_pose_anchor_has_no_jump_even_with_attachment_offset():
    ik = F14IK(F14_URDF_PATH)
    anchors = ik.forward_kinematics(HOME_Q)
    state = ARInputState('test'); state.update(packet(offset=.03), 1.)
    controller = state.data.left_wrist_pose
    mapper = DownwardPoseMapper(controller, anchors[0], use_filter=False, side='left')
    np.testing.assert_allclose(mapper.target(controller).translation, anchors[0].translation)
    current = controller.copy(); current[2, 3] += .1
    target = mapper.target(current)
    np.testing.assert_allclose(target.translation, anchors[0].translation + [0,0,.1])
    current[:3, :3] = np.array([[0,-1,0],[1,0,0],[0,0,1]])
    np.testing.assert_allclose(mapper.target(current).rotation, target.rotation)


def test_tracking_gap_disarms_held_buttons_without_resetting_world():
    state = ARInputState('test')
    state.update(packet(0), 10.)
    state.update(packet(1, buttons=('a',)), 10.7)
    assert state.valid and state.placed
    assert state.buttons.drain(10.7) == []
    state.update(packet(2), 10.71)
    state.update(packet(3, buttons=('a',)), 10.72)
    assert state.buttons.drain(10.72) == ['a']


def test_reference_reset_invalidates_old_poses_and_requires_new_revision():
    state = ARInputState('test'); state.update(packet(0), 10.)
    epoch = state.epoch
    state.require_replacement()
    assert not state.valid and not state.placed and state.epoch > epoch
    assert not state.update(packet(1), 10.01)
    assert state.update(packet(2, space=2, buttons=('a',)), 10.02)
    assert state.buttons.drain(10.02) == []


def test_anchor_jump_cannot_silently_resume_with_same_space():
    state = ARInputState('test'); state.update(packet(0), 10.)
    moved = packet(1); moved['world'][12] += .2
    assert not state.update(moved, 10.01)
    assert state.minimum_space == 2 and not state.placed
    assert not state.update(packet(2), 10.02)
    assert state.update(packet(3, space=2), 10.03)


@pytest.mark.parametrize('mutator', [
    lambda p: p.update(model_id='wrong'),
    lambda p: p.update(visible=False),
    lambda p: p['left'].update(emulated=True),
    lambda p: p['right'].update(tracked=False),
    lambda p: p['right']['state'].update(triggerValue=float('nan')),
    lambda p: p['world'].__setitem__(0, 2.),
])
def test_invalid_tracking_does_not_refresh_liveness_or_queue_buttons(mutator):
    state = ARInputState('test'); state.update(packet(0), 10.)
    invalid = packet(1, buttons=('a',)); mutator(invalid)
    assert not state.update(invalid, 10.01)
    assert not state.valid and state.timestamp == 10.
    assert state.buttons.drain(10.01) == []


def test_reordered_input_cannot_refresh_timestamp_or_start():
    state = ARInputState('test'); state.update(packet(5), 10.)
    assert not state.update(packet(4, buttons=('a',)), 10.01)
    assert state.timestamp == 10.
    assert state.buttons.drain(10.01) == []


def test_five_cm_gate_is_measured_link_frame_and_side_specific():
    state = ARInputState('test'); state.update(packet(), 10.)
    poses = np.stack((state.data.left_wrist_pose, state.data.right_wrist_pose))
    wrists = np.array([[.4, .18, .57], [.4, -.18, .57]])
    poses[0,0,3] += .049
    np.testing.assert_allclose(attachment_distances(poses, wrists), [.049,0], atol=1e-12)
    poses[1,0,3] += .051
    assert not np.all(attachment_distances(poses, wrists) <= .05)
    assert np.min(attachment_distances(poses[::-1], wrists)) > .3


def scene(boot=1., sequence=1):
    return {'version':1, 'timestamp':time.monotonic(), 'boot_time':boot, 'sequence':sequence,
            'links':{'left_dof7_link':[.4,.18,.57,1,0,0,0], 'right_dof7_link':[.4,-.18,.57,1,0,0,0]},
            'cubes':[[.5,.2,.315,1,0,0,0]]}


def test_scene_transport_and_stale_scene_gate():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1',0)); port=probe.getsockname()[1]
    endpoint=f'tcp://127.0.0.1:{port}'
    with closing(ScenePublisher(endpoint)) as publisher, closing(SceneSubscriber(endpoint)) as subscriber:
        deadline=time.monotonic()+2
        value=None
        while value is None and time.monotonic()<deadline:
            publisher.submit(scene(sequence=2)); time.sleep(.02); value=subscriber.receive()
        assert value and value['sequence']==2
        assert publisher.error is None
        subscriber.latest=None
        old=scene(); old['timestamp']-=2
        publisher.submit(old); time.sleep(.05)
        assert subscriber.receive() is None


def test_scene_rejects_nonfinite_and_missing_measured_wrists():
    value=scene(); value['links'].pop('left_dof7_link')
    with pytest.raises(ValueError): validate_scene(value)
    value=scene(); value['cubes'][0][0]=float('nan')
    with pytest.raises(ValueError): validate_scene(value)


def test_attachment_requires_current_sim_boot_and_scene_timestamp():
    interface=QuestARInterface.__new__(QuestARInterface)
    import threading
    interface.lock=threading.RLock(); interface.input=ARInputState('test')
    interface.input.update(packet(), time.monotonic())
    interface.scene=scene(boot=42.)
    poses=np.stack((interface.input.data.left_wrist_pose,interface.input.data.right_wrist_pose))
    assert interface.attachment(poses,42.)[0]
    assert not interface.attachment(poses,41.)[0]
    interface.scene['timestamp']-=1
    assert not interface.attachment(poses,42.)[0]


def test_start_rejection_distinguishes_distance_from_home_and_keeps_press_pose():
    import threading
    interface = QuestARInterface.__new__(QuestARInterface)
    interface.lock = threading.RLock(); interface.input = ARInputState('test')
    now = time.monotonic() - .1
    interface.input.update(packet(0), now)
    interface.input.update(packet(1, buttons=('a',), offset=.049), now+.01)
    interface.input.update(packet(2, offset=.08), now+.02)
    press = interface.recording_button_events(with_poses=True)[0][1]
    interface.scene = scene(boot=42.)
    context = dict(fresh=True, receiver_ready=True, restarted=False,
                   mapper_ready=True, episode_available=True, command_busy=False)
    ready = {'state':'READY', 'start_allowed':True}
    at_press = start_decision(interface.attachment_status(press,42.), episode=ready, **context)
    assert at_press['allowed']
    np.testing.assert_allclose(at_press['distances_m'], [.049,.049], atol=1e-12)
    current = np.stack((interface.input.data.left_wrist_pose, interface.input.data.right_wrist_pose))
    outside = start_decision(interface.attachment_status(current,42.), episode=ready, **context)
    assert outside['reasons'] == ['LEFT_OUTSIDE_5CM','RIGHT_OUTSIDE_5CM']
    assert 'L=8.00cm R=8.00cm' in start_message(outside)
    blocked = {'state':'READY','start_allowed':False,
               'start_readiness':{'reasons':['HOME_JOINT_ERROR']}}
    inside_but_home = start_decision(interface.attachment_status(press,42.), episode=blocked, **context)
    assert inside_but_home['reasons'] == ['HOME_JOINT_ERROR']
    assert 'OUTSIDE' not in start_message(inside_but_home)


@pytest.mark.parametrize('condition,reason', [
    ('home','HOME_JOINT_ERROR'),('speed','HOME_STILL_MOVING'),
    ('gripper','HOME_GRIPPER_NOT_OPEN'),('timeout','HOME_COMMAND_STALE'),
])
def test_home_diagnostics_preserve_existing_start_thresholds(condition, reason):
    from simulation.episode_reset import home_start_status
    from robot.f14_config import HOME_ACTION
    measured = HOME_ACTION.copy(); measured[14:] = 1.
    velocity = np.zeros(14)
    assert home_start_status(measured,velocity,'ACTIVE')['allowed']
    state = 'ACTIVE'
    if condition == 'home': measured[0] += .016
    if condition == 'speed': velocity[1] = .025
    if condition == 'gripper': measured[14] = .97
    if condition == 'timeout': state = 'HOLD'
    result = home_start_status(measured,velocity,state)
    assert result['reasons'] == [reason] and not result['allowed']
    assert result['allowed'] == bool(state=='ACTIVE'
        and np.max(np.abs(measured[:14]-HOME_ACTION[:14])) < .015
        and np.max(np.abs(velocity)) < .025 and np.min(measured[14:]) > .97)


def test_ar_overlay_does_not_add_dataset_camera_features():
    from dataset.recording import CAMERAS, features_for
    shapes={name:(32,48,3) for name in CAMERAS}
    features=features_for(shapes)
    assert len([key for key in features if key.startswith('observation.images.')]) == 3
    assert features['action']['shape'] == (16,)
    assert features['observation.state']['shape'] == (16,)
    assert not any('quest' in key or 'passthrough' in key or 'head' in key for key in features)
