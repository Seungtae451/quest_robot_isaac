"""Predictive target limits, finite-table edges and collection-only feedback."""
from contextlib import closing
import copy
from pathlib import Path
import time

import numpy as np

from robot.table_approach import limit_targets
from teleop.action_protocol import ActionReceiver, ActionSender


class Pose:
    def __init__(self,position):
        self.translation=np.asarray(position,float)
        self.rotation=np.eye(3)
    def copy(self):
        return copy.deepcopy(self)


def feedback(closing=.04):
    return {'enabled':True,'boot_time':1.,'timestamp':10.,'surface_z':.3,
            'table_xy':[.15,.75,-.4,.4],
            'arms':[{'wrist_position':[.4,y,.57],'closing_speed':closing,
                     'offset_bounds':[[-.05,-.05,-.235],[.05,.05,0.]]} for y in (.18,-.18)]}


def test_velocity_brakes_before_hard_clearance_and_preserves_other_axes():
    targets=[Pose([.4,.18,.50]),Pose([.4,-.18,.59])]
    corrected,limited,ready=limit_targets(targets,feedback(),1.,10.01)
    assert ready and limited==[True,False]
    np.testing.assert_allclose(corrected[0].translation,[.4,.18,.557])
    np.testing.assert_array_equal(corrected[0].rotation,targets[0].rotation)
    assert targets[0].translation[2]==.50
    stationary,_,_=limit_targets(targets,feedback(0.),1.,10.01)
    assert abs(stationary[0].translation[2]-.537)<1e-9


def test_receding_or_outside_table_motion_is_not_arbitrarily_pushed_up():
    data=feedback(.2)
    targets=[Pose([.4,.18,.55]),Pose([.4,-.18,.59])]
    corrected,_,_=limit_targets(targets,data,1.,10.01)
    assert corrected[0].translation[2]<=data['arms'][0]['wrist_position'][2]+1e-9
    for arm in data['arms']:
        arm['wrist_position'][0]=1.
    targets=[Pose([1.,.18,.1]),Pose([1.,-.18,.1])]
    corrected,limited,ready=limit_targets(targets,data,1.,10.01)
    assert ready and limited==[False,False]
    np.testing.assert_array_equal(corrected[0].translation,targets[0].translation)


def test_stale_or_wrong_session_feedback_fails_closed_but_off_is_valid():
    targets=[Pose([.4,.18,.5]),Pose([.4,-.18,.5])]
    assert not limit_targets(targets,None,1.,10.01)[2]
    assert not limit_targets(targets,feedback(),1.,11.)[2]
    assert not limit_targets(targets,feedback(),2.,10.01)[2]
    invalid=feedback();invalid['timestamp']=None
    assert not limit_targets(targets,invalid,1.,10.01)[2]
    disabled={'enabled':False,'boot_time':1.,'timestamp':10.}
    corrected,limited,ready=limit_targets(targets,disabled,1.,10.01)
    assert ready and limited==[False,False]
    np.testing.assert_array_equal(corrected[0].translation,targets[0].translation)


def test_monitor_side_channel_does_not_change_action_state_or_packet():
    with closing(ActionReceiver('127.0.0.1',0)) as receiver:
        with closing(ActionSender('127.0.0.1',receiver.socket.getsockname()[1])) as sender:
            data=feedback()
            data['timestamp']=time.monotonic()
            receiver.table_feedback=data
            initial=receiver.action.copy()
            sender.poll_feedback()
            receiver.poll()
            assert sender.poll_feedback() and sender.ready
            assert sender.table_feedback['enabled']
            assert sender.table_feedback['boot_time']==sender.boot_time
            np.testing.assert_array_equal(receiver.action,initial)
            assert receiver.latest is None


def test_inference_does_not_import_collection_assistance():
    root=Path(__file__).resolve().parents[1]
    for filename in ('simulation/isaac_policy_eval.py','scripts/serve_openpi_f14.py','scripts/open_loop_eval_f14.py'):
        source=(root/filename).read_text()
        assert 'table_approach' not in source
        assert 'table_assist_config' not in source


def box_feedback(start):
    data=feedback(0.)
    data['obstacles']=[{'name':'wall','bounds':[[.49,-.1,.3],[.51,.1,.34]]},
                       {'name':'bottom','bounds':[[.49,-.1,.3],[.67,.1,.308]]}]
    for arm in data['arms']:
        arm['wrist_position']=list(start)
        arm['offset_bounds']=[[-.01,-.01,-.05],[.01,.01,0.]]
        arm['part_bounds']=[arm['offset_bounds']]
        arm['velocity_bounds']=[[0.,0.,0.],[0.,0.,0.]]
    return data


def test_box_wall_blocks_entire_path_even_when_endpoint_is_clear():
    data=box_feedback([.4,0.,.37])
    targets=[Pose([.7,0.,.37]),Pose([.7,0.,.37])]
    corrected,flags,ready=limit_targets(targets,data,1.,10.01)
    assert ready and all(flags)
    assert .47<corrected[0].translation[0]<.478001
    np.testing.assert_array_equal(targets[0].translation,[.7,0.,.37])
    # Lifting over the rim, then entering from above, remains possible.
    targets=[Pose([.7,0.,.42]),Pose([.7,0.,.42])]
    data=box_feedback([.4,0.,.42])
    corrected,flags,ready=limit_targets(targets,data,1.,10.01)
    assert ready and not any(flags)
    data=box_feedback([.6,0.,.42])
    targets=[Pose([.6,0.,.37]),Pose([.6,0.,.37])]
    corrected,flags,ready=limit_targets(targets,data,1.,10.01)
    assert ready and not any(flags)
    # The bottom still blocks deeper insertion through its floor.
    targets=[Pose([.6,0.,.32]),Pose([.6,0.,.32])]
    corrected,flags,ready=limit_targets(targets,data,1.,10.01)
    assert ready and all(flags)
    assert corrected[0].translation[2]>=.36


def test_box_prediction_brakes_before_wall_and_allows_retreat():
    data=box_feedback([.46,0.,.37])
    for arm in data['arms']:
        arm['velocity_bounds']=[[.03,0.,0.],[.03,0.,0.]]
    targets=[Pose([.7,0.,.37]),Pose([.7,0.,.37])]
    corrected,flags,ready=limit_targets(targets,data,1.,10.01)
    assert ready and all(flags)
    assert .46<=corrected[0].translation[0]<.463001
    targets=[Pose([.4,0.,.37]),Pose([.4,0.,.37])]
    corrected,flags,ready=limit_targets(targets,data,1.,10.01)
    assert ready and not any(flags)
    # A small tracking error inside the margin must not trap a retreat.
    data=box_feedback([.479,0.,.37])
    assert not any(limit_targets(targets,data,1.,10.01)[1])
    deeper=[Pose([.7,0.,.37]),Pose([.7,0.,.37])]
    corrected,flags,ready=limit_targets(deeper,data,1.,10.01)
    assert ready and all(flags)
    np.testing.assert_array_equal(corrected[0].translation,[.479,0.,.37])
    # Two fingers are separate envelopes, not a solid filling their gap.
    data=box_feedback([.4,0.,.37])
    for arm in data['arms']:
        arm['part_bounds']=[[[-.01,-.15,-.05],[.01,-.12,0.]],
                            [[-.01,.12,-.05],[.01,.15,0.]]]
    targets=[Pose([.7,0.,.37]),Pose([.7,0.,.37])]
    assert not any(limit_targets(targets,data,1.,10.01)[1])


def test_table_side_crossing_is_blocked_before_the_edge():
    data=box_feedback([0.,0.,.28])
    data['obstacles']=[{'name':'table','bounds':[[.15,-.4,.26],[.75,.4,.30]]}]
    targets=[Pose([.4,0.,.32]),Pose([.4,0.,.32])]
    corrected,flags,ready=limit_targets(targets,data,1.,10.01)
    assert ready and all(flags)
    assert corrected[0].translation[0]<.138


def test_initial_margin_overlap_allows_lift_but_not_deeper_or_crossing_another_solid():
    data=box_feedback([.479,0.,.37])
    # The nearest face is -X, but +Z also escapes safely over the rim.
    lift=[Pose([.479,0.,.43]),Pose([.479,0.,.43])]
    corrected,flags,ready=limit_targets(lift,data,1.,10.01)
    assert ready and not any(flags)
    np.testing.assert_allclose(corrected[0].translation,lift[0].translation)
    # Drop an inward component rather than trapping a diagonal lift.
    diagonal=[Pose([.52,0.,.43]),Pose([.52,0.,.43])]
    corrected,flags,ready=limit_targets(diagonal,data,1.,10.01)
    assert ready and all(flags)
    np.testing.assert_allclose(corrected[0].translation,[.479,0.,.43])
    data['obstacles'].append({'name':'ceiling','bounds':[[.45,-.1,.40],[.52,.1,.41]]})
    corrected,flags,ready=limit_targets(lift,data,1.,10.01)
    assert ready and all(flags)
    assert corrected[0].translation[2] < .398001


def test_actual_open_home_can_lift_from_box_margin():
    import json
    path=Path(__file__).parent/'fixtures/open_home_collision_feedback.json'
    # Captured from Isaac's actual open fingers, not a hand-picked envelope.
    data=json.loads(path.read_text())
    data['timestamp']=10.;data['boot_time']=1.
    targets=[Pose(np.array(arm['wrist_position'])+[0.,0.,.04]) for arm in data['arms']]
    corrected,flags,ready=limit_targets(targets,data,1.,10.01)
    assert ready and not any(flags)
    for actual,target in zip(corrected,targets):
        np.testing.assert_allclose(actual.translation,target.translation)
