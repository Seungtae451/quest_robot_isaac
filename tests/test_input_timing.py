"""Measured event rates, cached snapshots, and time-consistent filters."""
import multiprocessing as mp
from types import SimpleNamespace

import numpy as np
import pinocchio as pin
import pytest

from teleop.input_timing import event_timing, timed_filter_alpha
from teleop.xr_pose import RelativePoseMapper


def test_controller_rate_prop_is_set_without_changing_other_scene_elements():
    from teleop.televuer_adapter import _ControllerRateSession
    from config import teleop_config as cfg
    from vuer.schemas import MotionControllers, ImageBackground
    calls = []
    original = SimpleNamespace(upsert=lambda element, **kw: calls.append((element.serialize(), kw)),
                               CURRENT_WS_ID="test")
    session = _ControllerRateSession(original)
    controller = MotionControllers(stream=True, left=True, right=True)
    session.upsert(controller, to="bgChildren")
    assert calls[-1][0]["fps"] == cfg.QUEST_INPUT_HZ == 60
    assert calls[-1][0]["left"] and calls[-1][0]["right"]
    image = ImageBackground(key="frame")
    expected = image.serialize()
    session.upsert(image, to="bgChildren")
    assert calls[-1][0] == expected
    assert session.CURRENT_WS_ID == "test"


def test_rate_and_jitter_use_arrivals_not_polling_frequency():
    times = [0., 1., 1.01, 1.02, 1.06, 1.07]
    stats = event_timing(times, 1.07, window=.07)
    assert stats["samples"] == 5
    assert stats["hz"] == pytest.approx(4 / .07)
    assert stats["interval_p50_ms"] == pytest.approx(10.)
    assert stats["interval_max_ms"] == pytest.approx(40.)
    assert event_timing(times, 3.)["hz"] > 0.
    assert event_timing(times, 10.)["hz"] == 0.


@pytest.mark.parametrize("hz", [20, 30, 60, 90])
def test_filter_response_is_equal_at_different_event_rates(hz):
    start = np.eye(4)
    current = start.copy(); current[0, 3] = .2
    current[:3, :3] = pin.exp3(np.array([.4, -.2, .1]))
    mapper = RelativePoseMapper(start, pin.SE3.Identity())
    for _ in range(hz):
        target = mapper.target(current, dt=1. / hz)
    response = 1. - .8 ** 30
    np.testing.assert_allclose(target.translation, [.2 * response, 0., 0.], atol=1e-12)
    np.testing.assert_allclose(pin.log3(target.rotation), np.array([.4, -.2, .1]) * response, atol=1e-12)
    assert timed_filter_alpha(.2, 1 / 60) == pytest.approx(1. - np.sqrt(.8))


def test_snapshot_cache_refreshes_for_head_and_controller_but_never_masks_staleness(monkeypatch):
    from teleop.televuer_adapter import QuestInterface
    tv = QuestInterface.__new__(QuestInterface)
    tv.tvuer = SimpleNamespace(snapshot_lock=mp.RLock(), controller_serial=mp.Value("Q", 1),
                              head_serial=mp.Value("Q", 1), controller_time=mp.Value("d", 9.9),
                              head_time=mp.Value("d", 9.9), controller_valid=mp.Value("b", True),
                              process=SimpleNamespace(is_alive=lambda: True))
    tv._snapshot_key = tv._snapshot_data = None
    calls = []
    def convert():
        calls.append(1)
        return SimpleNamespace(motion_data_ready=True, left_wrist_pose=np.eye(4), right_wrist_pose=np.eye(4))
    tv.get_tele_data = convert
    monkeypatch.setattr("teleop.televuer_adapter.time.monotonic", lambda: 10.)
    assert tv.snapshot()[1]
    assert tv.snapshot()[1] and len(calls) == 1
    tv.tvuer.head_serial.value += 1
    assert tv.snapshot()[1] and len(calls) == 2
    tv.tvuer.controller_serial.value += 1
    assert tv.snapshot()[1] and len(calls) == 3
    tv.tvuer.controller_valid.value = False
    assert not tv.snapshot()[1]
    tv.tvuer.controller_valid.value = True
    monkeypatch.setattr("teleop.televuer_adapter.time.monotonic", lambda: 10.7)
    assert not tv.snapshot()[1] and len(calls) == 3
