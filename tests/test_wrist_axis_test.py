"""Check diagnostic frame/sign reporting and trial interruption handling."""
import json

import numpy as np
import pinocchio as pin

from teleop.wrist_axis_test import WristAxisTest, rotation_vectors


def test_world_axis_and_sign_are_preserved_with_rotated_neutral():
    neutral = pin.exp3(np.array([0., 0., np.pi / 2]))
    for sign in (-1, 1):
        current = pin.exp3(np.array([sign * np.deg2rad(15), 0., 0.])) @ neutral
        local, world = rotation_vectors(neutral, current)
        np.testing.assert_allclose(world, [sign * 15., 0., 0.], atol=1e-10)
        np.testing.assert_allclose(local, [0., -sign * 15., 0.], atol=1e-10)


def test_trial_records_axis_mismatch_actual_lag_and_ik_failure(tmp_path):
    test = WristAxisTest(tmp_path)
    controllers = [np.eye(4), np.eye(4)]
    targets = [pin.SE3.Identity(), pin.SE3.Identity()]
    measured = [pin.SE3.Identity(), pin.SE3.Identity()]
    test.request_start()
    test.update(controllers, targets, measured, True, .03, now=0.)
    controllers[0][:3, :3] = pin.exp3(np.array([np.deg2rad(15), 0., 0.]))
    targets[0].rotation = pin.exp3(np.array([0., np.deg2rad(-15), 0.]))
    test.update(controllers, targets, measured, False, .04, now=12.)
    report = json.loads((test.directory / "summary.json").read_text())
    result = report["trials"][0]
    assert result["ik_failed_samples"] == 1
    peak = result["controller_peak_sample"]
    np.testing.assert_allclose(peak["controller_world_deg"], [15., 0., 0.], atol=1e-10)
    np.testing.assert_allclose(peak["target_world_deg"], [0., -15., 0.], atol=1e-10)
    np.testing.assert_allclose(peak["measured_world_deg"], [0., 0., 0.])
    np.testing.assert_allclose(peak["orientation_error_deg"], 15.)
    assert test.trial_index == 1 and not test.active


def test_tracking_interruption_retries_same_motion(tmp_path):
    test = WristAxisTest(tmp_path)
    test.request_start()
    test.update([np.eye(4)] * 2, [pin.SE3.Identity()] * 2,
                [pin.SE3.Identity()] * 2, True, .01, now=0.)
    test.interrupt("tracking lost")
    assert test.trial_index == 0 and not test.active
    assert test.summaries[-1]["interrupted"] == "tracking lost"
