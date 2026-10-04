"""Speed units, unequal arrival times and tracking gaps affect real comparisons."""
import numpy as np

from teleop.motion_speed import MotionSpeedMonitor


def test_known_hand_and_robot_speeds_with_duplicate_feedback():
    monitor = MotionSpeedMonitor()
    for t in np.linspace(0., 1., 61):
        hand = np.array([[.20*t, 0., 0.], [0., -.10*t, 0.]])
        robot = hand * .25
        monitor.observe('controller', t, hand)
        monitor.observe('measured_robot', t, robot)
        monitor.observe('measured_robot', t, robot + 10.)  # Re-read cached pose.
    stats = monitor.summary(1.)
    np.testing.assert_allclose(stats['controller']['mean_cm_s'], [20., 10.])
    np.testing.assert_allclose(stats['measured_robot']['mean_cm_s'], [5., 2.5])


def test_gap_and_reset_do_not_turn_recalibration_into_high_speed():
    monitor = MotionSpeedMonitor()
    monitor.observe('controller', 0., np.zeros((2, 3)))
    monitor.observe('controller', .2, np.ones((2, 3)) * .01)
    assert monitor.summary(.2)['controller'] is not None
    assert monitor.summary(1.)['controller'] is None
    monitor.observe('controller', 1., np.ones((2, 3)) * 100.)
    assert monitor.summary(1.)['controller'] is None
    monitor.observe('controller', 1.2, np.ones((2, 3)) * 100.)
    np.testing.assert_allclose(monitor.summary(1.2)['controller']['mean_cm_s'], [0., 0.])
    monitor.reset()
    assert monitor.summary(1.2) == {}
