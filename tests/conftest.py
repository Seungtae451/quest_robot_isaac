"""Explicit historical posture for tests of the retained downward IK tools."""
import numpy as np
import pytest


@pytest.fixture(scope="module")
def downward_home_q():
    # The old collision recording and fixed-orientation workspace use this
    # raised posture, independently of the editable live teleoperation HOME.
    return np.array([
        -1.0714420570, .7260463689, 1.5511059018, -1.4016318491,
        1.5896682282, -.6753974941, -1.0963101219,
        -1.0714388313, -.7260551171, -1.5511024793, -1.4016360913,
        -1.5896669913, .6753994612, -1.0963084577])
