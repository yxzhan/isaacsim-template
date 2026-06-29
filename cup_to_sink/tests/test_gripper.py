import numpy as np
from cup_to_sink import gripper as g


def test_width_to_fingers():
    np.testing.assert_allclose(g.width_to_finger_joints(0.08), [0.04, 0.04])


def test_fingers_to_width():
    assert abs(g.finger_joints_to_width(np.array([0.04, 0.04])) - 0.08) < 1e-9


def test_aperture():
    assert abs(g.width_to_aperture(0.04, 0.08, 0.0) - 0.5) < 1e-9


def test_aperture_clip():
    assert g.width_to_aperture(0.2, 0.08, 0.0) == 1.0 and g.width_to_aperture(-1, 0.08, 0.0) == 0.0
