import numpy as np
from cup_to_sink import randomize

RND = {
    "cup_xlim": [0.0, 0.1],
    "cup_ylim": [0.0, 0.1],
    "cup_yaw_range": [-3.14, 3.14],
    "cup_z_offset_range": [0.0, 0.0],
    "sink_target_sample_xy": True,
    "sink_target_xlim": [-0.02, 0.02],
    "sink_target_ylim": [-0.02, 0.02],
}


def test_determinism():
    a = randomize.sample_episode(
        7, RND, np.array([1.0, 1, 0.8, 1, 0, 0, 0]), np.array([0.5, 0.0])
    )
    b = randomize.sample_episode(
        7, RND, np.array([1.0, 1, 0.8, 1, 0, 0, 0]), np.array([0.5, 0.0])
    )
    np.testing.assert_allclose(a["cup_xy"], b["cup_xy"])
    np.testing.assert_allclose(a["sink_target_pose7d"], b["sink_target_pose7d"])


def test_different_seeds_differ():
    a = randomize.sample_episode(
        1, RND, np.array([1.0, 1, 0.8, 1, 0, 0, 0]), np.array([0.5, 0.0])
    )
    b = randomize.sample_episode(
        2, RND, np.array([1.0, 1, 0.8, 1, 0, 0, 0]), np.array([0.5, 0.0])
    )
    assert not np.allclose(a["cup_xy"], b["cup_xy"])


def test_within_limits():
    a = randomize.sample_episode(
        3, RND, np.array([1.0, 1, 0.8, 1, 0, 0, 0]), np.array([0.5, 0.0])
    )
    assert (
        RND["cup_xlim"][0]
        <= a["cup_xy"][0] - 0.5
        <= RND["cup_xlim"][1]
    )
