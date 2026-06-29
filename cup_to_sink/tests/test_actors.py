# test_actors.py
import numpy as np
from cup_to_sink.actors import CupActor

class FakeCup:
    def get_world_pose(self): return (np.array([1.,2,0.8]), np.array([1.,0,0,0]))

CFG={"contact_points":[{"name":"side","position_obj":[0.045,0,0.08],"quat_obj":[1.,0,0,0],"pregrasp_distance":0.10}],
     "functional_points":[{"name":"c","position_obj":[0.,0,0],"quat_obj":[1.,0,0,0]}]}

def test_contact_point_world():
    a=CupActor(FakeCup(),CFG)
    p=a.get_contact_point(0)
    np.testing.assert_allclose(p[:3], np.array([1.045,2.0,0.88]), atol=1e-6)

def test_pregrasp_distance():
    assert CupActor(FakeCup(),CFG).pregrasp_distance(0)==0.10


class FakeCupRotated:
    """Cup at [1, 2, 0.8] rotated 90° about +Z axis."""
    def get_world_pose(self):
        # 90° about Z: quat = [cos(pi/4), 0, 0, sin(pi/4)]
        return (np.array([1.,2.,0.8]), np.array([np.cos(np.pi/4), 0., 0., np.sin(np.pi/4)]))

CFG_ROTATED = {
    "contact_points": [
        {
            "name": "side",
            "position_obj": [0.045, 0., 0.08],
            "quat_obj": [1., 0., 0., 0.],
            "pregrasp_distance": 0.10
        }
    ],
    "functional_points": [
        {"name": "c", "position_obj": [0., 0., 0.], "quat_obj": [1., 0., 0., 0.]}
    ]
}

def test_contact_point_world_rotated():
    """Verify composition order is T_world_cup @ T_cup_contact (not reversed).

    A 90° rotation about Z maps the object-frame offset [0.045, 0, 0.08]
    to world offset [0, 0.045, 0.08], so the expected world contact position
    is [1.0 + 0.0, 2.0 + 0.045, 0.8 + 0.08] = [1.0, 2.045, 0.88].
    """
    a = CupActor(FakeCupRotated(), CFG_ROTATED)
    p = a.get_contact_point(0)
    np.testing.assert_allclose(p[:3], np.array([1.0, 2.045, 0.88]), atol=1e-6)
