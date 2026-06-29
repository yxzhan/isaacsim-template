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
def test_pregrasp_distance(): assert CupActor(FakeCup(),CFG).pregrasp_distance(0)==0.10
