import numpy as np
from cup_to_sink import transforms as T

def test_axis_angle_roundtrip():
    q = np.array([np.cos(0.3), 0.0, np.sin(0.3), 0.0])  # 0.6 rad about +Y
    rv = T.quat_to_axis_angle(q)
    np.testing.assert_allclose(np.linalg.norm(rv), 0.6, atol=1e-6)
    q2 = T.axis_angle_to_quat(rv)
    # same rotation (allow sign flip of quaternion)
    assert np.allclose(q2, q, atol=1e-6) or np.allclose(q2, -q, atol=1e-6)

def test_identity_axis_angle():
    rv = T.quat_to_axis_angle(np.array([1.0, 0, 0, 0]))
    np.testing.assert_allclose(rv, np.zeros(3), atol=1e-9)

def test_pose_6d_7d_roundtrip():
    p7 = np.array([1.0, 2.0, 3.0, np.cos(0.2), 0.0, 0.0, np.sin(0.2)])
    p6 = T.pose7d_to_6d(p7)
    p7b = T.pose6d_to_7d(p6)
    np.testing.assert_allclose(p7b[:3], p7[:3], atol=1e-9)
    assert np.allclose(p7b[3:], p7[3:], atol=1e-6) or np.allclose(p7b[3:], -p7[3:], atol=1e-6)

def test_make_T_and_compose_identity():
    Tid = T.make_T(np.zeros(3), np.array([1.0,0,0,0]))
    np.testing.assert_allclose(Tid, np.eye(4), atol=1e-9)
    A = T.make_T(np.array([1.,0,0]), np.array([1.,0,0,0]))
    np.testing.assert_allclose(T.compose(A, A)[:3,3], np.array([2.,0,0]), atol=1e-9)

def test_world_to_base_at_origin_is_identity():
    base = np.array([0.,0,0,1,0,0,0])
    p = np.array([1.,2,3,1,0,0,0])
    np.testing.assert_allclose(T.world_to_base(p, base), p, atol=1e-9)

def test_world_to_base_translation():
    base = np.array([1.,0,0,1,0,0,0])
    p = np.array([3.,0,0,1,0,0,0])
    out = T.world_to_base(p, base)
    np.testing.assert_allclose(out[:3], np.array([2.,0,0]), atol=1e-9)

def test_invert_T():
    A = T.make_T(np.array([1.,2,3]), T.axis_angle_to_quat(np.array([0.1,0.2,0.3])))
    np.testing.assert_allclose(T.compose(A, T.invert_T(A)), np.eye(4), atol=1e-6)
