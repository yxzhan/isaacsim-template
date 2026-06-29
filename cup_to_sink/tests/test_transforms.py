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

def test_quat_mul_ijk():
    """Test quaternion multiplication: i * j = k."""
    i = np.array([0, 1, 0, 0])
    j = np.array([0, 0, 1, 0])
    k = np.array([0, 0, 0, 1])
    result = T.quat_mul(i, j)
    np.testing.assert_allclose(result, k, atol=1e-6)

def test_quat_conj():
    """Test quaternion conjugation."""
    q = np.array([0.5, 0.5, 0.5, 0.5])
    expected = np.array([0.5, -0.5, -0.5, -0.5])
    result = T.quat_conj(q)
    np.testing.assert_allclose(result, expected, atol=1e-6)

def test_quat_rotate_90z():
    """Test rotating vector [1,0,0] by +90° rotation about +Z."""
    # Quaternion for +90° rotation about Z: [cos(π/4), 0, 0, sin(π/4)]
    q = np.array([np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)])
    v = np.array([1, 0, 0])
    expected = np.array([0, 1, 0])
    result = T.quat_rotate(q, v)
    np.testing.assert_allclose(result, expected, atol=1e-6)

def test_world_to_base_rotated():
    """Test world_to_base with rotated base frame.

    Base frame at position [1,0,0] rotated +90° about +Z.
    World point at [1,1,0] with identity orientation.
    In base frame, position should be approximately [1,0,0].
    """
    # Base at [1,0,0] with +90° rotation about Z
    base_pos = np.array([1., 0, 0])
    base_quat = np.array([np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)])
    base = np.concatenate([base_pos, base_quat])

    # World point at [1,1,0] with identity orientation
    world_point = np.array([1., 1, 0, 1, 0, 0, 0])

    # Transform to base frame
    result = T.world_to_base(world_point, base)

    # Check position only (first 3 elements)
    expected_pos = np.array([1, 0, 0])
    np.testing.assert_allclose(result[:3], expected_pos, atol=1e-6)
