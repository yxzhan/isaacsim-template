"""Quaternion and pose transform utilities (numpy only, no Isaac Sim dependencies)."""

import numpy as np


def quat_to_axis_angle(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion [w, x, y, z] to axis-angle rotation vector.

    Args:
        quat: quaternion [w, x, y, z]

    Returns:
        rotation vector (axis * angle in radians), shape (3,)
    """
    quat = np.asarray(quat, dtype=np.float64)
    w, x, y, z = quat[0], quat[1], quat[2], quat[3]

    # Vector part
    v = np.array([x, y, z])
    v_norm = np.linalg.norm(v)

    # Compute angle
    angle = 2.0 * np.arctan2(v_norm, w)

    # Normalize angle to [-pi, pi]
    if angle > np.pi:
        angle = angle - 2 * np.pi
    elif angle < -np.pi:
        angle = angle + 2 * np.pi

    # Handle identity/near-zero case
    if v_norm < 1e-10:
        return np.zeros(3, dtype=np.float64)

    # axis = v / v_norm, return axis * angle
    axis = v / v_norm
    return axis * angle


def axis_angle_to_quat(rotvec: np.ndarray) -> np.ndarray:
    """Convert rotation vector (axis * angle) to quaternion [w, x, y, z].

    Args:
        rotvec: rotation vector (axis * angle), shape (3,)

    Returns:
        quaternion [w, x, y, z]
    """
    rotvec = np.asarray(rotvec, dtype=np.float64)

    angle = np.linalg.norm(rotvec)

    # Handle identity/near-zero case
    if angle < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    axis = rotvec / angle
    w = np.cos(angle / 2.0)
    xyz = np.sin(angle / 2.0) * axis

    return np.array([w, xyz[0], xyz[1], xyz[2]], dtype=np.float64)


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions [w, x, y, z].

    Args:
        q1, q2: quaternions [w, x, y, z]

    Returns:
        product quaternion [w, x, y, z]
    """
    q1 = np.asarray(q1, dtype=np.float64)
    q2 = np.asarray(q2, dtype=np.float64)

    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]

    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2

    return np.array([w, x, y, z], dtype=np.float64)


def quat_conj(q: np.ndarray) -> np.ndarray:
    """Conjugate of a quaternion [w, x, y, z].

    Args:
        q: quaternion [w, x, y, z]

    Returns:
        conjugate quaternion [w, -x, -y, -z]
    """
    q = np.asarray(q, dtype=np.float64)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate a vector by a quaternion.

    Args:
        q: quaternion [w, x, y, z]
        v: 3D vector

    Returns:
        rotated vector
    """
    q = np.asarray(q, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)

    # q * [0, v] * q_conj
    v_quat = np.array([0.0, v[0], v[1], v[2]], dtype=np.float64)
    result_quat = quat_mul(q, quat_mul(v_quat, quat_conj(q)))

    return result_quat[1:4]


def quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion [w, x, y, z] to 3x3 rotation matrix.

    Args:
        quat: quaternion [w, x, y, z]

    Returns:
        3x3 rotation matrix
    """
    quat = np.asarray(quat, dtype=np.float64)
    w, x, y, z = quat[0], quat[1], quat[2], quat[3]

    # Normalize
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm

    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ], dtype=np.float64)

    return R


def matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [w, x, y, z].

    Uses Shepperd's method for numerical stability.

    Args:
        R: 3x3 rotation matrix

    Returns:
        quaternion [w, x, y, z]
    """
    R = np.asarray(R, dtype=np.float64)

    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0:
        S = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S

    return np.array([w, x, y, z], dtype=np.float64)


def make_T(position: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """Create a 4x4 transformation matrix from position and quaternion.

    Args:
        position: 3D position vector
        quat: quaternion [w, x, y, z]

    Returns:
        4x4 transformation matrix
    """
    position = np.asarray(position, dtype=np.float64)
    quat = np.asarray(quat, dtype=np.float64)

    R = quat_to_matrix(quat)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = position

    return T


def T_to_pose7d(T: np.ndarray) -> np.ndarray:
    """Extract position and quaternion from a 4x4 transformation matrix.

    Args:
        T: 4x4 transformation matrix

    Returns:
        7D pose [px, py, pz, qw, qx, qy, qz]
    """
    T = np.asarray(T, dtype=np.float64)

    position = T[:3, 3]
    R = T[:3, :3]
    quat = matrix_to_quat(R)

    return np.concatenate([position, quat])


def compose(T_a: np.ndarray, T_b: np.ndarray) -> np.ndarray:
    """Compose two transformation matrices (T_a @ T_b).

    Args:
        T_a, T_b: 4x4 transformation matrices

    Returns:
        4x4 composed transformation matrix
    """
    return T_a @ T_b


def invert_T(T: np.ndarray) -> np.ndarray:
    """Invert a transformation matrix.

    Args:
        T: 4x4 transformation matrix

    Returns:
        inverted 4x4 transformation matrix
    """
    T = np.asarray(T, dtype=np.float64)

    R = T[:3, :3]
    t = T[:3, 3]

    # For rotation matrices, R^{-1} = R^T
    R_inv = R.T
    t_inv = -R_inv @ t

    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv

    return T_inv


def pose7d_to_6d(pose7d: np.ndarray) -> np.ndarray:
    """Convert 7D pose (position + quaternion) to 6D (position + axis-angle).

    Args:
        pose7d: [px, py, pz, qw, qx, qy, qz]

    Returns:
        6D pose [px, py, pz, rvx, rvy, rvz]
    """
    pose7d = np.asarray(pose7d, dtype=np.float64)

    position = pose7d[:3]
    quat = pose7d[3:7]
    rotvec = quat_to_axis_angle(quat)

    return np.concatenate([position, rotvec])


def pose6d_to_7d(pose6d: np.ndarray) -> np.ndarray:
    """Convert 6D pose (position + axis-angle) to 7D (position + quaternion).

    Args:
        pose6d: [px, py, pz, rvx, rvy, rvz]

    Returns:
        7D pose [px, py, pz, qw, qx, qy, qz]
    """
    pose6d = np.asarray(pose6d, dtype=np.float64)

    position = pose6d[:3]
    rotvec = pose6d[3:6]
    quat = axis_angle_to_quat(rotvec)

    return np.concatenate([position, quat])


def world_to_base(pose7d_world: np.ndarray, base_pose7d: np.ndarray) -> np.ndarray:
    """Express a world pose in a base frame.

    Args:
        pose7d_world: 7D pose in world frame [px, py, pz, qw, qx, qy, qz]
        base_pose7d: 7D pose of base frame in world [px, py, pz, qw, qx, qy, qz]

    Returns:
        7D pose in base frame
    """
    pose7d_world = np.asarray(pose7d_world, dtype=np.float64)
    base_pose7d = np.asarray(base_pose7d, dtype=np.float64)

    T_world_base = make_T(base_pose7d[:3], base_pose7d[3:7])
    T_base_world = invert_T(T_world_base)
    T_world_pose = make_T(pose7d_world[:3], pose7d_world[3:7])

    T_base_pose = T_base_world @ T_world_pose

    return T_to_pose7d(T_base_pose)
