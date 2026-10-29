"""02단계 정답 예시. 먼저 노트북에서 직접 풀어보고, 막히면 여기서 확인하세요."""
import numpy as np


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def make_T(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def invert_T(T):
    R, p = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ p)


def transform_point(T, p):
    return (T @ np.append(p, 1.0))[:3]


def compose(*Ts):
    out = np.eye(4)
    for T in Ts:
        out = out @ T
    return out


def rpy_to_R(roll, pitch, yaw):
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def R_to_rpy(R):
    sp = -R[2, 0]
    if abs(sp) < 1 - 1e-9:
        pitch = np.arcsin(sp)
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:  # 짐벌락: yaw를 0으로 고정하고 roll로 몰아줌
        pitch = np.sign(sp) * np.pi / 2
        yaw = 0.0
        roll = np.arctan2(R[0, 1], R[1, 1]) if sp > 0 else np.arctan2(-R[0, 1], R[1, 1])
    return roll, pitch, yaw


def axis_angle_to_R(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def R_to_quat(R):
    """(w, x, y, z), w >= 0"""
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        q = np.array([s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1)
        q = np.zeros(4)
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = s / 4
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q = q / np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def quat_to_R(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
