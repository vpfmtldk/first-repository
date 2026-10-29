"""연습 문제 자동 채점. 노트북에서 `from checker import *` 로 불러 씁니다."""
import os

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE = os.path.join(_HERE, "..", "..", "hcr3a_pick_place", "scene.xml")


def _run(title, fn):
    try:
        fn()
    except NotImplementedError:
        print(f"⬜ {title}: 아직 구현하지 않았습니다 (NotImplementedError)")
        return False
    except AssertionError as e:
        print(f"❌ {title}: {e}")
        return False
    except Exception as e:  # noqa: BLE001
        print(f"❌ {title}: 실행 중 오류 {type(e).__name__}: {e}")
        return False
    print(f"✅ {title}: 통과")
    return True


def _is_rotation(R, name):
    R = np.asarray(R, float)
    assert R.shape == (3, 3), f"{name}: 3x3 행렬이어야 합니다 (현재 {R.shape})"
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9), f"{name}: R.T @ R 이 단위행렬이 아닙니다 (직교 아님)"
    assert np.isclose(np.linalg.det(R), 1.0), f"{name}: det(R) = {np.linalg.det(R):.3f} (1이어야 함, -1이면 거울상)"


def check_rotations(rot_x, rot_y, rot_z):
    def f():
        a = np.deg2rad(90)
        # 손으로 계산 가능한 값: z축 90도 회전은 x축 단위벡터를 y축으로 보냅니다.
        assert np.allclose(rot_z(a) @ [1, 0, 0], [0, 1, 0], atol=1e-9), "rot_z(90°) @ [1,0,0] 은 [0,1,0] 이어야 합니다"
        assert np.allclose(rot_x(a) @ [0, 1, 0], [0, 0, 1], atol=1e-9), "rot_x(90°) @ [0,1,0] 은 [0,0,1] 이어야 합니다"
        assert np.allclose(rot_y(a) @ [0, 0, 1], [1, 0, 0], atol=1e-9), "rot_y(90°) @ [0,0,1] 은 [1,0,0] 이어야 합니다"
        for fn, n in ((rot_x, "rot_x"), (rot_y, "rot_y"), (rot_z, "rot_z")):
            _is_rotation(fn(0.7), n)
    return _run("rot_x / rot_y / rot_z", f)


def check_transforms(make_T, invert_T, transform_point, compose):
    def f():
        R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)  # z축 90도
        T = make_T(R, [1, 2, 3])
        assert T.shape == (4, 4), "make_T 는 4x4 행렬을 돌려줘야 합니다"
        assert np.allclose(T[3], [0, 0, 0, 1]), "마지막 행은 [0 0 0 1] 이어야 합니다"
        assert np.allclose(T[:3, :3], R) and np.allclose(T[:3, 3], [1, 2, 3]), "회전/이동이 제자리에 들어가지 않았습니다"
        assert np.allclose(T @ invert_T(T), np.eye(4), atol=1e-9), "T @ invert_T(T) 가 단위행렬이 아닙니다"
        assert np.allclose(invert_T(T)[:3, 3], -R.T @ [1, 2, 3]), "역변환의 이동 성분은 -R.T @ p 입니다"
        # 시나리오: 카메라가 로봇 베이스에서 (1, 0, 0.5)에 있고 z축으로 90도 돌아 있습니다.
        T_base_cam = make_T(R, [1, 0, 0.5])
        p_base = transform_point(T_base_cam, [1, 0, 0])
        assert np.allclose(p_base, [1, 1, 0.5]), f"카메라 기준 (1,0,0) → 베이스 기준 (1,1,0.5) 이어야 합니다. 현재 {np.round(p_base, 3)}"
        p_cam = transform_point(invert_T(T_base_cam), [1, 1, 0.5])
        assert np.allclose(p_cam, [1, 0, 0]), "베이스 기준 점을 카메라 기준으로 되돌리면 (1,0,0) 이어야 합니다"
        A, B = make_T(R, [1, 0, 0]), make_T(np.eye(3), [0, 2, 0])
        assert np.allclose(compose(A, B), A @ B), "compose(A, B) 는 A @ B 입니다"
        assert np.allclose(compose(A, B, A), A @ B @ A), "인자가 3개 이상이어도 왼쪽부터 곱해야 합니다"
    return _run("make_T / invert_T / transform_point / compose", f)


def check_euler(rpy_to_R, R_to_rpy, rot_x, rot_y, rot_z):
    def f():
        r, p, y = 0.3, -0.5, 1.1
        R = rpy_to_R(r, p, y)
        _is_rotation(R, "rpy_to_R")
        assert np.allclose(R, rot_z(y) @ rot_y(p) @ rot_x(r), atol=1e-9), "규약: R = Rz(yaw) @ Ry(pitch) @ Rx(roll) 입니다"
        assert np.allclose(R_to_rpy(R), (r, p, y), atol=1e-9), "R_to_rpy(rpy_to_R(r,p,y)) 가 원래 값으로 돌아와야 합니다"
        # 짐벌락: pitch = ±90° 에서는 roll/yaw 를 따로 구분할 수 없지만 회전행렬은 복원되어야 합니다.
        # 부동소수점 오차로 우연히 맞는 일이 없도록, cos(pitch)가 정확히 0이 되게 반올림한 행렬을 씁니다.
        for pitch in (np.pi / 2, -np.pi / 2):
            Rg = np.round(rpy_to_R(0.4, pitch, 0.9), 12)
            r2, p2, y2 = R_to_rpy(Rg)
            assert np.all(np.isfinite([r2, p2, y2])), "짐벌락에서 nan 이 나왔습니다"
            assert np.allclose(rpy_to_R(r2, p2, y2), Rg, atol=1e-6), (
                f"pitch={np.rad2deg(pitch):.0f}° (짐벌락)에서 복원에 실패했습니다. "
                "cos(pitch)=0 이면 roll 과 yaw 가 한 축으로 합쳐져서 atan2(0, 0) 이 되어버립니다")
    return _run("rpy_to_R / R_to_rpy (짐벌락 포함)", f)


def check_axis_angle(axis_angle_to_R, rot_z):
    def f():
        assert np.allclose(axis_angle_to_R([0, 0, 1], 0.8), rot_z(0.8), atol=1e-9), "z축 회전은 rot_z 와 같아야 합니다"
        R = axis_angle_to_R([1, 1, 1], 2 * np.pi / 3)  # (1,1,1) 축으로 120° 돌리면 x→y→z→x
        assert np.allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-9), "(1,1,1)축 120° 회전은 x축을 y축으로 보냅니다"
        assert np.allclose(axis_angle_to_R([0, 0, 5], 0.8), rot_z(0.8), atol=1e-9), "축은 정규화해서 써야 합니다"
    return _run("axis_angle_to_R (로드리게스 공식)", f)


def check_quaternion(R_to_quat, quat_to_R, rpy_to_R):
    def f():
        assert np.allclose(R_to_quat(np.eye(3)), [1, 0, 0, 0]), "단위 회전의 쿼터니언은 [1,0,0,0] (w,x,y,z) 입니다"
        q = R_to_quat(rpy_to_R(0, 0, np.pi / 2))
        assert np.allclose(q, [np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)], atol=1e-9), "z축 90°는 [cos45°, 0, 0, sin45°] 입니다"
        rng = np.random.default_rng(0)
        for _ in range(200):  # 모든 분기(trace 가 작은 경우 포함)를 거치도록 임의 회전을 많이 시험
            R = rpy_to_R(*rng.uniform(-np.pi, np.pi, 3))
            q = R_to_quat(R)
            assert abs(np.linalg.norm(q) - 1) < 1e-9, "쿼터니언은 단위 길이여야 합니다"
            assert q[0] >= -1e-12, "q 와 -q 는 같은 회전입니다. w >= 0 으로 통일하세요"
            assert np.allclose(quat_to_R(q), R, atol=1e-7), "quat_to_R(R_to_quat(R)) 가 R 로 돌아와야 합니다"
        R180 = rpy_to_R(np.pi, 0, 0)  # trace = -1 인 까다로운 경우
        assert np.allclose(quat_to_R(R_to_quat(R180)), R180, atol=1e-7), "x축 180° 회전(trace=-1)에서 실패했습니다"
    return _run("R_to_quat / quat_to_R", f)


def check_with_mujoco(compose, make_T):
    """실제 HCR-3A 모델에서 링크 좌표계를 곱해 TCP 위치를 재현할 수 있는지 확인합니다."""
    def f():
        import mujoco

        model = mujoco.MjModel.from_xml_path(_SCENE)
        data = mujoco.MjData(model)
        data.qpos[:6] = [0.6, -1.2, -0.5, -2.0, 1.2, 0.3]
        mujoco.mj_forward(model, data)

        body = model.body("link_6").id
        site = model.site("tcp").id
        T_world_link6 = make_T(data.xmat[body].reshape(3, 3), data.xpos[body])

        R_link6_tcp = np.zeros(9)
        mujoco.mju_quat2Mat(R_link6_tcp, model.site_quat[site])
        T_link6_tcp = make_T(R_link6_tcp.reshape(3, 3), model.site_pos[site])

        T_world_tcp = compose(T_world_link6, T_link6_tcp)
        err = np.linalg.norm(T_world_tcp[:3, 3] - data.site_xpos[site])
        assert err < 1e-9, f"TCP 위치가 MuJoCo 값과 {err * 1000:.2f} mm 다릅니다. 곱하는 순서를 확인하세요"
        assert np.allclose(T_world_tcp[:3, :3], data.site_xmat[site].reshape(3, 3), atol=1e-9), "TCP 자세가 MuJoCo 값과 다릅니다"
    return _run("실제 로봇(MuJoCo) 좌표계 체인", f)


def check_quiz(name, answer, correct, hint):
    ok = answer == correct
    print(("✅" if ok else "❌") + f" {name}: " + ("정답" if ok else f"다시 생각해보세요. 힌트: {hint}"))
    return ok
