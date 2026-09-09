"""
Hand-Eye Calibration — 터치한 점들로부터 (R, t) 계산하기 (Phase 2)

체커보드 모서리를 손끝으로 터치하면서 모은 점 쌍:
    P_camera_i (카메라가 본 그 지점의 좌표)
    P_robot_i  (그 순간 로봇 관절 정보로 계산한 손끝의 좌표)

이 점 쌍들로부터, 모든 i에 대해 아래 식이 최대한 잘 맞는 R(회전), t(이동)을 구한다.

    P_robot_i  ≈  R @ P_camera_i + t

이 계산법을 Kabsch 알고리즘(Procrustes 문제)이라고 부른다.
"""

import numpy as np


def solve_hand_eye(camera_points, robot_points):
    """점 쌍 리스트로부터 (R, t)를 구한다.

    camera_points, robot_points: 같은 개수의 [x, y, z] 리스트.
    같은 인덱스끼리 "같은 물리적 지점"이어야 한다 (터치한 순서대로 맞춰서 넣기).
    """
    P_cam = np.array(camera_points, dtype=float)   # (N, 3)
    P_rob = np.array(robot_points, dtype=float)     # (N, 3)
    assert len(P_cam) >= 3, "최소 3개 이상의 점이 필요합니다 (많을수록 정확해짐)"

    # 1단계: 두 점 집합 각각의 "무게중심(centroid)"을 구한다
    centroid_cam = P_cam.mean(axis=0)
    centroid_rob = P_rob.mean(axis=0)

    # 2단계: 무게중심을 원점으로 옮긴다 (이동 성분을 일단 제거 -> 회전만 남긴다)
    cam_centered = P_cam - centroid_cam
    rob_centered = P_rob - centroid_rob

    # 3단계: 두 점 집합이 "얼마나 같이 움직이는지" 나타내는 행렬 H를 만든다
    H = cam_centered.T @ rob_centered

    # 4단계: SVD(특이값분해)로 H를 분해해서 최적 회전 R을 뽑아낸다
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))  # 뒤집힘(반사) 방지
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T

    # 5단계: 회전을 적용한 뒤 남는 차이가 이동 t
    t = centroid_rob - R @ centroid_cam

    return R, t


def transform_error(camera_points, robot_points, R, t):
    """구해진 (R, t)가 얼마나 잘 맞는지 각 점의 오차(m)를 확인"""
    errors = []
    for p_cam, p_rob in zip(camera_points, robot_points):
        predicted = R @ np.array(p_cam) + t
        err = np.linalg.norm(predicted - np.array(p_rob))
        errors.append(err)
    return errors


if __name__ == "__main__":
    # ---------- 동작 검증: 정답을 알고 있는 가짜 데이터로 먼저 테스트 ----------
    print("===== 검증: 정답 R,t를 알고 있는 가짜 점들로 알고리즘이 맞게 구하는지 확인 =====\n")

    # "진짜 정답"이라고 가정한 회전 30도 + 이동값 (실전에서는 이게 미지수)
    true_theta = np.radians(30)
    true_R = np.array([
        [1, 0, 0],
        [0, np.cos(true_theta), -np.sin(true_theta)],
        [0, np.sin(true_theta),  np.cos(true_theta)],
    ])
    true_t = np.array([0.2, 0.0, 0.5])

    # 카메라가 봤다고 가정한 5개의 점 (체커보드 모서리 5곳)
    camera_points = [
        [0.10, 0.05, 0.30],
        [0.15, -0.05, 0.28],
        [-0.10, 0.08, 0.32],
        [0.02, -0.10, 0.29],
        [-0.08, -0.03, 0.31],
    ]
    # 그 점들을 "진짜 정답" R,t로 변환한 게 로봇이 터치해서 잰 좌표라고 가정
    robot_points = [true_R @ np.array(p) + true_t for p in camera_points]

    # 이제 이 점 쌍들만 갖고 R, t를 역산해본다 (진짜 답은 모르는 척)
    R_est, t_est = solve_hand_eye(camera_points, robot_points)

    print("역산한 R:\n", np.round(R_est, 3))
    print("\n실제 R:\n", np.round(true_R, 3))
    print("\n역산한 t:", np.round(t_est, 3))
    print("실제 t:   ", np.round(true_t, 3))

    errors = transform_error(camera_points, robot_points, R_est, t_est)
    print(f"\n각 점 오차(m): {[round(e, 6) for e in errors]}")
    print("\n(오차가 0에 가까우면 알고리즘이 정확히 맞춘 것)")
