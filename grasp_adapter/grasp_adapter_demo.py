"""
Module 02 · Grasp Adapter — 실행 가능한 데모 (Phase 3)

Module 01(그립 예측 모델)이 내놓은 (u, v, theta)를 받아서
  1) 손가락 모양(프리셋) 결정
  2) 이미지 좌표 -> 카메라 기준 3D 좌표 (핀홀 카메라 모델, 평면 가정)
  3) 카메라 기준 -> 로봇 base 기준 (hand-eye calibration 변환 적용)
까지의 전체 계산을 보여준다.

실물 로봇 없이 지금 바로 실행 가능한 이유
------------------------------------------
- 카메라 intrinsic(FX, FY, CX, CY)은 카메라 사양서에 적힌 고정값이다.
- hand-eye calibration 변환(R, t)은 원래 Phase 2에서 실측해야 하는 값인데,
  여기서는 "예시값"을 하드코딩해서 계산 흐름만 미리 검증한다.
  실물 로봇이 오면 example_hand_eye_transform() 딱 한 함수만
  실측값으로 교체하면 나머지 코드는 그대로 재사용된다.

의존성: 표준 라이브러리(math)만 사용 — numpy 없이도 그대로 실행된다.
"""

import math

# ---------- 카메라 사양 (intrinsic parameters) ----------
# 카메라 스펙시트에 있는 값. 640x480 해상도 기준 예시.
FX, FY = 600.0, 600.0   # 초점거리 (픽셀 단위)
CX, CY = 320.0, 240.0   # 이미지 중심 (principal point)


def example_hand_eye_transform():
    """Phase 2에서 실측해야 할 (R, t) — 지금은 예시값.

    카메라가 30도 아래를 보도록 기울어 달려 있고,
    로봇 base 기준으로 (0.2, 0.0, 0.5)m 위치에 붙어 있다고 가정한 예시.
    R: 3x3 회전행렬(카메라가 로봇 기준에서 얼마나 돌아가 있는지)
    t: 평행이동 벡터(카메라가 로봇 base로부터 얼마나 떨어져 있는지)
    """
    theta = math.radians(30)
    c, s = math.cos(theta), math.sin(theta)
    R = [
        [1, 0, 0],
        [0, c, -s],
        [0, s,  c],
    ]
    t = [0.2, 0.0, 0.5]
    return R, t


def pixel_to_camera_xyz(u, v, depth_m):
    """1) 사진 속 위치(u, v) -> 카메라 기준 3D 좌표

    핀홀 카메라 모델의 역투영 공식:
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        Z = Z   (카메라 ~ 물체까지 거리. 테이블 높이가 고정이라 가정하고 구함)
    """
    Z = depth_m
    X = (u - CX) * Z / FX
    Y = (v - CY) * Z / FY
    return [X, Y, Z]


def matvec_add(R, p, t):
    """P_base = R @ P_cam + t (3x3 행렬 x 3벡터, numpy 없이 직접 계산)"""
    result = []
    for row, t_i in zip(R, t):
        val = sum(r * p_i for r, p_i in zip(row, p)) + t_i
        result.append(val)
    return result


def estimate_object_width_cm(pixel_width, depth_m):
    """참고: 픽셀 폭 -> 실제 폭(cm). 좌표 변환과 같은 핀홀 모델 공식을 재사용."""
    return (pixel_width * depth_m / FX) * 100  # m -> cm


def decide_preset(object_width_cm):
    """3) 물체 폭을 보고 손가락 모양(프리셋) 결정 — 규칙 기반, 지금은 이 정도로 단순하게."""
    if object_width_cm <= 3:
        return "PINCH"
    elif object_width_cm <= 7:
        return "TRIPOD"
    else:
        return "WRAP"


def run_grasp_adapter(u, v, theta_deg, pixel_width, table_height_m=0.30):
    print("===== Module 02: Grasp Adapter =====")
    print(f"[입력] Module 01 예측: u={u}, v={v}, θ={theta_deg}°, 픽셀폭={pixel_width}px\n")

    # 1단계: 손가락 모양 결정
    width_cm = estimate_object_width_cm(pixel_width, table_height_m)
    preset = decide_preset(width_cm)
    print(f"[1단계] 추정 물체 폭 = {width_cm:.1f}cm  ->  프리셋 = {preset}")

    # 2단계: 이미지 좌표 -> 카메라 기준 3D 좌표
    p_cam = pixel_to_camera_xyz(u, v, table_height_m)
    print(f"[2단계] 카메라 기준 3D 좌표 = "
          f"({p_cam[0]:.3f}, {p_cam[1]:.3f}, {p_cam[2]:.3f}) m")

    # 3단계: 카메라 기준 -> 로봇 base 기준
    R, t = example_hand_eye_transform()  # 실제로는 Phase 2 실측값으로 교체
    p_base = matvec_add(R, p_cam, t)
    theta_base = theta_deg  # 단순화: 카메라 광축과 로봇 접근축이 나란하다고 가정
    print(f"[3단계] 로봇 base 기준 3D 좌표 = "
          f"({p_base[0]:.3f}, {p_base[1]:.3f}, {p_base[2]:.3f}) m\n")

    # 최종 출력 -> Module 04(MoveIt)로 넘어갈 형태
    result = {
        "position_m": tuple(round(v_, 3) for v_ in p_base),
        "orientation_deg": theta_base,
        "hand_preset": preset,
    }
    print("[최종 출력 -> Module 04 MoveIt으로 전달]")
    for k, val in result.items():
        print(f"  {k}: {val}")
    return result


if __name__ == "__main__":
    # Module 01이 내놓았다고 가정한 예측값 (지난 설명과 동일한 예시)
    run_grasp_adapter(u=320, v=210, theta_deg=42, pixel_width=80)
