"""
SO-101 + AmazingHand 결합 MJCF 생성 스크립트

SO-101의 실제 gripperframe site(공식 엔드이펙터 기준점)에
AmazingHand를 붙이고, 기존 스톡 그리퍼(moving_jaw)는 제거한다.

MuJoCo의 MjSpec.attach()를 사용 — XML을 손으로 이어붙이지 않고
공식 API로 모델을 조립한다 (MuJoCo 3.x 기능).

실행: python3 build_combined.py
결과: so101_amazinghand_scene.xml 생성 + 로드 검증까지 수행
"""

import mujoco
import os

HERE = os.path.dirname(os.path.abspath(__file__))

so101_path = os.path.join(HERE, "so101", "so101_new_calib.xml")
amazinghand_path = os.path.join(HERE, "amazinghand", "amazinghand.xml")
out_path = os.path.join(HERE, "so101_amazinghand_scene.xml")

# 1) SO-101 스펙 로드
so101 = mujoco.MjSpec.from_file(so101_path)

# 2) 스톡 그리퍼(jaw) 제거 — AmazingHand로 대체할 자리
jaw = so101.body("moving_jaw_so101_v1")
so101.delete(jaw)
# jaw를 움직이던 액추에이터도 정리 (더 이상 대상 관절이 없음)
for act in list(so101.actuators):
    if act.name == "gripper":
        so101.delete(act)
# 더 이상 아무 body도 쓰지 않는 mesh asset도 지워야 컴파일러가 파일을 안 찾는다
jaw_mesh = so101.mesh("moving_jaw_so101_v1")
so101.delete(jaw_mesh)

# 3) AmazingHand 스펙 로드
amazinghand = mujoco.MjSpec.from_file(amazinghand_path)

# 4) SO-101의 공식 엔드이펙터 기준점(gripperframe site)에 AmazingHand를 붙인다
gripperframe = so101.site("gripperframe")
so101.attach(amazinghand, site=gripperframe, prefix="ah_")

# 5) 배경/바닥/조명 추가 (원래 scene.xml에 있던 것과 동일한 구성)
so101.worldbody.add_light(
    pos=[0, 0, 3.5], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
)
so101.worldbody.add_geom(
    name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05], pos=[0, 0, 0]
)

# 6) 실제로 컴파일이 되는지(=에러 없이 물리모델이 완성되는지) 검증
model = so101.compile()
print(f"컴파일 성공: nq={model.nq}, nu={model.nu}, nbody={model.nbody}")
joint_names = [
    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)
]
print("전체 관절:", joint_names)

# 7) XML로 저장 (텍스트로도 열람/수정 가능하게)
# so101 스펙은 so101/so101_new_calib.xml 기준 상대경로("../assets")를 갖고 있는데,
# 결합 결과물은 mujoco_model/ 바로 밑에 저장되므로 "assets"로 바꿔줘야 한다.
# (spec.meshdir를 compile() 이후에 바꾸면 재검증 과정에서 옛 기준 경로로 다시 파일을 찾아
#  에러가 나므로, 저장된 XML 텍스트를 후처리하는 방식으로 우회한다.)
import re

xml_str = re.sub(r'meshdir="[^"]*"', 'meshdir="assets/"', so101.to_xml())
with open(out_path, "w") as f:
    f.write(xml_str)
print(f"저장 완료: {out_path}")

# 8) 저장한 파일을 다시 로드해서 최종 확인
model2 = mujoco.MjModel.from_xml_path(out_path)
data2 = mujoco.MjData(model2)
mujoco.mj_step(model2, data2)
print("재로딩 + 1스텝 시뮬레이션 성공 — 최종 파일 정상 동작 확인")
