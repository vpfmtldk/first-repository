# SO-101 + AmazingHand MuJoCo 결합 모델

`SO-101`(TheRobotStudio/SO-ARM100) 팔에 `AmazingHand`(Pollen Robotics)를
엔드이펙터로 결합한 MuJoCo 시뮬레이션. 실제로 컴파일·로드·시뮬레이션 스텝까지
검증됐다 (아래 "검증한 것" 참고).

## 구성

```
mujoco_model/
├── assets/                      # 두 로봇의 STL 메쉬 전부 (공용, 21개 파일)
├── so101/
│   └── so101_new_calib.xml      # SO-101 원본 (TheRobotStudio 공식 파일, 무수정)
├── amazinghand/
│   └── amazinghand.xml          # AmazingHand 단순화 모델 (이번에 직접 작성)
├── build_combined.py            # 위 둘을 결합해 so101_amazinghand_scene.xml을 생성하는 스크립트
└── so101_amazinghand_scene.xml  # 최종 결합 결과물 (build_combined.py의 산출물)
```

## 출처 및 라이선스

- **SO-101**: [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) —
  `Simulation/SO101/so101_new_calib.xml`과 그 안에서 참조하는 메쉬를 그대로 받아왔다.
- **AmazingHand**: [pollen-robotics/AmazingHand](https://github.com/pollen-robotics/AmazingHand) —
  `cad/stl/*.stl` 중 필요한 부품만 받아와서 이름을 정리했다 (`palm.stl`,
  `proximal.stl`, `distal.stl`, `finger_frame_1.stl`, `wrist_interface.stl`,
  `so_arm_interface.stl` 등). `SO-ARM_Interface.stl`은 Pollen Robotics가 공식
  제공하는 SO-ARM 전용 손목 어댑터 부품이다.
- 각 저장소의 라이선스를 그대로 따른다. 실제 배포/사용 전 원 저장소의 라이선스 조건을 확인할 것.

## AmazingHand 모델 — 실측 vs 단순화

`amazinghand/amazinghand.xml`은 CAD(STL)를 파이썬으로 직접 파싱해서 얻은
**실측 치수**를 기반으로 만들었다:

| 부품 | 실측값 (bounding box) |
|---|---|
| proximal 링크 | 길이 ≈ 55mm |
| distal 링크 | 길이 ≈ 45mm (proximal과 약 9mm 겹침 = 관절부로 추정) |
| 손가락 단면 | 폭 10~16mm |
| finger_frame(밑동) | 폭 20mm |

**의도적으로 단순화한 부분** (실물과 다름, 반드시 알고 있어야 함):

- 실물 AmazingHand는 손가락 하나당 서보 2개가 **평행 링크(4절 링크) 기구**로
  굴곡+외전을 만들어내는 언더액추에이티드 구조다.
- 여기서는 그 대신 **관절 2개(외전 → 굴곡)의 단순 힌지 체인**으로 근사했고,
  proximal+distal은 하나의 강체로 합쳤다.
- 관절 총 개수(8 = 4손가락 × 2)는 실물과 동일하게 맞췄지만, 관절 하나하나의
  정확한 기구학(4절 링크의 커플링 특성)은 재현하지 않았다.
- **이 정도 단순화로 충분한 용도**: 손 모양 프리셋(PINCH/TRIPOD/WRAP) 검증,
  MoveIt 연동 테스트, 충돌 회피 경로 계획 확인
- **부족한 용도**: 정밀한 파지력·접촉 시뮬레이션, 실제 서보 토크 재현

팔-손 결합 지점(`SO-ARM_Interface.stl`이 실제로 어디에 어떤 각도로 붙는지)도
정확한 어셈블리 파일이 아니라 부품별 bounding box로 추정한 값이다. 실측/조립
사진과 대조해서 다듬는 걸 권장한다.

## 결합 방법 — SO-101의 `gripperframe`에 부착

SO-101의 `so101_new_calib.xml`에는 원래 스톡 그리퍼(`moving_jaw_so101_v1`)가
`gripper` 바디 밑에 달려 있고, 그 바로 옆에 공식 엔드이펙터 기준점인
`gripperframe`이라는 site가 있다. `build_combined.py`는:

1. 스톡 그리퍼(jaw) 바디와 관련 액추에이터·메쉬를 제거
2. MuJoCo `MjSpec.attach()` API로 `gripperframe` site 위치에 AmazingHand 스펙을 통째로 부착
3. 결과를 컴파일해서 에러 없는지 확인 후 XML로 저장

XML을 손으로 이어붙이지 않고 MuJoCo 공식 모델 조립 API(`MjSpec`, MuJoCo 3.x)를
쓴 이유는, 좌표 변환을 수동으로 계산하다 실수하는 걸 피하기 위해서다 — site에
붙이면 그 site의 위치·회전이 자동으로 반영된다.

## 검증한 것

```bash
pip install mujoco
python3 build_combined.py
```

- `MjSpec.compile()` 에러 없이 통과 (관절 13개: SO-101 팔 5개 + AmazingHand 8개, 바디 17개)
- 저장된 XML을 다시 `MjModel.from_xml_path()`로 로드 성공
- `mj_step()` 1000회 반복 실행 — NaN/발산 없이 안정적으로 유지됨
- 순기구학(`mj_forward`) 결과, AmazingHand 팜이 SO-101 손목 끝에서 자연스러운
  거리(월드 좌표 기준 팔 끝에서 약 10cm 더 뻗은 위치)에 위치함을 확인

**아직 검증 못 한 것**: 실제 렌더링 이미지로 육안 확인 (이 환경에 OpenGL
드라이버가 없어 오프스크린 렌더링 불가 — MuJoCo 뷰어(`python3 -m mujoco.viewer
--mjcf=so101_amazinghand_scene.xml`)가 되는 환경에서 직접 열어서 확인 필요).

## 다음에 할 일

1. **육안 확인**: MuJoCo 뷰어로 직접 열어서 어댑터 연결부·손가락 위치가
   실물과 비슷하게 보이는지 확인하고, 필요하면 `amazinghand.xml`의 palm
   부착 오프셋(`pos="0 0.03 0"`)과 손가락 간격(`replicate offset`)을 조정
2. **collision 정밀화**: 지금은 캡슐/박스 근사만 있음 — 필요하면 STL을
   convex decomposition해서 더 정확한 충돌 형상으로 교체
3. **평행 링크 재현**: 정밀한 접촉 시뮬레이션이 필요해지면 `equality`
   constraint나 tendon으로 실제 4절 링크 커플링을 흉내내는 방향으로 확장
