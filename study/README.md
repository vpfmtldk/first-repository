# 로봇 기초 공부 (HCR-3A 프로젝트용)

| 단계 | 내용 | 상태 |
|---|---|---|
| **02** | [좌표 변환](02_coordinate_transforms/exercise.ipynb) — 회전행렬, 4x4 동차변환, 오일러각, 쿼터니언 | 연습 문제 + 자동 채점 |
| 03 | 순기구학 (관절각 → TCP) | 예정 |
| 04 | 역기구학과 자코비안 | 예정 |
| 05 | 직선 경로와 궤적 | 예정 |
| 06 | 캘리브레이션 (TCP, 평면, 핸드아이) | 예정 |

## 시작하기

```bash
pip install numpy scipy mujoco jupyter
cd study/02_coordinate_transforms
jupyter lab exercise.ipynb
```

- 노트북의 `raise NotImplementedError` 를 지우고 직접 구현한 뒤, 바로 아래 채점 셀을 실행하세요. (✅ 통과 / ❌ 힌트 / ⬜ 아직 안 함)
- 막히면 `solution.py` 를 참고하되, 먼저 스스로 시도해 보세요.
- 문제 6은 이 저장소의 MuJoCo 모델(`hcr3a_pick_place/scene.xml`)을 사용합니다. 저장소 구조를 바꾸지 않으면 그대로 동작합니다.

## 추천 자료

- **Modern Robotics** (Lynch & Park) — 좌표 변환, 기구학, 궤적 계획을 한 번에 다룹니다. 저자 페이지에서 무료 프리프린트 PDF(2019년 12월판)를 받을 수 있고 Coursera 강의도 있습니다.
  - [책 페이지 (Northwestern)](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)
  - [Coursera 강의](https://www.coursera.org/learn/modernrobotics-course1) (6개 과정, 순서대로 듣기를 권장 — [ClassCentral 설명](https://ClassCentral.com/course/modernrobotics-18595))
- **Robotics, Vision & Control, 3판 (Python)** (Peter Corke) — 파이썬 예제 중심. 좌표 변환 연습 후 결과 검증용으로 쓰기 좋습니다.
  - [rvc3python](https://pypi.org/project/rvc3python) (책 예제 패키지. 일부 의존성이 최신 파이썬에서 안 될 수 있다고 안내되어 있음)
  - [spatialmath-python](https://pypi.org/project/spatialmath-python/1.1.8) (`SE3`, `SO3`, `UnitQuaternion` 등. 이 연습의 정답 검증용으로 좋음)
- **Introduction to Robotics: Mechanics and Control, 4판** (Craig) — DH 파라미터 중심의 고전 교재. [Pearson 페이지](https://www.pearson.com/en-us/subject-catalog/p/introduction-to-robotics-mechanics-and-control/P200000003304/9780133489798) — ISBN과 출판 연도 표기가 여러 가지이므로 구매 전에 판본을 확인하세요.
- **MoveIt 2 문서:** [moveit.picknik.ai](https://moveit.picknik.ai/main/)
- **ROS 2 / tf2 튜토리얼:** [docs.ros.org](https://docs.ros.org/en/humble/) 의 Tutorials → Intermediate → tf2 (페이지 위치는 직접 확인하세요)

## 규약 메모 (헷갈리기 쉬운 것들)

- `T_a_b` : b 좌표계의 점을 a 좌표계로 옮기는 행렬. 합성은 `T_a_c = T_a_b @ T_b_c`.
- 오일러각: `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`. 로봇 컨트롤러마다 규약이 다를 수 있습니다.
- 쿼터니언: 이 저장소와 MuJoCo는 `(w, x, y, z)`, ROS 메시지는 `(x, y, z, w)` 입니다.
