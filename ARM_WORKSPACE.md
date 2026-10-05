# scikit-robot으로 F14 가동범위 보기

현재 **자유 방향·그리퍼 끝 TCP** 기준 이미지는 다음 명령으로 생성합니다.
원래 낮은 HOME과 현재 테이블·상자·큐브 생성 영역을 함께 표시하며, 양팔 각각
16,384개 관절 한계 내 자세를 scikit-robot FK로 계산합니다. 3D 및 XY/YZ/XZ
투영도이며, 평면별 특정 높이의 IK 단면은 아닙니다. 충돌·경로·실시간 IK
추종 여부는 검증하지 않습니다.

```bash
~/stkim_ws/.venv/bin/python scripts/visualize_current_f14_workspace.py
```

이미지: `outputs/arm_workspace/current_f14/workspace.png`.
관절 표본·손끝 위치는 같은 폴더의 `samples.npz`, 설정·범위는 `summary.json`에
저장합니다. 주황색은 로봇 왼팔, 파란색은 오른팔, 별은 HOME 손끝입니다.

## 이전 손목 원점·수직 자세 비교 도구

설치된 **scikit-robot 0.3.39** 환경은 `~/stkim_ws/.venv`입니다. Isaac을 실행할
필요 없이 현재 rev 2.0.1 URDF, 관절 한계, HOME으로 계산합니다. 로봇 명령이나
기존 녹화 데이터는 변경하지 않습니다.

```bash
cd ~/stkim_ws/quest_robot_isaac
~/stkim_ws/.venv/bin/python scripts/visualize_arm_workspace.py
```

생성된 `outputs/arm_workspace/workspace.html`을 브라우저에서 여세요. 마우스로
회전·확대하고, 위 메뉴로 자유 방향 / 수직 고정 / 두 모드 비교를 선택합니다.
범례를 클릭하면 좌우 팔 점구름을 개별 표시·숨김할 수 있습니다. HTML에
Plotly를 포함하므로 인터넷 없이 열 수 있습니다. HOME의 로봇 외형은 실제
URDF 링크별 convex hull로 간소화한 표시이고, 아래 점구름 계산에는 사용하지 않습니다.

- **주황색:** 왼팔, **파란색:** 오른팔.
- **자유 방향:** 팔마다 관절 한계 내 4,096개 자세를 Sobol 표본 추출한 FK 위치.
- **수직 고정:** 현재 teleop과 같은 수직 방향·고정 yaw에서 scikit-robot batch IK로
  검사한 7.5cm 간격의 3D 격자. 목표마다 HOME에서 시작하고 랜덤 재시도를 포함한
  총 4회, 최대 200회 반복으로 탐색합니다. 좌우 팔은 독립 계산합니다.
- **평면도:** 손목 높이 `Z=0.54m`에서 2.5cm 격자를 검사한 결과. 테이블, 중앙
  상자의 외곽과 빨간 큐브 생성 띠를 겹칩니다. 그림의 생성 띠는 중심 구역을
  단순 표시한 것이며, 실제 생성기의 회전 큐브 모서리 여유 검사는 별도입니다.

점의 기준은 **손목 `left/right_dof7_link`의 원점**입니다. 손가락 끝이 아닙니다.
수직 그리퍼 끝은 약 23.5cm 아래이므로, 손목 높이 54cm일 때 끝 높이는 약
30.5cm입니다. 현재 테이블 상판 높이는 30cm입니다.

**이 결과는 표본 기반 기구학 범위입니다.** 충돌·자기 충돌·양팔 간 간섭·이동
경로·실제 모터 추종은 검사하지 않습니다. 회색 X는 지정한 계산량에서 IK 해를
찾지 못한 곳이며, 불가능하다는 증명은 아닙니다. 현재 teleop의 Pinocchio
100회 solver와 scikit-robot의 solver는 다르므로, 그림의 성공점이 teleop에서
항상 성공한다는 의미도 아닙니다. 저장한 성공 해는 FK, 관절 한계, 위치 1mm와
회전 0.002rad 이내 조건으로 다시 검사합니다.

더 촘촘히 계산하거나 평면도의 손목 높이를 바꿀 수 있습니다.

```bash
~/stkim_ws/.venv/bin/python scripts/visualize_arm_workspace.py \
  --samples 16384 --grid-step 0.05 --attempts 8 --slice-height 0.57
```

실제 STL 외형의 로봇과 수직 가동범위 점구름을 scikit-robot의 별도 창으로
보려면, 데스크톱 디스플레이가 있는 터미널에서 다음을 실행합니다. 창을 닫으면
종료합니다. HTML과 PNG는 이 창을 사용하지 않아도 생성됩니다.

```bash
~/stkim_ws/.venv/bin/python scripts/visualize_arm_workspace.py --viewer
```

출력:

| 파일 | 내용 |
|---|---|
| `workspace.html` | 회전·확대 가능한 3D 비교 |
| `workspace.png` | 전체 범위·수직 범위·테이블 높이 평면도 |
| `samples.npz` | 자유 FK의 XYZ/관절 값, 수직·평면 격자와 IK 성공 여부/7개 관절 값 |
| `summary.json` | 설정, URDF 경로, 관절 한계, 표본 수, 검증 오차 |
| `resolved_robot.urdf` | mesh 경로를 절대경로로 바꾼 시각화용 사본 |
| `validation.json` | 현재 결과의 FK를 Pinocchio와 독립 비교한 실행 기록 |

초기 결과는 자유 FK 8,192개, 수직·평면 IK 목표 2,646개를 검사했습니다.
각 팔의 자유·수직 결과에서 고른 총 128개 자세를 Pinocchio와 비교해 같은
URDF 기구학임을 확인했습니다. `validation.json`은 이 검증 실행의 기록이며,
다른 옵션으로 다시 계산한 결과에 대한 자동 검증 보고서는 아닙니다.

API 근거: [공식 scikit-robot FK/IK 사용법](https://scikit-robot.readthedocs.io/en/latest/reference/robot_model_tips.html),
[URDF 로딩·viewer 예제](https://scikit-robot.readthedocs.io/en/latest/examples/index.html).

## 실제 큐브 생성 영역

HTML의 **Verified spawn locations** 메뉴와 초록색 점은 실제 생성에 사용하는 큐브 중심입니다. 주황/파랑 점의 손목 기준과 다릅니다. `ik_spawn_pool.json`은 실제 Pinocchio 100회 IK로 연속 접근·집기·들기·상자에 놓기 경로를 검사한 260개 위치를 저장합니다. 일반 scikit-robot 점구름의 성공 여부로 직접 스폰하지 않습니다. `ik_spawn_coverage.png`는 정확한 생성 위치와 바디 카메라 투영을 보여줍니다. 초기 팔의 가림은 제외 조건이 아닙니다. 설정을 바꾸면 Isaac 시작 전에 후보를 재계산합니다. [설정과 제한](TABLETOP_IK_SPAWN.md)을 참고하세요.
