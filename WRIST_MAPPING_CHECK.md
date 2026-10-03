# 손목 기준 회전 매핑

2026-10-03 KST. 목표는 사용자가 손목을 비틀거나 꺾을 때 그리퍼도 같은
손목 동작을 하는 것입니다. 몸통 기준 회전축을 복사하지 않습니다.

## 적용 방식

보정 시 컨트롤러와 실제 로봇 손목 자세를 각각 중립으로 저장합니다.
컨트롤러 로컬 상대 회전을 그리퍼의 물리적인 손목 축으로 바꿉니다.

```python
relative = controller_neutral.rotation.T @ controller_now.rotation
target.rotation = robot_anchor.rotation @ B @ relative @ B.T
```

`B`의 열은 컨트롤러 X/Y/Z에 대응하는 손목 링크의 축입니다.

| 컨트롤러 동작 | 컨트롤러 로컬 축 | 왼손 링크 축 | 오른손 링크 축 |
|---|---|---|---|
| 비틀기 | X | -Y (손가락이 뻗는 방향) | -Y |
| 위아래 꺾기 | Y | -Z (손목의 왼쪽 방향) | +Z |
| 좌우 꺾기 | Z | +X (손목의 위 방향) | -X |

실험에서 주로 움직인 컨트롤러 로컬 축과 URDF 손가락 방향/좌우 거울
구조를 기준으로 정했습니다. `LEFT_CONTROLLER_TO_EE_ROT`와
`RIGHT_CONTROLLER_TO_EE_ROT`를 config에 지정하고 송신기가 팔 이름을
전달합니다. generic mapper의 기존 identity 기본값은 유지하지만 실제
teleoperation에서는 양손별 매핑을 사용합니다.

회전각 gain은 1이고, 복합 회전도 SO(3)로 유지합니다. HOME과 위치 이동
매핑은 변경하지 않았습니다. 현재 측정 자세에서 재보정하므로 중립의
회전 목표가 기존 자세와 같습니다. 속도/가속도 제한, 실패 시 감속 정지,
gripper trigger 동작도 유지합니다.

## 검증

- 전체 CPU 회귀 검사: 40 passed. 서로 다른 controller/robot 초기 방향,
  양손 3축의 양/음 회전, 복합 회전, 필터 수렴, 기존 이동 제한을 확인했습니다.
- 기존 실제 Quest 여섯 동작의 회전 기록 2,159개를 새 매핑으로 재생했습니다.
  각 시험 시작을 중립으로 다시 설정한 **회전 목표만의 검사**입니다.
  그리퍼 손목 좌표계로 환산한 목표와 입력의 축/부호/각도 차이는 최대
  1.6e-11도였고, 중립에서 회전 점프가 없었습니다.
  [결과](outputs/wrist_axis_check/20261003_153742/mapping_replay.json).
- 현재 HOME에서 양팔의 세 손목 동작을 각각 ±10도로 목표화한 12개 IK
  검사가 현재 30 iteration 설정에서 모두 성공했습니다. 충돌 검사는 아닙니다.
- 실제 Isaac + Quest WSS 서버에 합성 입력을 넣는 전체 검증이 통과했습니다.
  [pipeline 결과](outputs/pipeline_check/validation.json)에 저장했습니다.
  양팔 안정 회전 오차는 최대 0.0133도, 위치 오차는 최대 0.065 mm였습니다.
  IK 실패 후 감속 정지·복구, tracking loss, 송신기/Isaac 재시작도 통과했습니다.
  기대 회전도 새 양손 매핑을 반영했습니다. 새 HOME의 중력에 따른
  정지 오차가 약 1 mrad이므로 명목 HOME 비교 허용값은 2 mrad로 설정하고,
  재보정 후 중립 유지 시 실제 자세 변화는 별도로 0.3 mrad 이내인지 확인합니다.

수학적 목표 대응과 실제 헤드셋 조작 감각은 다릅니다. 수정 후 실제 Quest의
10~15도 작은 동작으로 다시 확인해야 합니다. 큰 회전의 IK 도달 가능성이나
충돌 회피를 보장하지 않습니다. 기존 `head_yaw` 입력 기준도 유지되므로
비교 실험에서는 머리를 고정합니다.

## 실행

Quest 송신기를 재시작하고 손을 편한 중립으로 유지해 보정합니다.
Isaac에 적용되는 관절 이동 제한은 계속 유지됩니다.

```bash
python scripts/run_quest_teleop.py --wrist-axis-test --no-filter
```

진단의 `HAND[twist,bend,sideways]deg`에서 controller와 target을 비교합니다.
실제 measured는 감속 추종으로 늦을 수 있어 끝 자세를 잠시 유지해야 합니다.
WORLD 회전벡터는 서로 다른 초기 손목 방향 때문에 일치할 필요가 없습니다.
