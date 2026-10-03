# Teleoperation 연속 관절 이동 검증

검증일: 2026-10-03 KST. env_isaaclab, 실제 Isaac GPU와 Quest WSS 서버를 사용했습니다. 물리 Quest 입력은 합성 이벤트로 대체했습니다.

## 적용된 동작

IK 출력은 도달할 목적지입니다. Isaac의 `JointMotionLimiter`가 측정된 초기 관절 자세에서 시작해, 매 physics step마다 연속 drive reference를 생성합니다. 새로운 IK 해가 나오거나 IK가 실패했다가 다시 성공해도 경로의 위치·속도를 초기화하지 않습니다. 운용 중 `write_joint_state_to_sim`으로 IK 자세를 쓰지 않습니다.

- 목표 관절 속도: 0.25rad/s, 약 14.3°/s.
- 목표 관절 가속도: 0.50rad/s², 약 28.6°/s².
- IK 실패/재보정: `F16H`로 팔을 감속 정지. 그리퍼 명령은 독립적으로 적용.
- 통신 timeout: 오래된 IK 목적지를 끝까지 따라가지 않고 감속 정지.
- 새 해/재연결: 현재 경로 상태에서 연속 이동. 보정 anchor와 IK seed는 실제 측정된 관절값.
- drive가 따라가지 못해 reference와 측정값의 차이가 0.05rad를 넘으면 해당 reference를 감속.
- `--no-filter`는 입력 low-pass만 끄고, 이동 제한은 유지.

설정은 [teleop_config.py](config/teleop_config.py)의 `ARM_MAX_VELOCITY`, `ARM_MAX_ACCELERATION`, `ARM_MAX_TRACKING_ERROR`입니다. 양팔 14개 관절에 적용합니다. 속도·가속도는 관절 drive reference 기준이며, 외력/충돌에 대한 실제 관절 속도 보장을 의미하지는 않습니다.

## 검증 결과

| 검사 | 결과 |
|---|---|
| 수학·UDP·IK·tabletop regression | `python -m pytest -q`: 23 passed |
| 실제 GPU replay | 900 physics steps 통과 |
| reference 최대 속도 | 0.250002rad/s (float32 오차 포함) |
| reference 최대 가속도 | 0.500178rad/s² (float32 오차 포함) |
| 실측 위치 차분의 최대 속도 | 0.250169rad/s |
| IK 실패 후 재개 첫 reference step | 0.000138879rad, 약 0.008° |
| 운용 중 joint state 직접 쓰기 | 0회; 초기 HOME에만 1회 |
| 큰 목적지 이동 도중 실패 | 이전 목적지에 도달하기 전에 감속 정지 |
| 통신 중단 | watchdog 이후 감속 정지 |
| HOME 복귀 최대 관절 오차 | 0.000362rad |
| 실제 WSS → IK → 제한된 drive → 실측 자세 | 통과 |
| WSS 도달 불가능 목표 → 새 도달 가능 목표 | 정지/느린 재개 통과; 첫 150ms 관절 변화 최대 0.002394rad |
| sender 재시작·Isaac 재시작 | 현재 자세 보정/HOME 재보정 통과 |

GPU replay는 손을 들어 테이블/상자 접촉을 피한 목표로 이동 연속성을 확인했습니다. 모션 제한은 충돌 회피 planner가 아니며, 중간 관절 경로가 상자 벽과 충돌하면 접촉 힘/solver 보정으로 별도의 빠른 움직임이 발생할 수 있습니다. 검사 중 이런 접촉 반응도 관측했으므로 실제 집기 경로의 충돌 검사는 별도로 필요합니다.

기존 USD의 강한 drive에 낮은 PhysX velocity hard limit을 추가하면 정지 HOME에서도 solver가 불안정해졌습니다. 그 설정은 제거했으며 원래 arm gain/physics limit을 사용합니다. 느린 이동은 수신기의 연속 drive reference로 제어합니다. GPU 결과의 `max_measured_velocity_rad_s`는 physics-step 실측 위치 차분이고, 별도로 기록한 순간 solver velocity와 구분합니다.

## 산출물과 재현

- [목표·연속 reference·실측 위치 그래프](outputs/joint_motion_check/trajectory.png)
- [GPU 결과](outputs/joint_motion_check/validation.json), [매 step 기록](outputs/joint_motion_check/samples.json)
- [WSS pipeline 결과](outputs/pipeline_check/validation.json)

```bash
conda activate env_isaaclab
cd ~/stkim_ws/quest_robot_isaac
python -m pytest -q
python scripts/test_joint_motion.py --headless --device cuda:0
python scripts/test_pipeline.py
```

변경된 hold packet과 측정 feedback 동작을 함께 적용하려면 Isaac/Quest 프로세스를 모두 재시작합니다.
