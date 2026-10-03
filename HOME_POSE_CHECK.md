# 변경된 HOME 자세 진단

> 이동 제어 업데이트: 현재 teleoperation은 IK 해를 목적지로만 사용하며, 관절 속도·가속도를 제한해 현재 자세에서 연속 이동합니다. 실패 시 감속 정지하고 다시 풀리면 그 위치에서 천천히 재개합니다. 아래 작업공간 진단은 IK 도달 범위에 관한 결과입니다. 최신 이동 동작과 검증은 [MOTION_CONTROL_CHECK.md](MOTION_CONTROL_CHECK.md)를 참고하세요.

> 모델 변경 안내: 아래 진단은 rev 2.0.0 URDF 기준입니다. 현재 실행 모델은 `assets/F14_URDF_rev_2_0_1 2/urdf/FlaminGO_14Dof_Arm_Robot_v2.urdf`이며, 오른팔 dof3/dof4의 관절 축과 HOME 부호가 반대입니다. 수정된 HOME은 같은 실제 시작 자세를 나타냅니다. 기존 USD와의 부호 차이는 명령·측정 state·관절 한계에 반영합니다. 변경 후 검증 자료는 `outputs/urdf_revision_check/`에 저장합니다.

모델 변경 후 regression **15 passed**, 양팔 +5cm X와 회전 IK 통과. 실제 Isaac에서 HOME·전방 4cm·위쪽 2cm·회전 5°·HOME 복귀를 확인했으며, 이전 HOME 실측 위치와 차이는 양팔 모두 0mm였습니다. 안정 상태 EE 오차는 최대 0.211mm, 변환된 USD 관절 한계는 새 URDF와 일치했습니다. [새 모델 GPU 결과](outputs/urdf_revision_check/validation.json), [HOME view](outputs/urdf_revision_check/quest_composite_home.png).

검사일: 2026-10-03 KST. 현재 `robot/f14_config.py` HOME, 실제 env_isaaclab, Pinocchio URDF와 Isaac GPU/USD를 사용했습니다. 물리 Quest는 연결하지 않았습니다.

## 결과

현재 HOME은 모든 관절 한계 안에 있지만, 양쪽 dof6가 ±45.264°로 ±50° 한계에 가깝습니다. 현재 손목 방향을 유지하는 6DoF IK에서 일부 방향의 이동 여유가 작고, 관절 한계에 닿으면 clipping 후 수렴이 느려집니다. 한 팔만 실패해도 송신기는 양팔의 마지막 유효 목표를 유지합니다. 이 동작이 추종 중 정지하는 느낌의 유력한 소프트웨어 원인입니다. 실제 controller 입력/회전축/무선 지연은 이번 검사로 확정할 수 없습니다.

## 초기 자세와 관절 한계

- 기존 EE 위치: left (0.42102, 0.26729, 0.53370), right (0.42102, -0.26728, 0.53370) m.
- 현재 EE 위치: left (0.28349, 0.27183, 0.49672), right (0.28349, -0.27183, 0.49672) m.
- 전방 위치가 약 13.75cm 줄고 높이가 약 3.70cm 내려갔습니다.
- 초기 Jacobian 최소 특이값은 기존 0.0807 → 현재 0.1097입니다. HOME 자체의 Jacobian이 퇴화한 것은 아닙니다. 길이/각도를 혼합한 Jacobian 수치는 상대 비교에만 사용했습니다.

| 관절 | 왼쪽 한계 / HOME (°) | 오른쪽 한계 / HOME (°) | HOME에서 가까운 한계까지 (°) |
|---|---|---|---|
| dof1 | -195.00 … 85.00 / 0.00 | -195.00 … 85.00 / 0.00 | 85.00 |
| dof2 | -17.00 … 170.00 / 45.84 | -170.00 … 17.00 / -45.84 | 62.84 |
| dof3 | -144.53 … 144.53 / 22.92 | -144.53 … 144.53 / 22.92 | 121.61 |
| dof4 | -103.00 … 12.00 / -87.66 | -12.00 … 103.00 / 87.66 | 15.34 |
| dof5 | -40.00 … 220.00 / 13.75 | -220.00 … 40.00 / -13.75 | 53.75 |
| dof6 | -50.00 … 50.00 / -45.26 | -50.00 … 50.00 / 45.26 | 4.74 |
| dof7 | -90.00 … 90.00 / -15.47 | -90.00 … 90.00 / -15.47 | 74.53 |

실제 USD와 URDF 관절 한계의 최대 차이는 3.56e-07 rad로 일치합니다.

## 현재 solver로 천천히 움직일 때의 범위

HOME에서 시작해 한 팔씩, 자세를 고정한 채 2mm 간격으로 이동했습니다. 현재 설정(max_iter=30, eps=2e-4, dt=.3, damping=1e-4)에서 매 단계 성공한 마지막 위치입니다. 다른 팔은 HOME에 고정했습니다. 첫 실패에서 중단한 경로별 추종 범위이며, 로봇의 절대 작업공간이나 충돌 없는 범위를 뜻하지 않습니다.

| 방향 | 기존 HOME | 현재 HOME |
|---|---|---|
| 뒤쪽 -X | 17.0cm | 8.0cm |
| 앞쪽 +X | 6.4cm | 6.2cm |
| 아래 -Z | 10.2cm | 18.0cm |
| 위 +Z | 3.6cm | 3.6cm |
| 좌우 Y | 안쪽 ≥20cm / 바깥쪽 14cm | 양방향 ≥20cm (검사 상한) |

좌우 팔은 대칭입니다. 뒤쪽 첫 실패는 dof4 한계에 걸렸습니다. 전방/위쪽 첫 실패는 dof6 한계에 걸렸고, 200 iteration으로 늘리면 그 첫 실패 목표는 수렴했습니다. 그러나 더 큰 이동은 여전히 실패할 수 있습니다. HOME에서 한 번에 +5cm Z를 주면 30 iteration은 실패, 200 iteration은 성공했습니다. +10cm Z는 200 iteration에서도 실패했습니다. 실패 자체가 목표의 물리적 도달 불가능성을 증명하지는 않습니다.

현재 HOME 위치를 고정한 local EE 회전 sweep에서는 -X 약 6.5°, left +Y/right -Y 약 10°에서 첫 실패가 나며 dof6 한계에 닿습니다. 나머지 검사 방향은 60°까지 성공했습니다. 이 회전축은 로봇 EE local 축이고, 실제 controller 축과의 체감 대응은 실기 확인이 필요합니다.

## 실제 카메라 view

- [현재 HOME composite](outputs/home_pose_check/quest_composite.png)
- [기존 HOME composite](outputs/camera_check/quest_composite.png)
- [현재 BODY](outputs/home_pose_check/front.png), [LEFT WRIST](outputs/home_pose_check/left_wrist.png), [RIGHT WRIST](outputs/home_pose_check/right_wrist.png)
- [실제 camera/EE world pose](outputs/home_pose_check/camera_poses.json)

BODY는 base_link에 고정되어 여전히 아래 30°를 봅니다. 새 HOME에서는 손이 몸 가까이 내려와 그리퍼 대부분이 화면 하단/좌우 바깥에 걸립니다. EE 원점의 BODY 투영 위치는 대략 left (-50, 518), right (690, 518) pixel로 640×480 화면 밖입니다. 손가락 일부는 원점보다 앞쪽에 있어 하단에 보입니다.

손목 카메라의 world 하향각은 기존 약 39° → 현재 약 59.4°입니다. 실제 링크에 대한 장착 offset은 1mm 이내로 검증됐지만, 새 HOME의 손목 방향 때문에 기존 앞쪽 확인용 큐브는 손목 영상에 보이지 않습니다. 바닥과 그리퍼 기구가 주로 보입니다.

BODY는 `config/teleop_config.py`의 BODY mount와 `simulation/cameras.py`의 focal_length(현재 모든 센서 18mm)를 조정할 수 있습니다. 현재 HOME 양손을 포함하려면 BODY 시야각을 넓히는 쪽이 적합합니다. 예를 들어 BODY만 focal_length 14mm로 줄이면 현재 pinhole 투영 계산상 양쪽 EE 원점이 화면 안에 들어옵니다. 실제 렌더링으로 해당 카메라 후보를 검증한 것은 아닙니다. 손목 카메라 mount 회전은 새 HOME에서 보는 작업 위치에 맞춰 별도 조정할 수 있습니다.

## 실제 Isaac drive 추종

합성 IK 목표를 격리 UDP 포트 15016으로 보내고 각 목표를 1.2초 유지한 뒤 측정했습니다. controller/filter 경로를 생략해 drive 자체를 확인했습니다. HOME → 양팔 전방 4cm → 위쪽 2cm → local X 회전 5° → HOME 복귀 순서입니다.

| 목표 | 최대 관절 목표 오차 (rad) | 최대 EE 위치 오차 (mm) | 최대 EE 회전 오차 (°) |
|---|---|---|---|
| home | 0.000362 | 0.141 | 0.037 |
| forward_4cm | 0.000375 | 0.167 | 0.034 |
| up_2cm | 0.000432 | 0.211 | 0.044 |
| local_x_5deg | 0.000374 | 0.137 | 0.033 |
| return_home | 0.000362 | 0.141 | 0.037 |

카메라 약 27–28fps, command reject 0. 이 검사는 안정된 상태의 오차이며 headset 체감 지연이나 transient drive 응답의 측정값은 아닙니다. `POSITION_FILTER_ALPHA=ROTATION_FILTER_ALPHA=.2`는 30Hz에서 계단 입력의 95%에 도달하는 데 약 0.45초 걸립니다. 진단 시 `--verbose --no-filter`로 필터 지연과 `IK FAIL`을 구분할 수 있습니다. 머리를 움직이면 현재 head_yaw reference에서 controller 상대 좌표도 달라집니다.

## HOME 조정 후보

어깨/팔꿈치를 그대로 두고 **LEFT_HOME[5]=-0.50, RIGHT_HOME[5]=+0.50 rad**로 비교 계산했습니다. dof6 한계 여유가 4.74° → 21.35°가 되며, 같은 solver/sweep 조건에서 +X 추종 6.2 → 17.8cm, +Z 3.6 → 8.2cm로 늘었습니다. EE 위치는 몇 mm 정도 바뀌고 방향은 바뀝니다. 이 후보의 실제 GPU 카메라 view, 충돌, Quest 체감은 아직 검증하지 않았습니다. 뒤쪽 이동은 dof4 한계 때문에 여전히 약 8cm였습니다.

## 상세 수치와 재현

- [IK sweep 원본](outputs/home_pose_check/ik_workspace.json), [진단 스크립트](outputs/home_pose_check/analyze.py)
- [손목 HOME 후보 비교](outputs/home_pose_check/wrist_candidates.json)
- [실제 drive 측정](outputs/home_pose_check/drive_tracking.json), [요약](outputs/home_pose_check/drive_summary.json), [측정 스크립트](outputs/home_pose_check/check_drives.py)
- [카메라 로그](outputs/home_pose_check/isaac.log), [drive 로그](outputs/home_pose_check/drive_tracking.log)

```bash
cd ~/stkim_ws/quest_robot_isaac
conda activate env_isaaclab
PYTHONPATH=. python outputs/home_pose_check/analyze.py
python outputs/home_pose_check/check_drives.py
python scripts/run_quest_teleop.py --verbose --no-filter --host-ip <COMPANY_PC_IP>
```

기존 `test_cameras.py --smoke-test`와 core regression의 HOME FK 기준에는 이전 HOME 위치가 하드코딩되어 있습니다. 현재 HOME에 그대로 실행하면 자세 변경 자체로 실패합니다. 이번 GPU 검사는 그 오래된 기준을 사용하는 smoke-test 대신 bounded 일반 실행과 별도 실제 state 측정을 사용했습니다. 이전 `VALIDATION_TELEOP.md`의 수치는 이전 HOME에 대한 결과입니다.
