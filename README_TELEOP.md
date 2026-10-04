# F14 / FlaminGO — Quest 3 × Isaac Sim teleoperation

두 개의 터미널에서 실행하는 **XYZ 이동 + 그리퍼 개폐** 양팔 teleoperation입니다. 양쪽 그리퍼 방향은 항상 수직 아래로 고정해 IK를 계산하며, 컨트롤러 회전은 조작에 반영하지 않습니다. 왼팔 7 + 오른팔 7 + 좌우 gripper 2의 **16D semantic action**과 실제 VLA RGB 카메라 3개를 사용합니다. HOME과 고정 EE 방향은 `robot/f14_config.py`에서 설정합니다.

수직 그리퍼의 손가락 끝은 손목 링크보다 약 23.5cm 아래입니다. 기존 HOME 높이는 테이블과 충돌하므로, 새 손목 HOME은 **왼쪽 `(0.40, 0.18, 0.57)`m, 오른쪽 `(0.40, -0.18, 0.57)`m**로 설정했습니다. 기존 위치보다 약 10cm 앞·10cm 위·각각 5cm 바깥쪽이며, 손가락 끝은 현재 테이블 위 약 3.5cm에 놓입니다. 고정 방향을 유지할 수 있는 관절 한계 안의 IK 자세입니다. 변경 후에는 **Isaac과 Quest 송신기를 모두 재시작**하세요.

현재 IK URDF는 `assets/F14_URDF_rev_2_0_1 2/urdf/FlaminGO_14Dof_Arm_Robot_v2.urdf`입니다. action과 HOME은 이 모델의 관절 각도를 사용합니다. 기존 Isaac USD와 축이 반대인 오른팔 dof3/dof4는 시뮬레이터 경계에서 부호를 변환하며, 측정 state와 수신 관절 한계도 같은 기준으로 변환합니다. 따라서 두 관절 HOME의 부호를 바꿔도 실제 시작 자세는 유지됩니다.

## 실행

Terminal 1 — Isaac:

```bash
conda activate env_isaaclab
cd ~/IsaacLab_2_3_2/IsaacLab
./isaaclab.sh -p ~/stkim_ws/quest_robot_isaac/scripts/run_isaac_teleop.py --device cuda:0
```

GUI가 필요 없으면 `--headless`를 추가합니다. 카메라 렌더링은 launcher가 `enable_cameras=True`로 자동 활성화합니다.

Terminal 2 — Quest / IK:

```bash
conda activate env_isaaclab
cd ~/stkim_ws/quest_robot_isaac
python scripts/run_quest_teleop.py --host-ip <COMPANY_PC_IP>
```

IP를 생략하면 현재 라우팅 정보를 통해 탐색합니다. 여러 NIC가 있으면 `ip -4 addr`로 Quest와 같은 LAN의 주소를 확인하고 명시하세요. 서버는 `0.0.0.0:8012`에 bind하며 실행 시 아래 URL을 출력합니다.

```text
https://<COMPANY_PC_IP>:8012/?ws=wss://<COMPANY_PC_IP>:8012&grid=False
```

설치된 Vuer 0.0.60은 `/`에서 client와 WebSocket을 함께 제공합니다. 로컬 client 대신 호스팅 client를 사용하는 대체 주소도 출력합니다.

```text
https://vuer.ai?ws=wss://<COMPANY_PC_IP>:8012&grid=False
```

Quest Browser에서 주소를 열고 인증서 신뢰를 처리한 뒤 VR/XR 세션에 들어가세요. **controller mode**입니다. 초기 2초 안정화 후 서로 다른 이벤트에서 45개 pose sample을 모읍니다. 머리와 양손을 가만히 유지하고 trigger를 놓으세요. 움직임이 크면 평균을 버리고 다시 수집합니다.

두 프로세스의 실행 순서는 자유입니다. 둘 다 준비되기 전에는 arm HOME / gripper open을 유지합니다. Quest terminal에서 **`R` + Enter**를 입력하면 팔을 감속 정지하고 현재 측정된 손목 XYZ를 기준으로 재보정합니다. EE 방향은 재보정 후에도 수직 아래로 유지합니다. 재접속과 프로세스 재시작도 자동 재보정합니다. 양쪽 모두 Ctrl+C로 종료합니다.

## 확인한 환경과 의존성

### 컨트롤러 회전 무시 확인

Isaac을 평소처럼 실행하고, Quest 송신기를 아래 옵션으로 실행합니다.
컨트롤러 회전을 관찰하는 진단이며, 회전으로 로봇 방향을 바꾸지는 않습니다.
속도·가속도 제한은 유지합니다. `--no-filter`는 입력 필터의 지연만 제거합니다.

```bash
python scripts/run_quest_teleop.py --wrist-axis-test --no-filter
```

Quest 연결 및 보정 후 터미널 안내에 따라 왼손 비틀기 → 위아래 꺾기 → 좌우
꺾기, 이어서 오른손 세 동작을 시험합니다. 각 시험은 손과 로봇이 중립에서
멈춘 뒤 `N` + Enter로 시작하며 12초간 기록합니다. 머리와 반대 손을 고정하고
한 방향으로 10~15도 천천히 회전 → 1~2초 유지 → 중립 → 반대 방향 → 유지 →
중립으로 돌아옵니다. 처음 움직인 물리적 방향을 메모하세요. 다음 시험도
`N` + Enter로 시작합니다. 추적 끊김·재보정으로 중단된 시험은 재시도합니다.

`WORLD[X,Y,Z]deg`의 controller / target / measured는 각 시험 시작 대비 회전
벡터입니다. 공통 축은 X 앞, Y 왼쪽, Z 위이고 부호는 오른손 법칙입니다.
컨트롤러 XYZ를 고정한 채 회전하면 controller 벡터만 변하고 target 벡터는
0으로 유지되어야 합니다. 실제 measured에는 관절 추종 오차가 있을 수 있습니다.
로컬 링크 축 벡터도 JSON에 저장합니다. measured는 Isaac 관절 feedback의 URDF FK이며 최대 60Hz로
갱신됩니다. 손이 먼저 움직인 순간의 지연과 축 불일치를 혼동하지 말고 끝에서
잠시 유지한 기록도 비교하세요. `IK HOLD`이면 목표와 실제가 달라지는 것이
회전 입력의 영향만을 뜻하지 않습니다.

결과는 `outputs/wrist_axis_check/<시각>/summary.json`과 시험별 JSON에
저장됩니다. 자동으로 축이나 부호를 수정하지 않습니다. 실제 Quest의 여섯
동작을 수행해야 실기 결과가 생깁니다.

| 항목 | 로컬에서 확인한 값 |
|---|---|
| Isaac Sim | 5.1.0.0 |
| Isaac Lab checkout | `git describe`: v2.3.2, `VERSION`: 2.3.2 |
| Python | env_isaaclab / 3.11 |
| TeleVuer | `~/stkim_ws/xr_teleoperate/teleop/televuer` editable source |
| vuer / params-proto | 0.0.60 / 2.13.2 |
| Pinocchio (`pin`) | 2.7.0 |
| NumPy / OpenCV | 1.26.4 / 4.11.0 |
| pyzmq | 26.2.1 — 누락되어 이 패키지만 `--no-deps`로 추가 |

기존 environment를 upgrade하지 않습니다. 새로 복제한 환경에서 ZMQ만 빠졌다면:

```bash
python -m pip install --no-deps pyzmq==26.2.1
```

`requirements-teleop.txt`는 관련 버전을 기록합니다. 기존 env에 일괄 upgrade하는 용도가 아닙니다. CameraCfg/Articulation 코드는 **실제 설치된 v2.3.2 source**를 확인했습니다. [해당 버전 Camera API 문서](https://isaac-sim.github.io/IsaacLab/v2.3.2/source/api/lab/isaaclab.sensors.html#isaaclab.sensors.CameraCfg)도 참고할 수 있습니다.

## 프로세스 / 파일 구조

```text
Quest Browser controllers --HTTPS/WSS :8012--> Quest process
    TeleVuer wrapper → calibration → relative SE(3) → Pinocchio IK
                                      │
                              UDP 127.0.0.1:5005 (16D)
                                      ↓
                                  Isaac process
                          F14 articulation + 3 RGB cameras
                                      │
                raw RGB ──────────────┼── future dataset callback
                                      ↓
                           operator-only RGB composite
                           JPEG worker, latest-frame PUB
                                      │
                             ZMQ 127.0.0.1:5556
                                      ↓
                         Quest SUB → BGR → render_to_xr
```

TeleVuer는 내부적으로 WebSocket child process와 shared-memory image writer를 사용합니다. Isaac와 Vuer를 같은 Python process에서 실행했을 때 연결이 실패했던 기존 실험을 반영해 두 runtime의 import와 launch를 분리했습니다. Isaac 프로세스에는 TeleVuer/Vuer/Pinocchio import가 없습니다. Quest 프로세스에는 Isaac import가 없습니다. **Vuer가 fork한 다음** ZMQ를 시작하므로 이미 실행 중인 ZMQ thread를 fork하지도 않습니다.

| 파일 | 역할 |
|---|---|
| `config/teleop_config.py` | rate, filter, timeout, camera offset/gain |
| `robot/f14_config.py` | HOME, joint 이름, 원본 asset 경로 |
| `robot/f14_ik.py` | 기존 LOCAL DLS 알고리즘, arm-only Jacobian |
| `robot/joint_motion.py` | IK 목적지까지 관절 속도·가속도를 제한하는 연속 drive reference |
| `robot/f14_ik_legacy.py` | 수정 전 solver 백업 |
| `robot/gripper.py` | trigger 정규화, scalar↔finger mapping |
| `teleop/quest_teleop_server.py` | Quest main loop와 calibration UX |
| `teleop/televuer_adapter.py` | 실제 설치 API, timestamp watchdog, SSL/URL |
| `teleop/xr_pose.py` | SE(3) calibration, 1:1 mapping, SO(3) filter |
| `teleop/action_protocol.py` | 공통 16D UDP packing/validation, 상태 조회 |
| `teleop/xr_video.py` | JPEG/ZMQ worker와 XR BGR adapter |
| `simulation/f14_scene.py` | USD spawn, tabletop scene, name lookup, HOME, measured state |
| `simulation/tabletop.py`, `config/tabletop_config.py` | 열린 상자, 물리 큐브, 랜덤 배치와 생성 금지 구역 |
| `simulation/cameras.py` | 실제 센서 3개, raw observation, snapshot |
| `simulation/quest_view_compositor.py` | BODY 중앙 크게, 좌우 WRIST 작게 |
| `simulation/isaac_teleop_app.py` | physics/camera loop, recorder callback |
| `scripts/run_*_teleop.py` | 두 runtime의 실행 진입점 |
| `scripts/test_*.py`, `tests/` | 독립 진단과 regression tests |

기존 `teleop/quest_ik_sender.py`의 사용자 수정, `scripts/isaac_joint_receiver.py`, `scripts/teleop_quest_isaac.py`, `scripts/test_f14.py`는 그대로 보존했습니다. **새 시스템은 `run_*_teleop.py`로 실행**하세요. 기존 sender/receiver에는 14D packet과 magic 불일치가 있으므로 새 실행 경로와 섞지 않습니다. 기존 단일 프로세스 스크립트도 reference용으로만 남겼습니다. MuJoCo 프로젝트와 원본 USD/URDF는 수정하지 않았습니다.

## 16D action과 수신 안전 동작

```python
action_16 = [*left_arm_q_7, *right_arm_q_7, left_closure, right_closure]
# 0:7 left radians / 7:14 right radians / 14:16 [0=open, 1=closed]
```

UDP `!4sId16f`, 정확히 **80 bytes**: `F16A`, uint32 sequence, float64 timestamp, float32 16개. `F16A`의 팔 각도는 검증된 다음 IK 목적지이며, PhysX에 바로 적용하지 않습니다. 같은 크기의 `F16H` packet은 IK 실패/보정 중 팔 감속 정지를 요청하고 gripper 입력은 독립적으로 적용합니다. timestamp는 동일 Linux host의 `time.monotonic()` 초입니다. Wall-clock 조정 영향을 받지 않으며, **다른 PC로 action UDP를 옮길 때는 clock/schema 재설계가 필요**합니다. 외부 LAN에 action/video 포트를 열지 않습니다.

receiver는 길이, magic, finite 값, gripper 범위, USD arm limits, 시간 순서와 0.5초 freshness를 검사합니다. 잘못된 packet은 watchdog을 갱신하지 않습니다. 수신 queue를 비워 가장 최신 유효 packet만 적용합니다. sequence가 0으로 돌아가는 sender 재시작도 monotonic timestamp로 처리합니다.

- `WAITING`: 아직 유효 명령 없음 → HOME / open.
- `ACTIVE` + `F16A`: 유효한 최근 IK 목적지까지 속도·가속도를 제한하며 이동.
- `ACTIVE` + `F16H`: 팔을 감속 정지하고 정지한 drive reference 유지. gripper는 계속 사용 가능.
- `HOLD`: 마지막 명령이 0.5초보다 오래됨 → 오래된 목적지를 계속 쫓지 않고 감속 정지.

읽기 전용 `F16?` 요청에 수신기가 `F16S + boot monotonic time + measured state`를 돌려줍니다. 아직 도달하지 않은 IK 목적지가 아니라 실제 측정된 관절/그리퍼 자세입니다. IK 초기값과 재보정 anchor도 이 측정값을 사용합니다. 송신기 재시작은 현재 자세에서 재보정하고, 시뮬레이터 재시작은 새로운 boot를 감지해 HOME에서 다시 보정합니다. tracking 상실·receiver 응답 상실 때는 action 송신을 중단하며, 보정 중에는 `F16H`로 정지시킵니다.

Isaac는 매 physics step(60Hz)에서 연속 관절 목표를 생성합니다. 기본 목표 속도 **0.25rad/s ≈ 14.3°/s**, 목표 가속도 **0.50rad/s² ≈ 28.6°/s²**이며 `config/teleop_config.py`의 `ARM_MAX_VELOCITY`, `ARM_MAX_ACCELERATION`에서 조절합니다. 기존 PhysX 팔 drive gain과 solver 속도 한계는 유지합니다. 같은 낮은 속도를 PhysX hard limit으로도 강제하면 설치된 모델의 강한 drive와 충돌해 정지 자세에서도 solver가 불안정해졌으므로, 연속 drive reference로 느린 이동을 제어합니다. 새 IK 해가 멀리 있거나 다른 branch여도 경로의 위치·속도를 초기화하지 않습니다. `--no-filter`도 이 제한을 해제하지 않습니다.

팔이 충돌 등으로 목표를 따라가지 못해 reference와 측정 관절의 차이가 `ARM_MAX_TRACKING_ERROR=0.05rad`를 넘으면 해당 관절 reference를 감속시켜 목표가 계속 멀어지는 것을 막습니다. 속도 제한은 손끝의 직선 속도 제한이나 충돌 회피 계획을 뜻하지 않습니다. 시뮬레이터 초기 HOME 설정 때만 joint state를 쓰며, 운용 중에는 drive target으로 이동합니다.

## Pose / IK

현재 URDF의 양팔 가동범위를 scikit-robot으로 시각화하려면
[ARM_WORKSPACE.md](ARM_WORKSPACE.md)를 참고하세요. 자유 방향 범위와 현재
수직 그리퍼 조건의 범위를 테이블 위에서 비교하는 HTML/PNG를 생성합니다.

설치된 wrapper는 이미 OpenXR → robot basis를 변환합니다. `+X=forward, +Y=left, +Z=up`. 기본 `arm_reference_mode="head_yaw"`는 head 위치와 yaw에 상대적인 pose를 반환합니다. 이를 그대로 유지했으므로 acceptance test 때 머리는 고정하세요. 머리를 움직이면 wrapper 좌표도 변합니다.

```python
p_target = p_robot_anchor + (p_controller_now - p_controller_neutral)
R_target = LEFT_EE_DOWN_ROT  # right arm: RIGHT_EE_DOWN_ROT
# R_target @ [0, -1, 0] == [0, 0, -1]
```

위치 anchor는 보정 시 실제 측정한 로봇 자세의 FK입니다. 컨트롤러의
비틀기·위아래 꺾기·좌우 꺾기는 모두 무시하고, 위치 변화량만 사용합니다.
보정의 정지 여부도 위치로만 판단합니다. URDF 손가락의 전방 축은 손목
링크의 `-Y`이므로, 양손별 고정 회전으로 이 축을 world `-Z`에 맞춥니다.
Yaw도 고정하며 손가락 개폐 축은 world Y에 놓입니다.

solver는 **XYZ와 고정 방향을 함께 푸는 전체 pose IK**입니다. 방향 조건을
없애는 방식이 아니며, 5~7번을 포함한 모든 팔 관절이 필요에 따라 움직여
수직 목표를 유지합니다. 위치 gain은 1이고 5mm neutral deadband가 있습니다.
원래 컨트롤러 목표는 유지하되, IK 목적지는 현재 측정 FK에서 목표 방향으로 최대 4cm씩 따라갑니다. 새 controller event만 처리하고,
위치·그리퍼 필터 계수는 실제 처리 간격에 따라 보정해 기존 30Hz의 응답
시간을 유지합니다. `--no-filter`는 입력 필터만 끄며 관절 속도·가속도 제한은
유지합니다. `--position-only`는 호환 옵션으로, 기본 동작과 동일합니다.

각 IK 목적지의 방향은 항상 수직 아래입니다. 실제 물리 드라이브에는 추종
오차가 있으며, 속도 제한 관절 경로의 중간 자세에는 작은 방향 오차가 있을
수 있습니다. 가동범위 밖이나 테이블 충돌이 생기는 XYZ는 여전히 도달할 수
없습니다. LeRobot에는 이전과 똑같이 모든 관절의 실제 값과 구동 명령을 저장합니다.

rev 2.0.1 URDF, `left/right_dof7_link`의 LOCAL Jacobian DLS를 사용합니다.
현재 최대 100 iteration, `eps=2e-4`, `dt=.3`, damping `1e-4`입니다.
**IK가 실패하거나 관절 해가 0.20rad 이상 바뀌면 이동량을 절반으로 줄여 최대 4번 재시도**합니다. 추가 탐색은 12ms가 지난 후에는 시작하지 않고 재시도당 최대 50 iteration을 사용합니다. 성공한 해만 FK·관절 한계·수직 방향을 검사해 보냅니다. 유효한 해가 없을 때만 `F16H`로 양팔을 감속 정지하며, 실패한 partial q는 보내지 않습니다.
`ACTIVE / NEARBY IK`는 축소한 위치로 계속 이동하는 상태입니다. `step`, `fraction`, `remainingL/R_mm`, `attempts`로 원래 목표까지 남은 거리와 탐색 횟수를 확인합니다. `IK HOLD` 로그의 `posErrL/R_mm`, `rotErrL/R_deg`, `nearLimits`, `iter`로
팔별 잔차와 관절 한계를 구분합니다. trigger는 독립적으로 사용할 수 있습니다.

## Gripper

wrapper source를 한 번 검사하여 trigger encoding을 선택한 뒤, 실제 Quest 입력 방향에 맞게 trigger closure를 반전합니다. 따라서 trigger를 놓으면 gripper가 열리고 누르면 닫힙니다. `--trigger-encoding standard|legacy-inverted-10`으로 다른 wrapper를 명시할 수도 있습니다. squeeze는 기본 0→1이며 반전하지 않습니다.

```python
opening = 0.0425 * (1 - closure)
finger_targets = [-opening_left, +opening_left,
                 -opening_right, +opening_right]
```

실제 USD limit은 0.042489517m이므로 finger endpoint에만 약 10μm 차이를 반영합니다. 실제 GPU 검사에서 원본 finger drive 강성 2.3~2.8 N/m은 중력을 버티지 못했습니다. **런타임 finger actuator에만** stiffness=2000 N/m, damping=30 N·s/m를 설정했습니다. USD effort limit 14N과 모든 arm drive gain은 유지합니다. 이는 위치 mapping gain 변경과 무관합니다.

## 카메라와 dataset interface

Quest A/B/X를 사용하는 LeRobot v3 수집을 지원합니다. Isaac에 `--record`를
추가하면 오른손 A=녹화 시작, B=정지, 정지 후 A=저장 / 왼손 X=폐기입니다.
A 이전에는 HOME과 열린 그리퍼를 유지하며, 녹화 중에만 조작을 적용합니다.
영상은 녹화 중 병렬 인코딩하고, 저장 확정은 속도 제한 HOME 복귀와 함께
진행합니다. 새 큐브 배치와 저장이 모두 끝나면 다음 시연을 시작할 수 있습니다.
Quest 영상 평면은 `config/teleop_config.py`의 `QUEST_SCREEN_DISTANCE=1.5`m에 표시합니다.
실행 명령과 π0.5 연결은 [RECORDING_LEROBOT.md](RECORDING_LEROBOT.md)를 참고하세요.

현재 장면은 60×80cm 테이블(상판 높이 30cm), 테이블 중앙의 열린 수집 상자(외부 18×18×4cm), 밝은 빨간색 3cm 큐브 1개를 포함합니다. 상자는 바닥과 네 벽에 충돌이 있고 윗면이 열려 있습니다. 큐브는 20g rigid body로 중력·충돌·마찰이 적용되어 테이블 위에 놓이거나 상자 안에 담길 수 있습니다.

큐브는 Isaac 시작 시 매번 다른 위치와 yaw로 생성됩니다. 상자 양쪽 경계에서 2cm 이상 떨어진 **왼쪽(+Y) 또는 오른쪽(-Y) 구역**에서 생성합니다. 상자가 있는 중앙 띠에는 생성하지 않고, 몸통 쪽 테이블 끝을 기준으로 길이의 **1/4~2/3 구간**만 사용합니다. 가까운 첫 1/4과 먼 쪽 마지막 1/3에는 생성하지 않습니다. 회전한 큐브의 모서리까지 금지 구역과 테이블 가장자리를 침범하지 않게 배치합니다. 물리 동작 중 사용자가 왼쪽 구역으로 옮기는 것은 가능합니다.

테이블 중심은 `TABLE_CENTER=(0.45, 0, 0.28)`로 기존보다 몸통 쪽으로 10cm 이동했습니다. 기본 큐브 생성은 실제 teleop의 100회 IK로 HOME→접근→집기→들기→상자 위→내려놓기 경로를 검사한 **260개 위치(좌우 각각 130개)**로 제한합니다. 카메라 시야 안의 위치만 사용하며, 초기 팔에 가려지는 위치는 허용합니다. 세부 설정과 범위는 [TABLETOP_IK_SPAWN.md](TABLETOP_IK_SPAWN.md)를 참고하세요.

배치·상자 크기·큐브 수/크기/색상은 `config/tabletop_config.py`에서 수정합니다. `CUBE_COUNT`를 늘리면 초기 큐브끼리 겹치지 않게 배치합니다. 같은 배치를 재현하려면 Isaac 실행 명령에 `--scene-seed 42`를 추가하세요. 기존 `--camera-debug` 색상 큐브는 별도의 시야 확인용입니다.

```bash
# 테이블 지지·상자 바닥/벽 충돌·실제 RGB를 검사하고 종료
python scripts/test_tabletop.py --headless --device cuda:0
# 결과: outputs/tabletop_check/validation.json 및 PNG

# 큰 IK 목표·실패 후 재개·통신 timeout의 연속 이동 검사
python scripts/test_joint_motion.py --headless --device cuda:0
# 결과: outputs/joint_motion_check/validation.json, samples.json
```

`CameraCfg.OffsetCfg(convention="world")`의 camera 축은 forward=+X, up=+Z입니다. 아래 offset은 **parent link 좌표**, quaternion은 **wxyz**입니다.

| observation 이름 | parent | 해상도 | offset (m) | rotation wxyz |
|---|---|---|---|---|
| front / BODY | base_link | 640×480 | (.25, 0, .95) | (.803857, 0, .594823, 0) |
| left_wrist | left_dof7_link | 320×240 | (.055, -.03, 0) | (.379928, -.379928, .596368, -.596368) |
| right_wrist | right_dof7_link | 320×240 | (-.055, -.03, 0) | (.596368, .596368, -.379928, -.379928) |

Body는 전방 아래 73도이며, 높이 95cm·전방 25cm에서 테이블을 내려다봅니다. Body focal length는 16mm로 시야를 넓혔습니다. wrist는 link의 -Y 집기 방향을 기준으로 camera 아래 25도입니다. 좌우 link가 mirror이므로 영상의 위쪽 방향도 각각 local +X/-X로 맞췄습니다. offset은 config에서 조정합니다. 실제 rigid link를 이름으로 찾아 그 아래 camera prim을 생성하므로 import directory나 joint numeric ordering에 의존하지 않습니다.

세 센서의 `camera.data.output["rgb"]`를 그대로 유지합니다. raw는 `(1,H,W,3)` uint8 tensor이며 robot state는 `(1,16)`입니다. operator 변환 때만 batch index를 선택하고 CPU로 복사합니다. dataset callback은 아래 구조를 받습니다.

```python
{
    "observation.state": measured_state_16,
    "observation.images.front": body_rgb,
    "observation.images.left_wrist": left_wrist_rgb,
    "observation.images.right_wrist": right_wrist_rgb,
}
# run(..., observation_callback=callback)
# callback(observation, commanded_action_16, simulation_time_seconds)
```

callback의 `commanded_action_16`은 최종 IK 목적지가 아니라 그 시점에 실제 적용한 속도 제한 drive reference입니다. `observation.state`는 실제 측정값입니다. callback이 다음 frame 이후에도 데이터를 보관하려면 tensor를 clone해야 합니다. 센서 버퍼는 재사용됩니다. 현재 gripper state는 command가 아니라 실제 두 finger 위치로부터 평균 opening을 계산합니다. callback에서는 오래 걸리는 I/O를 하지 마세요. `--record`의 LeRobot 저장은 별도 프로세스에서 수행하며 관측과 다음 카메라 간격의 적용 명령을 짝짓습니다.

Display composite는 1280×720, BODY가 가용 폭의 64%, 좌우 각각 18%, aspect-ratio 유지입니다. resize와 label은 새 canvas에만 적용합니다. JPEG quality=80, PUB/SUB 단일 메시지, CONFLATE, pending frame 1개, nonblocking send를 사용합니다. JPEG encoder는 Isaac main loop 밖 thread에서 실행하며 socket도 해당 thread가 소유합니다. SUB는 최신 frame 하나만 decode합니다. 영상이 1초 이상 stale이면 Quest에 안내 이미지를 표시하고 자동 재연결을 기다립니다. `render_to_xr`가 **BGR을 입력받아 내부에서 RGB로 변환**하는 설치 버전의 동작을 adapter에 명시했습니다.

기본 physics/control/camera는 60/60/30Hz입니다. `QUEST_INPUT_HZ=60`은 브라우저에 요청하는 controller 스트림 주기이고 실제 수신 주기는 따로 측정합니다. `FEEDBACK_HZ=60`으로 실제 관절 피드백을 조회합니다. rendering과 GPU→CPU 복사는 camera 주기에만 합니다. 처리시간이 길어지면 실제 Hz는 낮아지며 오래된 frame을 따라잡기 위해 재생하지 않습니다. 입력 주기와 최적화 측정값은 [QUEST_INPUT_CHECK.md](QUEST_INPUT_CHECK.md)를 참고하세요.

## SSL / LAN

탐색 순서는 기존 TeleVuer와 같습니다: `XR_TELEOP_CERT` + `XR_TELEOP_KEY` → `~/.config/xr_teleoperate/{cert,key}.pem` → TeleVuer package root. startup에서 경로를 검사합니다. 비밀 key 내용은 출력하지 않습니다.

**현재 발견한 기존 인증서에는 Subject Alternative Name이 없습니다.** 로컬 WSS 테스트는 이 self-signed 인증서를 사용했지만, Quest Browser에서 현재 LAN IP의 신뢰 여부는 별도 확인이 필요합니다. 기존 인증서를 보존하려면 새 IP용 파일명을 사용하세요.

```bash
HOST_IP=<COMPANY_PC_IP>
mkdir -p ~/.config/xr_teleoperate
openssl req -x509 -newkey rsa:2048 -nodes -days 365 \
  -keyout ~/.config/xr_teleoperate/company-key.pem \
  -out ~/.config/xr_teleoperate/company-cert.pem \
  -subj "/CN=${HOST_IP}" \
  -addext "subjectAltName=IP:${HOST_IP}"
chmod 600 ~/.config/xr_teleoperate/company-key.pem
export XR_TELEOP_CERT=~/.config/xr_teleoperate/company-cert.pem
export XR_TELEOP_KEY=~/.config/xr_teleoperate/company-key.pem
```

회사에서 신뢰하는 CA 발급 인증서를 사용할 수 있다면 그것을 우선 사용하세요. self-signed SAN을 넣는 것만으로 Quest가 자동 신뢰하지는 않습니다. Quest에서 HTTPS 페이지를 먼저 열어 인증서 신뢰를 처리하고 XR 진입을 확인하세요. 필요시 Linux 방화벽:

```bash
sudo ufw allow 8012/tcp
```

Quest와 PC가 같은 LAN이어도 Wi-Fi AP client isolation이나 회사 정책이 단말 간 통신을 차단할 수 있습니다. 별도 터널/Windows forwarding은 필요하지 않습니다.

## 진단 / acceptance tests

작업 directory는 `~/stkim_ws/quest_robot_isaac`, Python은 `env_isaaclab`입니다. 같은 8012/5005/5556을 쓰는 live 프로세스와 진단 프로세스를 동시에 띄우지 마세요.

```bash
# 순수 수학 + IK + 실제 UDP/ZMQ 회귀 테스트
python -m pytest -q
python scripts/test_ik.py
python scripts/test_udp_action.py

# Quest controller tracking만 (pass-through는 이 진단에서만 사용)
python scripts/test_quest_tracking.py --host-ip <COMPANY_PC_IP>

# Quest 영상: synthetic RGB 색 확인 / 실제 Isaac 영상
python scripts/test_quest_video.py --pattern --host-ip <COMPANY_PC_IP>
python scripts/test_quest_video.py --host-ip <COMPANY_PC_IP>

# 하드웨어 없이 실제 HTTPS/WSS events·video·disconnect 검사
python scripts/test_quest_video.py --self-test

# 실제 두 process + 합성 controller WSS 입력의 전체 경로 검사 (GPU 필요)
python scripts/test_pipeline.py

# 실제 GPU HOME·좌우 독립 gripper·wrist camera parenting·RGB 검사
cd ~/IsaacLab_2_3_2/IsaacLab
./isaaclab.sh -p ~/stkim_ws/quest_robot_isaac/scripts/test_cameras.py --headless --device cuda:0
```

Camera test는 360 physics step 후 종료하고 `outputs/camera_check/`에 raw PNG, composite, 이동 후 PNG, camera pose JSON, 성공 시 `validation.json`을 저장합니다. `ISAAC SMOKE TEST PASSED` 메시지까지 확인하세요. 단순 process exit code만으로 통과를 판단하지 않습니다. Kit fast shutdown이 오류 exit code를 덮을 수 있어 traceback도 확인합니다. 일반 실행에서 snapshot만 원하면 `--snapshot-dir outputs/view --camera-debug`를 사용합니다.

Pipeline test는 실제 Isaac와 Quest main server를 child process로 실행하고 합성 WSS 입력을 보냅니다. 15005/15556을 사용하며 8012는 비어 있어야 합니다. IK를 통과한 action 수신, 실제 RGB 영상 IPC, tracking loss HOLD, sender 재시작, simulator 재시작과 양쪽 종료를 검사한 뒤 `outputs/pipeline_check/`에 로그와 `validation.json`을 남깁니다. 물리 Quest 자체는 사용하지 않습니다.

실제 Isaac 실행 중 양팔 HOME + 왼쪽 gripper만 닫는 별도 제어 진단:

```bash
python scripts/test_udp_action.py --send --left-gripper 1 --right-gripper 0 --seconds 3
```

Quest hardware acceptance는 다음을 확인하세요.

1. calibration 후 머리를 고정하고 한 controller를 +10cm/+20cm 이동: `--verbose --no-filter`의 `rawL/rawR`, `Lxyz/Rxyz`가 .10/.20m. 원래 요청 이동량은 유지되지만 실제 IK 목적지는 가능한 작은 단계로 제한됨.
2. 컨트롤러 XYZ를 고정한 채 각 축을 회전: 팔은 움직이지 않고 양쪽 그리퍼는 아래를 향함. XYZ 이동 중에도 로그는 `orientation=DOWN_FIXED`, 그리퍼 방향 유지.
3. trigger release=open / press=closed. 좌우가 독립. `--gripper-input squeeze`도 확인.
4. BODY 중앙 크게, 양쪽 WRIST 작게. 손목 이동에 맞춰 raw camera와 Quest 시야가 동일하게 움직임.
5. controller sleep/브라우저 종료 → tracking timeout → Isaac HOLD. 다시 연결 → 현재 held pose 기준 재보정.
6. 어느 프로세스를 먼저 종료해도 다른 프로세스가 crash하지 않음. 종료 후 포트 재사용 가능.

일부 XR client가 tracking이 끊겨도 유효한 stale pose를 계속 송신한다면 서버는 새 pose와 구분할 수 없습니다. 이벤트 정지, invalid matrix, 명시적 `connected/tracked=False`는 검출합니다. 실제 Quest sleep/occlusion 동작은 hardware 검사 항목입니다.

## 구현 범위와 남은 실기 확인

control, camera, video IPC, LeRobot v3 에피소드 수집과 tabletop 장면이 구현되어 있습니다. 모델 학습은 별도 LeRobot π0.5 환경에서 수행합니다. collision avoidance나 실제 로봇 torque safety는 포함하지 않습니다. 이 프로그램은 Isaac simulation용입니다.

설치 버전에 특유한 카메라 Fabric 초기 pose 동기화와 STOP callback을 코드에 설명했습니다. 원본 package를 수정하지 않고 앱 초기화/종료 순서에서 처리합니다. Quest 착용 상태의 XYZ 조작감, LAN 인증서 신뢰, headset browser XR 진입, 실제 controller tracking loss, 사용자에게 편한 wrist 시야는 현장 확인이 필요합니다. 자세한 자동 실행 결과는 `VALIDATION_TELEOP.md`에 기록합니다.

로봇 외형 색상은 `robot/appearance.py`의 `PALETTE`에서 관리합니다. 받침 기둥과 중앙 상단 부품을 포함한 로봇 전체는 팔과 같은 블랙, 그리퍼 손가락은 파란색 `#2F90CE`입니다. Isaac에서는 링크별 재질을 적용하며 URDF와 가동범위 HTML에도 같은 색 구성을 반영했습니다.

녹화 시작 A를 누를 때마다 해당 버튼 입력 프레임의 양손 XYZ를 새 중립 기준으로 잡고 이동 필터를 초기화합니다. 대기 중 손을 옮겨도 이전 에피소드의 기준을 사용하지 않습니다. Quest 로그의 `New episode neutral captured at A press`로 매 시작의 기준 갱신을 확인할 수 있습니다. REVIEW에서 저장하는 A는 기준을 바꾸지 않습니다.

수집한 데이터로 로컬 OpenPI π0.5 LoRA 학습을 시작하는 명령과 준비 결과는 [OPENPI_TRAINING.md](OPENPI_TRAINING.md)를 참고하세요.
