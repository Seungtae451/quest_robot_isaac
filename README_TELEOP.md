# F14 / FlaminGO — Quest 3 × Isaac Sim teleoperation

두 개의 터미널에서 실행하는 **XYZ 이동 + 회전 + 그리퍼 개폐** 양팔 teleoperation입니다. 기본 Quest 화면은 **passthrough AR의 실제 크기 3D 장면**입니다. IK 목표는 **그리퍼 길이 중앙의 XYZ와 컨트롤러 회전 변화량**을 사용합니다. 수직 자세를 강제하지 않습니다. **사용자의 오른손→로봇 왼팔, 왼손→로봇 오른팔**로 대응하고 고정 공간의 XYZ 방향은 반전하지 않습니다. 왼팔 7 + 오른팔 7 + 좌우 gripper 2의 **16D semantic action**과 실제 VLA RGB 카메라 3개를 사용합니다. HOME과 그리퍼 중앙 TCP offset은 `robot/f14_config.py`에서 설정합니다.

초기 HOME은 왼팔 `[0, 0.64, -0.11, -1.53, -0.58, 0, 0]`, 오른팔 `[0, -0.64, 0.11, -1.53, 0.58, 0, 0]`rad입니다. joint 2·3·5는 좌우 대칭 부호로 적용합니다. 조작 TCP는 그리퍼 길이 중앙이며 손목 로컬 offset은 `(0, -0.18125, -0.0225)`m입니다. 변경 후 **Isaac과 Quest 송신기를 모두 재시작하고 Quest 페이지도 새로 고침**하세요.

현재 IK URDF는 `assets/F14_URDF_rev_2_0_1 2/urdf/FlaminGO_14Dof_Arm_Robot_v2.urdf`입니다. action과 HOME은 이 모델의 관절 각도를 사용합니다. 기존 Isaac USD와 축이 반대인 오른팔 dof3/dof4는 시뮬레이터 경계에서 부호를 변환하며, 측정 state와 수신 관절 한계도 같은 기준으로 변환합니다. 따라서 두 관절 HOME의 부호를 바꿔도 실제 시작 자세는 유지됩니다.

## 실행

Quest 표시용 모델과 Three.js를 한 번 준비합니다. 현재 PC에는 이미 생성했습니다.
URDF/색상/원본 STL이 바뀌면 다시 실행하세요. 표시용 내보내기는 numpy만 사용합니다.

```bash
cd ~/stkim_ws/quest_robot_isaac
uv run --with numpy python scripts/export_quest_ar_assets.py
```

현재 rev 2.0.1 **URDF의 visual 20개(19개 링크), 원본 삼각형 3,931,282개를 전부**
링크 로컬 좌표의 GLB로 내보냅니다. 메시 단순화나 부품 생략을 하지 않습니다.
base의 기둥과 상부 바디도 별도 visual로 모두 포함하고 URDF 색상을 유지합니다.
물리 모델·관절·카메라와 원본 asset 파일은 변경하지 않습니다. 로컬
`outputs/quest_ar/assets`에 GLB와 고정 버전 Three.js를 저장하므로 Quest는
외부 CDN에 접속할 필요가 없습니다. HTTPS 서버 의존성은 `aiohttp`입니다.
원본 GLB는 약 303MB이며 손실 없는 gzip 전송 파일(약 82MB)을 함께 생성합니다.
처음 로딩에는 시간이 필요합니다. 재생성 후 **Quest 송신기를 재시작하고
Quest Browser 페이지를 새로 고침**하세요. 로딩이 끝난 뒤 AR을 시작합니다.
녹화 환경이 없으면 먼저 [저장 환경 설치](RECORDING_LEROBOT.md#실행)를 수행하세요.

Terminal 1 — Isaac:

```bash
conda activate env_isaaclab
cd ~/IsaacLab_2_3_2/IsaacLab
./isaaclab.sh -p ~/stkim_ws/quest_robot_isaac/scripts/run_isaac_teleop.py --device cuda:0 --record --no-quest-video
```

GUI가 필요 없으면 `--headless`를 추가합니다. 카메라 렌더링은 launcher가 `enable_cameras=True`로 자동 활성화합니다.

Terminal 2 — Quest / IK:

```bash
conda activate env_isaaclab
cd ~/stkim_ws/quest_robot_isaac
python scripts/run_quest_teleop.py --host-ip <COMPANY_PC_IP> #192.168.100.136
```

IP를 생략하면 현재 라우팅 정보를 통해 탐색합니다. 여러 NIC가 있으면 `ip -4 addr`로 Quest와 같은 LAN의 주소를 확인하고 명시하세요. 서버는 `0.0.0.0:8012`에 bind하며 실행 시 아래 URL을 출력합니다.

```text
https://<COMPANY_PC_IP>:8012/
```

Quest Browser에서 주소를 열고 인증서를 신뢰한 뒤 **AR 시작**을 누르세요.
실제 방 위에 로봇·책상·상자·큐브가 나타납니다. Isaac ground와 배경은
Quest에서 그리지 않습니다. Quest가 양안과 머리 움직임을 직접 렌더링하며
Isaac은 측정된 19개 링크와 큐브 pose만 별도 ZMQ로 전송합니다.

1. 오른손으로 바닥을 가리켜 원점을 정합니다. 배치 중 오른쪽 스틱은 yaw,
   왼쪽 스틱은 위치를 미세 조정합니다. **첫 A는 공간 배치만 고정**합니다.
2. **오른손 컨트롤러를 로봇 왼팔 그리퍼 중앙, 왼손 컨트롤러를 로봇 오른팔 그리퍼 중앙의 반지름 1cm 구**에 넣습니다. 구의 색과 거리 안내를 확인하세요.
3. HOME/열린 그리퍼/최신 추적이 준비된 뒤 **다시 A를 눌러 녹화와 조작**을 시작합니다.
   매번 A 순간의 대응 컨트롤러와 측정 그리퍼 중앙 위치를 새 기준으로 캡처합니다.
4. B로 정지하고 검토 중 A로 저장하거나 X로 폐기합니다. HOME 복귀·새 큐브 후
   같은 공간 배치를 유지하며 2번부터 반복합니다.

연결 거리는 **손 전체나 손가락 끝이 아닌 하늘색 컨트롤러 grip 추적점** 기준입니다.
좌우 구는 각각 위치 1cm 이내이고 대응 축 각각 10° 이내이면 초록색, 범위 밖이면 주황색, 추적/장면이 없으면
회색입니다. 하늘색 점과 구는 로봇 메시 안에 들어가도 보이도록 표시합니다.
양쪽 구를 초록색으로 만든 뒤 `서버: 연결 준비 완료`를 확인하고 A를 놓았다가
다시 누르세요. 구의 중심은 joint 7이 아니라 **그리퍼 길이 중앙 TCP**입니다. HUD의 L/R은 로봇 팔 기준이며 각각 사용자 오른손/왼손에 대응합니다.
컨트롤러 X/Y축과 그리퍼 중앙 X/Z축을 길이 5cm의 반투명 흰색 점선으로 표시합니다. 아래 좌우 대응에 따라 각각 10° 이내로 정렬하세요.
머리를 움직이거나 몸을 이동해 연결 위치를 확인할 수 있습니다.

A 시작 거부 시 HUD는 **A 순간의 L/R 거리와 거부 이유**를 8초간 표시합니다.
현재 거리도 0.01cm 단위로 표시합니다(추적 장치 정확도를 뜻하지는 않습니다).
터미널의 `LEFT_OUTSIDE_1CM`/`RIGHT_OUTSIDE_1CM`은 해당 로봇 팔과 대응 손의 거리 초과,
`LEFT_AXES_MISALIGNED`/`RIGHT_AXES_MISALIGNED`는 축 정렬 오차 초과입니다.
`HOME_JOINT_ERROR`/`HOME_STILL_MOVING`/`HOME_GRIPPER_NOT_OPEN`은 초기
로봇 상태 미준비입니다. 시작 판정과 마지막 A의 결과는
`outputs/quest_input_check/runtime.jsonl`의 `ar_start`에도 저장되며
LeRobot feature에 추가하지 않습니다. 거리와 HOME 제한은 완화하지 않았습니다.

AR의 입력 기준은 **A로 고정한 로봇 공간**이며 head_yaw 변환을 사용하지 않습니다.
실제 1m를 1m로 취급하고, 시작 시 위치 1cm·회전 10° 이내 오프셋은 그대로 유지해 갑작스러운
팔 이동 없이 상대 XYZ 이동량과 공간 기준 회전 변화량을 전달합니다. 머리를 돌려도 입력 축이 바뀌지 않습니다.
녹화 중에는 사용자가 걸으면서 이동한 컨트롤러 위치도 팔 목표에 반영됩니다.
구경하려면 A 전 또는 B로 정지한 상태에서 이동하세요.

**LeRobot은 기존 body(`front`), left_wrist, right_wrist RGB와 state16/action16만 저장**합니다.
passthrough·머리 시점·손 표시·1cm 구·점선 축·UI는 브라우저 전용이며 Isaac USD 장면이나
녹화 카메라에 추가하지 않습니다. Quest 화면 자체는 별도 녹화하지 않습니다.
3D 표시의 조명/반사는 Isaac RTX와 다를 수 있지만 URDF visual 형상은 그대로 사용합니다.
실제 방의 벽/사용자에 대한 가림·물리 충돌을 시뮬레이션하는 기능은 포함하지 않습니다.

공간 앵커가 지원되면 사용하고 local-floor 기준 배치를 fallback으로 제공합니다.
일시적인 컨트롤러 추적 끊김은 조작과 녹화를 정지시켜 REVIEW로 넘깁니다.
머리 이벤트만 늦었다는 이유로 팔을 재보정하지 않고, 추적 복귀만으로 조작을 자동 재개하지 않습니다.
XR 원점 reset·앵커 점프·세션 재접속은 공간을 다시 배치해야 합니다.
브라우저 종료 후 공간 앵커를 영구 복원하는 기능은 포함하지 않습니다.
AR에서 `R` + Enter는 조작을 정지시키며 새 A 연결을 요구합니다. 양쪽 터미널은 Ctrl+C로 종료합니다.

### 기존 2D 화면 사용

Isaac 명령에서 `--no-quest-video`를 빼고 Quest 명령에 `--xr-mode immersive` 또는
`--xr-mode ego`를 붙이면 기존 TeleVuer body 카메라 패널을 사용할 수 있습니다.
이 모드의 주소는 `https://<COMPANY_PC_IP>:8012/?ws=wss://<COMPANY_PC_IP>:8012&grid=False`이며
`https://vuer.ai?ws=wss://<COMPANY_PC_IP>:8012&grid=False`도 지원합니다.
2D 모드만 기존 head_yaw 기준과 2초/45개 표본 자동 보정을 사용합니다.
그 모드의 매핑 확인에서는 머리와 양손을 가만히 유지하고 trigger를 놓으세요.
AR을 쓰지 않는 Isaac에서는 `--no-ar-scene`으로 표시용 스트림도 끌 수 있습니다.

## 확인한 환경과 의존성

### 컨트롤러 회전 무시 확인

Isaac을 평소처럼 실행하고, Quest 송신기를 아래 옵션으로 실행합니다.
컨트롤러 회전을 관찰하는 진단이며, 회전으로 로봇 방향을 바꾸지는 않습니다.
속도·가속도 제한은 유지합니다. `--no-filter`는 입력 필터의 지연만 제거합니다.

```bash
python scripts/run_quest_teleop.py --xr-mode immersive --wrist-axis-test --no-filter
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
| ProxQP (`proxsuite`) | 0.7.2 |
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
    TeleVuer wrapper → calibration → relative XYZ + fixed rotation → Pinocchio + ProxQP
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
| `robot/differential_ik.py` | Quest 전용 Pinocchio + ProxQP, 양팔 독립 속도 제어 |
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
| `simulation/quest_view_compositor.py` | Quest에 BODY 카메라 하나만 표시 |
| `simulation/isaac_teleop_app.py` | physics/camera loop, recorder callback |
| `scripts/run_*_teleop.py` | 두 runtime의 실행 진입점 |
| `scripts/test_*.py`, `tests/` | 독립 진단과 regression tests |

기존 `teleop/quest_ik_sender.py`의 사용자 수정, `scripts/isaac_joint_receiver.py`, `scripts/teleop_quest_isaac.py`, `scripts/test_f14.py`는 그대로 보존했습니다. **새 시스템은 `run_*_teleop.py`로 실행**하세요. 기존 sender/receiver에는 14D packet과 magic 불일치가 있으므로 새 실행 경로와 섞지 않습니다. 기존 단일 프로세스 스크립트도 reference용으로만 남겼습니다. MuJoCo 프로젝트와 원본 USD/URDF는 수정하지 않았습니다.

## 16D action과 수신 안전 동작

```python
action_16 = [*left_arm_q_7, *right_arm_q_7, left_closure, right_closure]
# 0:7 left radians / 7:14 right radians / 14:16 [0=open, 1=closed]
```

UDP `!4sId16f`, 정확히 **80 bytes**: magic, uint32 sequence, float64 timestamp, float32 16개. 현재 Quest는 `F16D`로 적분된 differential IK 관절 reference를 전송합니다. Isaac은 추가 목적지 저역통과 없이 따라가되 속도·가속도·추종 제한과 timeout 감속을 계속 적용합니다. `F16A`의 팔 각도는 검증된 다음 IK 목적지이며, PhysX에 바로 적용하지 않습니다. 같은 크기의 `F16H` packet은 IK 실패/보정 중 팔 감속 정지를 요청하고 gripper 입력은 독립적으로 적용합니다. timestamp는 동일 Linux host의 `time.monotonic()` 초입니다. Wall-clock 조정 영향을 받지 않으며, **다른 PC로 action UDP를 옮길 때는 clock/schema 재설계가 필요**합니다. 외부 LAN에 action/video 포트를 열지 않습니다.

receiver는 길이, magic, finite 값, gripper 범위, USD arm limits, 시간 순서와 0.5초 freshness를 검사합니다. 잘못된 packet은 watchdog을 갱신하지 않습니다. 수신 queue를 비워 가장 최신 유효 packet만 적용합니다. sequence가 0으로 돌아가는 sender 재시작도 monotonic timestamp로 처리합니다.

- `WAITING`: 아직 유효 명령 없음 → HOME / open.
- `ACTIVE` + `F16D`: differential reference를 속도·가속도·추종 제한 안에서 따라감.
- `ACTIVE` + `F16A`: 유효한 최근 IK 목적지까지 속도·가속도를 제한하며 이동.
- `ACTIVE` + `F16H`: 팔을 감속 정지하고 정지한 drive reference 유지. gripper는 계속 사용 가능.
- `HOLD`: 마지막 명령이 0.5초보다 오래됨 → 오래된 목적지를 계속 쫓지 않고 감속 정지.

읽기 전용 `F16?` 요청에 수신기가 `F16S + boot monotonic time + measured state`를 돌려줍니다. 아직 도달하지 않은 IK 목적지가 아니라 실제 측정된 관절/그리퍼 자세입니다. IK 초기값과 재보정 anchor도 이 측정값을 사용합니다. 송신기 재시작은 현재 자세에서 재보정하고, 시뮬레이터 재시작은 새로운 boot를 감지해 HOME에서 다시 보정합니다. tracking 상실·receiver 응답 상실 때는 action 송신을 중단하며, 보정 중에는 `F16H`로 정지시킵니다.

Isaac는 매 physics step(60Hz)에서 연속 관절 목표를 생성합니다. 기본 목표 속도 **3.14rad/s ≈ 180°/s**, 목표 가속도 **10.0rad/s²**이며 `config/teleop_config.py`의 `ARM_MAX_VELOCITY`, `ARM_MAX_ACCELERATION`에서 조절합니다. 기존 PhysX 팔 drive gain과 solver 속도 한계는 유지합니다. 같은 낮은 속도를 PhysX hard limit으로도 강제하면 설치된 모델의 강한 drive와 충돌해 정지 자세에서도 solver가 불안정해졌으므로, 연속 drive reference로 느린 이동을 제어합니다. 새 IK 해가 멀리 있거나 다른 branch여도 경로의 위치·속도를 초기화하지 않습니다. `--no-filter`도 이 제한을 해제하지 않습니다.

팔이 충돌 등으로 목표를 따라가지 못해 reference와 측정 관절의 차이가 `ARM_MAX_TRACKING_ERROR=0.05rad`를 넘으면 해당 관절 reference를 감속시켜 목표가 계속 멀어지는 것을 막습니다. 속도 제한은 손끝의 직선 속도 제한이나 충돌 회피 계획을 뜻하지 않습니다. 시뮬레이터 초기 HOME 설정 때만 joint state를 쓰며, 운용 중에는 drive target으로 이동합니다.

Teleoperation에서는 이 추종 오차를 줄이는 후퇴 명령을 허용합니다. reference를 측정값으로 순간 이동시키지 않고 기존 속도·가속도 한도 안에서 복구합니다. Inference의 추종 제한 동작은 기존대로 유지합니다.

## Pose / IK

현재 URDF의 양팔 가동범위를 scikit-robot으로 시각화하려면
[ARM_WORKSPACE.md](ARM_WORKSPACE.md)를 참고하세요. 자유 방향 범위와 현재
예전 수직 그리퍼 조건의 범위를 테이블 위에서 비교하는 HTML/PNG를 생성합니다.

AR에서는 A로 고정한 공간 변환의 역변환으로 컨트롤러 grip 위치를 로봇 좌표로 옮깁니다. `+X=forward, +Y=left, +Z=up`이며 머리 위치와 yaw는 입력에 반영하지 않습니다. 기존 2D 모드의 TeleVuer wrapper만 `arm_reference_mode="head_yaw"`를 사용하므로 해당 모드의 매핑 검사에서는 머리를 고정하세요.

```python
p_target = p_robot_anchor + (p_controller_now - p_controller_neutral)
# TCP = wrist.translation + wrist.rotation @ GRIPPER_CONTROL_OFFSET
# 로봇 L은 사용자 R, 로봇 R은 사용자 L. XYZ 부호는 그대로.
# 회전 target/회전 유지 제약은 사용하지 않습니다.
```

위치 anchor는 A 순간 실제 측정 관절로 계산한 **그리퍼 중앙 TCP**입니다. 컨트롤러의 비틀기·위아래 꺾기·좌우 꺾기는 모두 무시하고 위치 변화량만 사용합니다. 그리퍼 방향은 회전 명령 없이 팔의 움직임과 관절 한계 회피에 따라 자연스럽게 달라집니다. A 시점의 기준을 매번 새로 잡으며 머리 방향에 따라 XYZ 축을 뒤집지 않습니다.

현재 Quest 제어는 **Pinocchio + ProxQP differential IK**입니다. Pinocchio로
현재 연속 관절 reference의 FK와 LOCAL Jacobian을 구하고, ProxQP로 양팔의
7개 관절 속도를 각각 계산합니다. 먼 목표에 대한 전체 IK를 반복하거나
이동량을 절반씩 재시도하는 대신 `q_reference += qdot * control_dt`로 작은
이동을 누적합니다. A를 누를 때와 재보정이 끝날 때 실제 관절에서 새로 시작합니다.

QP 목적함수는 **그리퍼 중앙의 XYZ 속도 오차만** 줄입니다. 회전 비용·회전 유지 제약은 제거했습니다. 양수
정규화 항은 특이점에서 수치 계산을 안정화하고, Jacobian nullspace의
관절 한계 회피 항은 작업 자세를 유지하면서 여유 관절을 활용합니다.
관절 속도 3.14rad/s, 가속도 10.0rad/s², URDF 범위, 한계 전 감속 거리,
측정 관절과 reference 사이의 추종 지연을 제약으로 적용합니다. 추종 지연이
커져도 측정값 쪽으로 후퇴하는 방향은 허용합니다. `QP_POSITION_GAIN`,
`QP_MAX_CARTESIAN_SPEED`, `QP_LIMIT_GAIN` 등은 `config/teleop_config.py`에 있습니다.
그리퍼 XYZ 목표 속도 상한은 `QP_MAX_CARTESIAN_SPEED=0.75m/s`(75cm/s)입니다.
실제 이동은 관절 속도·가속도, 추종 오차 및 충돌 보조 제약에 따라 더 느릴 수 있습니다.

TCP의 LOCAL Jacobian은 Pinocchio의 고정 offset frame으로 계산해 손목 회전에 따른 끝점 이동도 포함합니다. 충돌 보조는 실제 기울기의 손가락 경계를 사용하고, QP에서는 `v_corner = v_TCP + ω × offset`으로 회전에 따른 접촉도 제한합니다. 적분 뒤 중간 자세와 최종 자세의 회전된 경계를 다시 확인합니다. 도달 불가능한 XYZ까지 이동할 수 있게 되지는 않으며 가능한 속도가 없으면 해당 팔은 감속합니다. 이 보조는 수집에만 적용하고 inference에는 적용하지 않습니다.

테이블·상자 보조는 녹화 중에만 적용됩니다. 기존 목표 경로 제한에 더해
QP에도 collider별 분리면 속도 제약을 넣고, 적분된 wrist 경로를 다시 검사합니다.
이미 여유 거리 안에 들어가 있으면 위로 들거나 바깥으로 빠져나갈 수 있습니다.
QP가 미수렴/불가능 상태를 반환하면 그 결과를 보내지 않습니다. 해당 팔만
안전한 감속 reference를 유지하고 다른 팔의 QP는 계속 실행합니다.

로그의 `step=QP`, `qpL/R`는 각 팔 QP 결과입니다. `QP_PARTIAL` /
`ACTIVE / QP ARM BRAKE`는 한쪽 결과가 거부되어 해당 팔이 감속 중이라는 뜻입니다.
`remainingL/R_mm`는 **실제 측정 FK에서 보조 적용 후 목표까지의 거리**입니다.
`IK=OK`는 해당 주기의 속도 계산이 성공했다는 의미이며 목표에 이미 도착했다는
뜻은 아닙니다. 두 팔 모두 유효한 QP가 없거나 보조 피드백이 끊기면 `F16H`로
감속합니다. gripper 입력은 독립적입니다.

속도 비교는 Quest 로그의 `handL/R_cm_s`와 `robotL/R_cm_s`를 봅니다.
전자는 로봇 좌표로 변환된 컨트롤러 XYZ(AR: 고정 공간, 기존 2D: head_yaw), 후자는 Isaac 측정 관절의
FK로 구한 그리퍼 XYZ 속도이며, 모두 실제 경과 시간 기준 cm/s입니다.
약 100ms 간격의 변위로 속도를 추정하고 최근 1초 평균을 표시하므로
짧은 왕복 움직임이나 tracking 잡음은 그대로 순간 속도로 표시되지 않습니다.
보정/에피소드 전환/추적 끊김에서는 측정을 초기화하고, 중복 피드백은 제외합니다.
`outputs/quest_input_check/runtime.jsonl`의 `motion_speed`에는 입력,
필터 후 목표, 충돌 보조 후 목표, QP reference, 실제 측정 FK 속도의 평균/P95가
각각 저장됩니다. 이 진단 로그는 LeRobot 데이터에 들어가지 않습니다.
`ik_max_ms`로 P95 밖의 IK 지연도 확인하며, `loop_p95_ms`/`loop_max_ms`는
영상 전달과 로그 출력을 포함한 전체 반복의 처리 시간입니다(주기 유지용 sleep 제외).
Isaac 로그의 `physics`는 실제 시간당 physics step 수이고 `rtf`는
시뮬레이션 시간/실제 시간입니다. 예를 들어 `rtf=0.7`이면 시뮬레이터가
실시간의 70% 속도로 진행하므로 IK 시간과 별도로 렌더링/physics 부담을 확인합니다.

입력 위치 gain 1, 1mm neutral deadband, 처리 간격에 맞춘 필터는 유지합니다.
`--no-filter`는 입력 필터만 끄며 관절 제한을 해제하지 않습니다.
`--position-only`는 회전을 무시하는 기존 XYZ 전용 호환 옵션입니다. 기본은 XYZ+회전입니다. LeRobot의 실제 관절 state16,
최종 실행 drive reference action16, body/좌우 wrist RGB 3개는 그대로 저장합니다.
**Inference의 모델 action 제어에는 ProxQP나 테이블 보조를 적용하지 않습니다.**
기존 반복 DLS IK는 작업 공간 및 스폰 위치의 오프라인 검증에 남아 있습니다.

GPU 검증 명령 (Quest 및 데이터 기록 없이 별도 테스트 환경):

```bash
python scripts/test_table_assist.py --headless --device cuda:0 --differential-ik
```

결과는 `outputs/proxqp_check/validation.json`에 저장합니다.
ProxQP 설정과 warm/cold 시작 API는 [공식 API 문서](https://simple-robotics.github.io/proxsuite/md_doc_22-ProxQP__api.html)를 참고했습니다.

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

녹화 중에는 실제 그리퍼 충돌 형상의 속도로 테이블·상자 접근을 예측하고,
상판과 상자 바닥·네 벽을 통과하는 XYZ 경로를 IK 전에 제한합니다.
상자 안으로는 그리퍼를 벽 위로 올린 뒤 접근합니다. 외력 없이 보정한 실행
action과 측정 state를 기록하며, inference에는 적용하지 않습니다.
초기 열린 손가락 등이 이미 벽의 안전 영역에 걸쳐 있으면 위쪽·바깥쪽 탈출을 허용하고, 벽 중심 쪽으로 더 들어가는 성분만 제거합니다. 이 탈출 경로도 다른 벽과 바닥에 대한 검사를 거칩니다.
설정은 `config/table_assist_config.py`, 비활성화 옵션은 Isaac의
`--no-table-assist`입니다.

현재 장면은 60×80cm 테이블(상판 높이 30cm), 테이블 중앙의 열린 수집 상자(외부 18×18×4cm), 밝은 빨간색 3cm 큐브 1개를 포함합니다. 상자는 바닥과 네 벽에 충돌이 있고 윗면이 열려 있습니다. 큐브는 20g rigid body로 중력·충돌·마찰이 적용되어 테이블 위에 놓이거나 상자 안에 담길 수 있습니다.

큐브는 Isaac 시작 시 매번 다른 위치와 yaw로 생성됩니다. 상자 양쪽 경계에서 2cm 이상 떨어진 **왼쪽(+Y) 또는 오른쪽(-Y) 구역**에서 생성합니다. 상자가 있는 중앙 띠에는 생성하지 않고, 몸통 쪽 테이블 끝을 기준으로 길이의 **1/4~2/3 구간**만 사용합니다. 가까운 첫 1/4과 먼 쪽 마지막 1/3에는 생성하지 않습니다. 회전한 큐브의 모서리까지 금지 구역과 테이블 가장자리를 침범하지 않게 배치합니다. 물리 동작 중 사용자가 왼쪽 구역으로 옮기는 것은 가능합니다.

테이블과 상자 중심은 이동 전의 `TABLE_CENTER=(0.55, 0, 0.28)`, `BOX_CENTER_XY=(0.55, 0)`로 복원했습니다. 큐브 생성도 당시와 같은 **영역 내 연속 랜덤 좌표와 yaw**를 사용합니다(`CUBE_SPAWN_REQUIRE_IK=False`). 큐브 전체가 들어갈 영역은 X=`0.415~0.635`m, 왼쪽 Y=`0.110~0.385`m 또는 오른쪽 Y=`-0.385~-0.110`m입니다. 실제 중심 범위는 회전한 큐브의 반폭만큼 더 좁아집니다. 이후 추가했던 수직 자세 기반 260개 IK 격자 및 카메라 시야 필터는 기본 생성에 사용하지 않습니다. 모든 생성 위치의 IK 도달 가능성을 보장하는 설정은 아닙니다. 이전 격자 검사 설명은 [TABLETOP_IK_SPAWN.md](TABLETOP_IK_SPAWN.md)에 남겨두었습니다.

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
| front / BODY | base_link | 640×480 | (.08, 0, .85) | (.906308, 0, .422618, 0) |
| left_wrist | left_dof7_link | 320×240 | (.055, -.03, 0) | (.379928, -.379928, .596368, -.596368) |
| right_wrist | right_dof7_link | 320×240 | (-.055, -.03, 0) | (.596368, .596368, -.379928, -.379928) |

Body는 테이블 이동 전처럼 전방 아래 50도, 높이 85cm·전방 8cm, focal length 18mm로 복원했습니다. wrist는 link의 -Y 집기 방향을 기준으로 camera 아래 25도입니다. 좌우 link가 mirror이므로 영상의 위쪽 방향도 각각 local +X/-X로 맞췄습니다. offset은 config에서 조정합니다. 실제 rigid link를 이름으로 찾아 그 아래 camera prim을 생성하므로 import directory나 joint numeric ordering에 의존하지 않습니다.

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

Display composite는 1280×720이며 기존 BODY(`front`) 원본 RGB 하나를 종횡비를 유지해 크게 표시합니다. 아래에는 조작·녹화 상태를 표시합니다. 추가 정면·측면 카메라는 생성하지 않습니다. **LeRobot과 정책 관측에는 기존 `front`, `left_wrist`, `right_wrist` 세 카메라만 사용**하며 mount·해상도·feature 이름은 유지합니다. resize와 상태 문구는 Quest 표시용 canvas에만 적용하고 원본 RGB는 변경하지 않습니다. JPEG 인코딩은 별도 thread에서 수행하며 최신 frame을 전송합니다.


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
2. 컨트롤러 XYZ를 고정한 채 각 축을 회전: 회전 입력이 관절 명령을 바꾸지 않음. XYZ 이동 중 로그는 `orientation=FREE tcp=GRASP_TIP`이며 그리퍼 기울기는 달라질 수 있음.
3. 오른손 trigger는 로봇 왼쪽 gripper, 왼손 trigger는 로봇 오른쪽 gripper를 조작. release=open / press=closed. `--gripper-input squeeze`도 같은 교차 대응.
4. Quest 기본 AR에서는 3D 로봇과 작업대를 표시. body·양손목 카메라 3개는 dataset에만 기존대로 저장됨.
5. controller sleep/브라우저 종료 → tracking timeout → Isaac HOLD. 다시 연결 → 현재 held pose 기준 재보정.
6. 어느 프로세스를 먼저 종료해도 다른 프로세스가 crash하지 않음. 종료 후 포트 재사용 가능.

일부 XR client가 tracking이 끊겨도 유효한 stale pose를 계속 송신한다면 서버는 새 pose와 구분할 수 없습니다. 이벤트 정지, invalid matrix, 명시적 `connected/tracked=False`는 검출합니다. 실제 Quest sleep/occlusion 동작은 hardware 검사 항목입니다.

## 구현 범위와 남은 실기 확인

control, camera, video IPC, LeRobot v3 에피소드 수집과 tabletop 장면이 구현되어 있습니다. 모델 학습은 별도 LeRobot π0.5 환경에서 수행합니다. 수집용 테이블·상자 접근 보조는 있지만, 모든 물체에 대한 충돌 회피나 실제 로봇 torque safety는 포함하지 않습니다. 이 프로그램은 Isaac simulation용입니다.

설치 버전에 특유한 카메라 Fabric 초기 pose 동기화와 STOP callback을 코드에 설명했습니다. 원본 package를 수정하지 않고 앱 초기화/종료 순서에서 처리합니다. Quest 착용 상태의 XYZ 조작감, LAN 인증서 신뢰, headset browser XR 진입, 실제 controller tracking loss, 사용자에게 편한 wrist 시야는 현장 확인이 필요합니다. 자세한 자동 실행 결과는 `VALIDATION_TELEOP.md`에 기록합니다.

로봇 외형 색상은 `robot/appearance.py`의 `PALETTE`에서 관리합니다. 받침 기둥과 중앙 상단 부품을 포함한 로봇 전체는 팔과 같은 블랙, 그리퍼 손가락은 파란색 `#2F90CE`입니다. Isaac에서는 링크별 재질을 적용하며 URDF와 가동범위 HTML에도 같은 색 구성을 반영했습니다.

녹화 시작 A를 누를 때마다 해당 버튼 입력 프레임의 양손 XYZ를 새 중립 기준으로 잡고 이동 필터를 초기화합니다. 대기 중 손을 옮겨도 이전 에피소드의 기준을 사용하지 않습니다. Quest 로그의 `New episode neutral captured at A press`로 매 시작의 기준 갱신을 확인할 수 있습니다. REVIEW에서 저장하는 A는 기준을 바꾸지 않습니다.

수집한 데이터로 로컬 OpenPI π0.5 LoRA 학습을 시작하는 명령과 준비 결과는 [OPENPI_TRAINING.md](OPENPI_TRAINING.md)를 참고하세요.

### 컨트롤러 축 대응과 AR 기본 배치

물리 왼손 −Y → 로봇 오른팔 X, 왼손 X → 오른팔 Z입니다.
물리 오른손 Y → 로봇 왼팔 X, 오른손 X → 왼팔 −Z입니다.
컨트롤러에는 X/Y, 그리퍼에는 X/Z 점선을 표시하고 이 대응으로
각각 10° 이내인지 판정합니다. 위치 허용 오차는 1cm입니다.
XYZ 이동은 고정 공간 기준이며 A 순간 기준의 회전 변화량을 추종합니다.

AR 장면 전체는 실제 바닥보다 80cm 높고 기본 yaw는 180°입니다.
`config/ar_config.py`의 `PLACEMENT_HEIGHT`/`PLACEMENT_YAW_DEG`로 조정합니다.
로봇·테이블·상자가 함께 이동하며 Isaac 환경·LeRobot 카메라에는 적용되지 않습니다.

Quest 표시 축 이름은 A/B입니다(컨트롤러 Y↔그리퍼 X가 A, 컨트롤러 X↔그리퍼 Z가 B). 점선은 각 축의 양방향으로 5cm씩, 총 10cm이며 이름은 한쪽에만 표시합니다. 기존 좌우 방향 대응은 유지합니다.

현재 테이블·박스 접근 보조는 비활성화되어 있습니다(`config/table_assist_config.py: ENABLED=False`). 접근 감속·목표 보정·충돌 QP 제약을 적용하지 않습니다. 실제 물리 충돌과 관절 한계·속도·가속도 제한은 유지합니다.

현재 A 시작 조건은 양손 대응 TCP 거리 **4cm 이내**입니다. 각도는 시작 조건에서 제외했으며 A/B 축은 참고용으로 표시합니다. A 순간 자세를 새 기준으로 캡처하는 회전 추종은 유지합니다. AR 기본 생성 높이는 **70cm**입니다.

컨트롤러 위치 변화량은 AR 배치 행렬의 역변환으로 로봇 좌표계에 옮깁니다. 추가 부호 반전은 하지 않습니다(`TRANSLATION_AXIS_SIGNS=(1,1,1)`). 따라서 화면에서 손과 로봇 목표점의 이동 방향이 같으며, 180° 배치와 회전 축 대응·A 순간 기준 캡처도 유지합니다.


### 빠른 IK 추종 설정

`config/teleop_config.py`에서 위치/회전 추종 이득 10/s, 관절 속도
3.14rad/s, 가속도 10rad/s², TCP 목표 속도 0.75m/s를 사용합니다.
입력 위치/회전 필터 alpha는 0.8이며 위치 deadband는 1mm입니다.
추종 예측 시간은 0.5초에서 0.02초로 줄였고 실제 제어 dt보다 작게
예측하지 않습니다. 실제 구동과 reference의 오차 제한 0.05rad 및
해당 경계까지의 정지 거리는 유지합니다. 회전 목적함수 가중치는 3에서
1로 줄였지만 XYZ+회전 목표 자체를 제거하지 않습니다.

고속 차분 IK의 비선형 회전 근사 오차는 이동 중 0.002rad(약 0.11°)
이내에서 허용하며, 정지 목표는 기존 0.2mm/0.0002rad 수렴 검증을 합니다.
관절 한계, 유한값, QP 제약과 timeout 정지는 유지합니다. 접근 보조는 꺼져 있습니다.

재현: `python scripts/benchmark_ik_response.py` (env_isaaclab).
[비교 결과](outputs/ik_diagnosis/fast_response_benchmark.json)는 현재 HOME의
FK로 만든 도달 가능한 두 목표를 이전/현재 설정으로 비교합니다. 실제
관절이 reference를 완벽하게 따라온다는 조건이므로 PhysX/렌더링 지연은
포함하지 않습니다. Isaac과 Quest 송신기를 모두 재시작해야 적용됩니다.
