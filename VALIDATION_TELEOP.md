# 실행 검증 기록

검증일: 2026-10-03 KST. 사용자 workspace의 실제 `env_isaaclab`, Isaac Sim 5.1.0, Isaac Lab git tag v2.3.2, RTX 5080을 사용했습니다. 물리 Quest는 연결하지 않았습니다. WSS controller 입력은 합성 데이터입니다.

## 결과

| 검사 | 결과 / 근거 |
|---|---|
| Python syntax, import, CLI | 통과: 새 모듈 compileall, Quest help, TeleVuer signature/SSL paths, Pinocchio/ZMQ imports |
| 수학/프로토콜/통신 regression | `python -m pytest -q`: **14 passed** |
| standalone dual-arm 6DoF IK | 양팔 +5cm X + .02rad 회전 수렴; LOCAL SE(3) residual < 2e-4 |
| 1:1 mapping | +.10 / +.20 / +.50 / +1.00m 목표 그대로 유지, 30 / 90 / 170도 회전 보존 |
| SO(3) | 비항등 neutral, ±179도 평균, filter steady state 검사 통과 |
| gripper | inverted 10→0 전 구간, 좌우 독립 mapping, measured inverse 검사 통과 |
| UDP | 80-byte packing, malformed/NaN/magic/range 검사, stale/future/out-of-order 폐기, 재시작, HOLD 통과 |
| JPEG/ZMQ | 실제 PUB/SUB loopback, RGB/BGR 색, thread 종료 통과 |
| TeleVuer WSS | 실제 TLS WebSocket handshake, controller/head 이벤트, image uplink, disconnect watchdog, shared image color 통과 |
| 실제 F14 GPU 실행 | canonical USD spawn, 18 joints / 19 bodies, 이름 기반 arm/finger/EE lookup 통과 |
| 실제 HOME FK | left **0.0335205mm**, right **0.0335729mm** 오차 |
| 실제 drive tracking | HOME arm 최대 약 .001132rad; gripper semantic 최대 오차 약 .008948 |
| 실제 gripper 독립 동작 | HOME/open → left close/right open → left open/right close 검사 통과 |
| 실제 RGB | BODY 480×640×3, 좌우 WRIST 각 240×320×3, uint8, nonblank 확인 |
| camera mounting | 실제 camera world position = EE transform × 설정 offset, 오차 <1mm 검사 통과 |
| wrist motion | 각 arm dof1 +.08rad 후 left camera 32.867mm / right 32.873mm 이동; BODY 0mm |
| compositor | 720×1280×3, BODY 중앙 크게, 좌우 WRIST; raw buffer 무변경 검사 통과 |
| 전체 프로세스 경로 | **PIPELINE TEST PASSED**, 실제 Isaac와 Quest main entrypoint 사용 |
| 종료/재시작 | 양쪽 SIGINT exit 0, sender 재시작 시 held pose 유지, Isaac 종료 후 Quest 유지, Isaac 재시작 감지/HOME 재보정 |

## 실제 출력 artifact

- [HOME composite](outputs/camera_check/quest_composite.png)
- [이동 후 composite](outputs/camera_check/quest_composite_moved.png)
- Raw: [BODY](outputs/camera_check/front.png), [LEFT WRIST](outputs/camera_check/left_wrist.png), [RIGHT WRIST](outputs/camera_check/right_wrist.png)
- [GPU 수치 결과](outputs/camera_check/validation.json)
- [실제 world camera / EE poses](outputs/camera_check/camera_poses.json)
- [전체 pipeline 수치 결과](outputs/pipeline_check/validation.json)
- [Isaac pipeline log](outputs/pipeline_check/isaac.log), [Quest pipeline log](outputs/pipeline_check/quest.log)

`outputs/`는 gitignore 대상이며 테스트를 다시 실행하면 재생성됩니다. 원본 robot assets는 수정하지 않았습니다.

## 전체 pipeline 테스트가 확인한 범위

`scripts/test_pipeline.py`가 실제 `run_isaac_teleop.py`와 `run_quest_teleop.py`를 실행합니다. 격리한 action/video 포트 15005/15556과 실제 TeleVuer WSS 8012를 사용했습니다.

1. 실제 Isaac 3-camera 영상의 ZMQ 수신 확인.
2. 합성 head/controller 이벤트로 calibration → 처음 HOME 유지.
3. 양 controller +.04m / .02rad, closure left=.8/right=.2 입력.
4. **Isaac가 수신한 command**의 FK 위치 residual 약 .1178mm/.1177mm, 회전 residual 약 6.00e-5/3.99e-5rad 확인. 이 수치는 command IK 정확도이며 PhysX measured EE 오차와 구별됩니다.
5. WSS image uplink와 Quest 로그의 실제 video 수신 확인.
6. WSS 연결 종료 후 Isaac HOLD 및 마지막 16D target 불변 확인.
7. Quest 프로세스를 종료/재시작하고 neutral을 30cm 바꾸어도 held robot target 불변 확인.
8. Isaac 종료 시 Quest가 계속 동작하고 WAITING FOR ISAAC 및 stale video 안내로 전환.
9. Quest가 살아 있는 동안 Isaac 재시작 → 새로운 boot 감지 → HOME 기준 재보정 → HOME/open 유지 확인.
10. 생성한 두 프로세스를 SIGINT로 종료하고 exit 0 확인.

기본 목표 주파수는 physics/control/camera 60/30/30Hz입니다. 이 실행에서는 camera 약 **27–29Hz**, Quest 수신도 대체로 **26–29Hz**였습니다. 로그의 video frame age는 주로 **10–50ms**였습니다. 이는 같은 PC에서의 capture→SUB age이고, headset 렌더링·무선망·사용자 체감 지연의 측정값은 아닙니다.

## 검증 중 발견해 반영한 수정

- 기존 송신/수신의 magic 불일치를 새 공통 `F16A`/16D 프로토콜로 해소했습니다. 기존 실험 스크립트와 사용자의 수정은 보존했습니다.
- 설치된 wrapper의 trigger encoding이 `10 - raw*10`임을 확인했습니다. 단일 sample 임계값으로 encoding을 바꾸지 않습니다.
- `motion_data_ready`가 latched flag인 것을 확인하고 실제 controller/head 이벤트 freshness를 shared monotonic timestamp로 판단합니다.
- 원본 USD finger 강성 2.3–2.8 N/m에서는 중력에 의해 한 finger씩 limit으로 이동해 half-closed state가 되었습니다. 런타임 finger stiffness=2000, damping=30으로 보정했습니다. 팔 gain/effort limit/asset 파일은 그대로입니다.
- Camera의 첫 Fabric pose 조회가 오래된 zero-joint USD pose로 world transform을 초기화했습니다. 첫 buffer/pose 조회를 **HOME write 이전**으로 옮겨 해결하고 실제 EE×offset 관계를 검사했습니다.
- Lab 2.3.2 STOP callback의 무한 rendering 대기를 피하도록 callback 해제 후 timeline을 중지합니다. 설치된 Kit에서 full extension teardown은 종료 시 segfault가 있었으므로, 앱 소유 sockets/threads/sensors를 먼저 정리한 뒤 **Kit 기본 fast shutdown**을 사용합니다. 최종 smoke/pipeline 종료는 exit 0으로 확인했습니다.
- SimulationApp의 기본 SIGINT handler는 Python `finally` 이전에 plugin을 unload하고, 단순 KeyboardInterrupt는 native physics callback에 잡혀 무시될 수 있습니다. signal handler는 종료 flag만 설정하고 main loop 경계에서 빠져나와 앱 자원을 먼저 정리한 뒤 Kit를 종료하도록 처리했습니다.
- 설치된 Vuer는 client disconnect 다음 image upsert에서 missing-session assertion을 발생시켰습니다. 해당 세션이 실제로 제거된 경우만 adapter에서 처리하고 다른 오류는 그대로 올립니다.

## 물리 Quest에서 남은 검사

- 현재 회사 LAN에서 Quest Browser HTTPS/WSS 및 XR 진입. **기존 인증서에 SAN이 없으므로** README의 현재 IP용 재발급/신뢰 절차를 확인해야 합니다. 인증서를 임의 교체하지 않았습니다.
- controller 실제 10/20cm 이동과 30도 회전의 물리 방향/축 체감. 코드의 numeric mapping은 검사했으며 wrapper head-yaw reference를 유지했습니다.
- 실제 trigger/squeeze와 controller sleep/occlusion 상태 신호.
- 착용 상태의 시야 크기, wrist offset, 무선망 포함 체감 지연.
- 카메라 영상은 실제 RGB임을 확인했지만, task object/collision avoidance/training dataset 작성은 이번 구현 범위가 아닙니다.

재현 명령과 architecture/좌표계 설명은 [README_TELEOP.md](README_TELEOP.md)를 참고하세요.
