# 실행 검증 기록

검증일: 2026-10-03 KST. 사용자 workspace의 실제 `env_isaaclab`, Isaac Sim 5.1.0, Isaac Lab git tag v2.3.2, RTX 5080을 사용했습니다. 물리 Quest는 연결하지 않았습니다. WSS controller 입력은 합성 데이터입니다.

## 최신: XYZ 입력과 수직 그리퍼 (2026-10-03 23:45 KST)

현재 운용 모드는 컨트롤러 XYZ와 개폐만 사용하며, 양쪽 EE 방향은 고정된
수직 아래 목표로 IK를 계산합니다. 아래의 이전 6DoF 결과는 변경 전 기록입니다.

- 전체 회귀 검사 **69 passed**, standalone 양팔 +5cm X / 수직 방향 IK 통과.
- 새 HOME 손목 `(0.40, ±0.18, 0.57)m`에서 실제 물리 드라이브·열린 그리퍼가
  안정적으로 유지됨. 낮은 이전 HOME에서는 수직 손가락이 테이블과 충돌했으므로
  새 HOME과 관절 한계에 여유가 있는 고정 yaw/IK 자세를 사용함.
- 실제 Isaac + 격리 WSS 18012의 양손 서로 다른 회전 입력으로 팔이 움직이지 않음.
- XYZ 이동 후 측정 EE의 전체 방향 오차 좌/우 **0.05245 / 0.05274도**.
  시험 중 손가락 전방 축의 수직 대비 최대 기울기는 **0.7486도**. 모든 IK 목표는
  정확한 고정 방향이며, 물리 추종과 관절 경로 중간 자세에는 작은 오차가 있음.
- A 이전 HOME 유지, A 순간 위치 기준, B 정지, A 저장/X 폐기, 느린 HOME 복귀,
  새 큐브, 다음 에피소드 동일 시작 상태, 저장/초기화 병렬 처리 모두 통과.
- 135프레임을 공식 LeRobot v3로 저장·재로딩. 팔 14개와 그리퍼 2개의 실제 state와
  적용 action, RGB 세 스트림 및 task 기록 형식 유지. 기존 수집 데이터는 변경하지 않음.

근거: [검증 JSON](outputs/recording_pipeline_check/20261003_234507/validation.json),
[공식 LeRobot 재로딩](outputs/recording_pipeline_check/20261003_234507/reload.log),
[Isaac 로그](outputs/recording_pipeline_check/20261003_234507/isaac.log),
[Quest 로그](outputs/recording_pipeline_check/20261003_234507/quest.log).

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

## 2026-10-04: IK 스폰 제한·가까운 테이블·바디 카메라

- 테이블 중심 X=0.45m, 수직 고정 Pinocchio 100회 IK로 접근·집기·들기·상자에 놓기 경로를 검사한 좌우 각각 130개 후보. `outputs/arm_workspace/ik_spawn_pool.json`, `ik_spawn_coverage.png`, 갱신한 `workspace.html`.
- 초기 팔의 가림은 후보 제외 조건이 아님. 전체 큐브의 이상적인 카메라 시야와 관절 한계 여유·경로 연속성 검사. 충돌 계획은 포함하지 않음.
- Body mount `(0.25,0,0.95)`, 아래 73도, focal length 16mm. 실제 GPU RGB 네 배치와 HOME 유지 검사 통과: `outputs/ik_spawn_camera_check/verified_layout/validation.json`.
- 전체 pytest **74 passed**, compileall 및 git diff --check 통과.
- 실제 Isaac/Quest WSS/공식 LeRobot 재로드 통합 검사 통과: `outputs/recording_pipeline_check/20261004_012555/validation.json`. 125프레임, 세 영상 병렬 인코딩, A 이전 HOME, 느린 HOME 복귀, 저장/폐기, 새 큐브 두 배치, 동일 시작 상태 검증. 기존 사용자 데이터는 수정하지 않음.

## 2026-10-04: 매 녹화 시작 A 입력 프레임으로 중립 기준 갱신

버튼 이벤트 큐에 해당 프레임의 양손 pose를 함께 보관합니다. 매 READY→A 시작에서 그 pose로 mapper와 필터를 새로 만들고 실제 로봇 위치를 anchor로 사용합니다. 다른 손 위치로 두 번째 A를 누른 뒤 유지해도 HOME이 유지되는 실제 Isaac/합성 Quest WSS/공식 LeRobot 재로드 검사 통과: `outputs/recording_pipeline_check/20261004_030121/validation.json`의 `every_a_press_recaptures_neutral=true`. 139프레임 저장, 시작 상태 일치, 저장·폐기·리셋 검증. 전체 pytest 75 passed. 실제 헤드셋 입력은 이 자동 검사에 포함하지 않습니다.

## 2026-10-04: IK 실패 시 가까운 성공 위치로 추종

기존 DLS를 유지하고 `robot/reachable_ik.py`에서 현재 측정 FK 기준 최대 4cm 방향 이동을 검사합니다. 실패/0.20rad 초과 관절 해는 이동량을 절반으로 줄여 최대 네 번 재탐색합니다. 유효한 해만 관절 한계·full-pose FK를 검증해 전송합니다. 탐색 시작 제한 12ms(진행 중인 solve를 중단하는 hard deadline은 아님), 재시도 최대 50회. 실제 속도/가속도 제한은 기존 receiver 그대로입니다. 양팔은 공통 이동 비율로 검사하며 유효한 단계가 없을 때만 함께 제동합니다. 충돌 회피/전역 최단거리 투영은 아님.

- 전체 pytest **79 passed**: 실제 URDF의 도달 불가 요청→근처 성공 위치 이동→안쪽 복귀, 실패 partial q 거부, FK/수직 방향·관절 제한, 반복/시간 예산 확인.
- 실제 Isaac + 합성 Quest WSS + 공식 LeRobot 검사: `outputs/recording_pipeline_check/20261004_033240/validation.json`. 중립에서 +1m의 불가능한 요청에도 검증된 단계로 전진, 요청을 되돌리면 복귀. 수직 자세, 매 A 기준 갱신, 녹화·저장·폐기·느린 HOME 리셋 통과. 기존 데이터 수정 없음.
- 실제 경계 반복 계산 100회, P95 약 9.6ms / 최대 약 12.6ms(동일 PC, 실제 물리 드라이브 없이 seed 진행). 실시간 비용은 부하와 자세에 따라 달라짐.

## 2026-10-04: Pinocchio + ProxQP differential IK

Quest의 반복 DLS/공통 backtracking을 양팔 독립 QP 속도 제어로 교체했습니다.
ProxQP 0.7.2를 기존 env_isaaclab에 설치했고 Pinocchio 2.7.0/NumPy는 유지했습니다.
각 팔은 7개 속도 변수, 13개 이하의 제약 행을 사용합니다. 중복 충돌 분리면을
합치고 목적함수를 정규화하며, 변경된 Jacobian의 preconditioner를 갱신합니다.
최대 외부/내부 반복은 100/50회이고, 미수렴 시 시간 예산 안에서 한 번만
cold restart합니다. 모든 결과는 제약과 nonlinear FK로 다시 검증합니다.

- 관련 회귀 **111 passed**. 실제 URDF의 정밀도·속도·가속도·관절 범위,
  도달 불가 목표의 독립 팔 동작/후퇴, 막힌 측정 관절, 실패 결과 거부,
  작은 PhysX HOME 오차와 변하는 collider envelope, packet/timeout 검사 포함.
  openpi/GR00T 전용 의존성이 없는 Isaac 환경에서는 해당 별도 테스트 모듈을
  실행하지 않았습니다.
- 실제 GPU: 양팔 열린 HOME에서 **40.282mm** 상승, 이어서 1,440 physics step의
  상판 하강/상자 벽 접근. 최소 상판 간격 좌/우 **2.231/2.230mm**, 손가락의
  상판·상자 바닥·네 벽 접촉력 **0N**. 계산 시간 P50/P95/최대
  **1.386/1.485/5.623ms**. 초기 수직 오차 회복 중 22개 팔 결과는 nonlinear
  검증에서 거부되어 안전하게 감속했고, QP 자체 미수렴은 없었습니다.
  이 수치는 해당 테스트 경로에 한정되며 전체 작업 공간의 충돌 보증은 아닙니다.
- 실제 Isaac + 합성 Quest WSS + 공식 LeRobot 저장/재읽기 통과:
  **369프레임**, 컨트롤러 회전 무시, 불가능한 목표로 점진 이동 후 복귀,
  A 순간 기준 갱신, 저장/폐기/동일 HOME/새 큐브 검증. 측정 EE 방향 오차
  좌/우 **0.0325/0.0329도**, 이동 중 손가락 축 최대 기울기 **0.2391도**.
- 데이터는 기존 state16/action16 및 body(front)/좌우 wrist RGB 3개 형식입니다.
  사용자 데이터셋은 수정하지 않았고, 검증은 별도 출력 데이터셋에 저장했습니다.
  inference는 기존 JointMotionLimiter 기본 제어를 유지합니다.

근거: [GPU 결과](outputs/proxqp_check/validation.json),
[녹화 결과](outputs/recording_pipeline_check/20261004_230256/validation.json),
[공식 LeRobot 재읽기](outputs/recording_pipeline_check/20261004_230256/reload.log).
녹화 없는 운용 모드에서도 XYZ 추종·도달 불가 요청 후 복귀·timeout 감속·
Quest 재시작·Isaac 재시작/재보정·정상 종료를 확인했습니다:
[전체 연결 검증](outputs/pipeline_check/validation.json). 제어 tick 중앙값 59.6Hz,
합성 입력 30Hz에서 IK 처리 중앙값 28.8Hz였습니다.
실제 헤드셋 착용/무선망과 head 추적 상실에 따른 재보정은 별도 확인 대상입니다.

## 2026-10-04: 손 입력/실제 그리퍼 속도 진단

최근 실제 Quest runtime의 QP 활성 상태 로그 60개를 분석했습니다.
IK 계산 중앙값 약 1.352ms, 구간별 P95 최대 5.416ms,
제어 tick 중앙값 58.99Hz, 입력 중앙값 62.50Hz였습니다.
샘플된 QP 상태 120개는 SOLVED이고, 24개 상태 줄에서 TABLE/BOX LIMIT가
있었습니다. 이 로그는 초당 상태 샘플이므로 모든 solve 결과를 집계한 것은
아니며, 기존 로그에 XYZ 시계열이 없어 당시 손/로봇 속도는 복원할 수 없습니다.
분석 결과: `outputs/motion_speed_check/latest_runtime_summary.json`.

실제 URDF/QP로 양팔 10cm 상승 목표를 오프라인 비교했습니다. 측정 관절이
reference를 즉시 따라간다는 조건이며 충돌/물리 드라이브/렌더링은 제외했습니다.
1mm 이내 도달 시간은 현재 설정 3.167s, 관절 속도만 2배 3.167s,
가속도만 2배 3.117s, 추종 예측 시간 0.5→0.25s는 2.200s였습니다.
현재 설정의 최대 EE 속도는 이 경로에서 약 4.12cm/s였습니다.
오차가 0이어도 추종 제약이 관절 속도를 0.05/0.5=0.1rad/s로 제한하므로
ARM_MAX_VELOCITY만 올리면 해당 경로에서 속도가 바뀌지 않습니다.
실제 설정 변경 없이 별도 Python 프로세스 안에서만 값을 바꿨습니다.
비교 결과: `outputs/motion_speed_check/limits_ab_10cm.json`(4cm 비교도 별도 저장).

Quest 진단에 컨트롤러/필터 목표/보조 목표/QP reference/측정 FK의
벽시계 기준 속도(cm/s), IK 최대 시간, 전체 반복 시간을 추가했습니다.
Isaac 진단에는 physics Hz와 실시간 진행 비율(rtf)을 추가했습니다.
수치는 LeRobot state/action/image에 추가하지 않습니다. 관련 회귀 41개 통과,
속도 단위/중복 피드백/추적 끊김/보정 초기화 확인, 두 실행 파일 문법과
Quest --help 확인. 실제 손 속도 비교에는 두 프로세스를 재시작한 후 새 입력이 필요합니다.

## 2026-10-05: 고정 공간 Quest passthrough AR

기본 Quest 모드를 로컬 Three.js/WebXR AR로 변경했습니다. 원본 USD의
표시 메시를 링크 로컬 GLB로 내보내고, Isaac의 실제 측정 링크 19개와
큐브 pose를 별도 읽기 전용 스트림으로 표시합니다. 표시용 복사본은
135,707개 삼각형/약 3.31MB이며 원본 물리 모델은 변경하지 않습니다.
첫 A는 공간 배치만 고정하고, 다음 A는 양손 grip 추적점이 각각 측정 dof7
원점의 5cm 구 안에 있을 때만 시작합니다. 원점 배치는 에피소드 리셋 후에도
유지하고 매 시작 A의 입력 pose로 상대 XYZ 기준을 갱신합니다.

- 관련 Python 회귀 **89 passed**. 공간 변환·실제 미터 단위·회전 무시,
  첫 A 소모/버튼 edge, 5cm 연결 조건, 오래된 입력/모델 불일치 거부,
  reference reset/앵커 점프 재배치, 기존 IK·충돌 보조·녹화 형식 검사 포함.
- 실제 Three.js GLTFLoader로 GLB를 읽어 19개 링크 이름·geometry를 검사했고,
  클라이언트 좌표 변환과 xr-standard grip/button 매핑 검사도 통과했습니다.
- 실제 Isaac GPU + 합성 WebXR WSS 입력 + 공식 LeRobot 저장 검사 통과.
  공간 배치 A와 8cm 밖 시작 요청이 녹화를 시작하지 않고, 녹화 A 순간
  HOME이 유지됩니다. head 이벤트 없이 양팔 약 24mm 상승, B 정지/A 저장,
  X 폐기, HOME 복귀·새 큐브·공간 유지, 추적 상실 시 REVIEW로 정지하고
  자동 재개하지 않는 동작을 확인했습니다. `--no-quest-video` 상태에서도
  데이터 카메라는 정상 저장됩니다.
- 별도 검증 데이터셋 **1에피소드/52프레임/20fps**를 공식 LeRobot 로더로
  다시 읽었습니다. state16/action16과 body(front) 640×480, 좌우 wrist
  320×240 RGB만 존재하며 pi0.5 입력 feature 검사를 통과했습니다.
  기존 사용자 데이터셋은 수정하지 않았습니다.
- 실제 Chrome에서 HTTPS 페이지와 3D 장면을 확인했습니다. HTTP 200,
  JavaScript 오류 0개입니다. 데스크톱에서는 AR 미지원 안내를 표시합니다.

근거: [GPU·WSS 결과](outputs/quest_ar_validation/20261005_012340/result.json),
[LeRobot 재읽기](outputs/quest_ar_validation/20261005_012340/reload.log),
[브라우저 결과](outputs/quest_ar_validation/20261005_012340/browser.json),
[브라우저 화면](outputs/quest_ar_validation/20261005_012340/browser.png).

실제 Quest 3 착용 상태의 passthrough·양안 렌더링·공간 앵커·무선 지연은
아직 검증하지 않았습니다. Quest Browser에서 방 안 배치 고정, 양손 5cm
연결, 머리만 회전/이동했을 때 로봇 입력 불변, 저장 후 동일 배치 유지,
메뉴/추적 상실 시 정지를 확인해야 합니다. 실제 방의 가림·충돌과
브라우저 재시작 후 영구 앵커 복원은 구현 범위에 포함하지 않습니다.

자동 검사 재현:

```bash
conda activate env_isaaclab
cd ~/stkim_ws/quest_robot_isaac
python -m pytest -q tests/test_quest_ar.py
python scripts/test_ar_pipeline.py
```

두 번째 명령은 별도 포트와 새 검증 데이터셋을 사용하며 실제 Isaac GPU를
실행합니다. 실행 안내는 [README_TELEOP.md](README_TELEOP.md#실행)입니다.

## 2026-10-05: AR 녹화 시작 거부 진단

기존의 포괄적인 `AR start rejected` 문구를 A 순간의 양손 거리와 개별 실패
조건으로 분리했습니다. 현재 시작 가능 상태도 HUD/터미널/runtime.jsonl에
표시합니다. HOME의 관절 오차·reference 속도·그리퍼 열림·명령 freshness를
녹화 상태 응답에만 추가했고 LeRobot feature와 기존 시작 한계는 유지했습니다.
HUD는 거리 소수점 둘째 자리, 주황/초록/회색 구와 메시 안에서도 보이는
하늘색 grip 추적점을 표시합니다. A 거부 안내는 8초간 유지합니다.

- 관련 Python 검사 **60 passed**. A를 누른 뒤 손이 이동해도 판정과 안내는
  A 순간의 pose를 사용하고, 거리 초과와 HOME 미준비를 구분함을 확인했습니다.
  기존 녹화·버튼·제어 회귀 및 JavaScript/GLTFLoader 검사 통과.
- 실제 Isaac GPU/WSS 통합 검사 통과: 양손 **8.00cm**에서 HOME이 준비돼
  있어도 `LEFT_OUTSIDE_5CM, RIGHT_OUTSIDE_5CM`으로 거부됩니다. 연결과
  시작 준비가 모두 완료된 뒤 녹화 시작, 이동, 저장/폐기, HOME/새 큐브,
  추적 상실 정지를 확인했습니다. 별도 1에피소드/52프레임과 기존 3개 카메라.
- 에피소드 READY 직후 첫 HOME 유지 명령이 도착하기 전에는
  `HOME_COMMAND_STALE`이 잠시 표시될 수 있습니다. 자동 재시작이나 시작
  요청 예약은 하지 않으며, HUD의 서버 준비 완료 후 새 A를 누릅니다.

근거: [통합 결과](outputs/quest_ar_validation/20261005_015145/result.json).
실제 Quest 착용 상태의 시인성은 사용자가 확인해야 합니다.

## 2026-10-05: Quest에 URDF 원본 형상 전체 표시

표시 모델의 소스를 기존 USD에서 현재 rev 2.0.1 URDF로 변경했습니다.
기존 표시 모델은 원본 약 393만 삼각형을 약 14만으로 단순화했지만,
이제 19개 링크의 visual 20개, **3,931,282개 삼각형을 모두 보존**합니다.
base의 기둥/상부 바디 두 visual도 포함합니다. 위치와 법선이 같은
vertex만 합쳐 인덱싱하며 부품·면 제거, 메시 decimation은 하지 않습니다.
URDF visual origin/scale/색상을 적용하고 각 링크의 전체 visual을 Isaac
측정 링크 pose로 이동합니다. 제어와 녹화 feature 변경은 없습니다.

- Python **46 passed**: 작은 분리 부품·날카로운 모서리의 법선,
  visual origin/비균일 scale/반사 처리, AR/기존 제어 검사 포함.
- 실제 Three.js GLTFLoader로 20개 visual을 로드해 **각 원본 STL의
  모든 삼각형 vertex 좌표와 GLB의 인덱스 복원 좌표가 정확히 일치**함을 확인.
  19개 링크 이름과 base 두 부품의 공통 이동도 검사했습니다.
- 저장된 실제 Isaac HOME 장면과 URDF FK의 19개 링크 프레임을 비교했습니다.
  최대 위치 차이 0.316mm, 회전 차이 0.052도 이내로 표시 좌표계 호환을 확인했습니다.
  HOME 목표와 물리 측정값의 작은 추종 오차가 포함된 비교입니다.
- 실제 Chrome HTTPS 페이지: HTTP 200, JavaScript 오류 0개, visual 20개,
  3,931,282개 삼각형 로드 및 화면 확인. GLB 약 303MB를 gzip 약 82MB로
  전송하고 브라우저에서 복원합니다. 형상 압축/단순화는 하지 않습니다.

근거: [브라우저 결과](outputs/quest_ar_validation/full_urdf_view/browser.json),
[표시 화면](outputs/quest_ar_validation/full_urdf_view/browser.png),
[모델 명세](outputs/quest_ar/assets/manifest.json).
실제 Quest 3의 로딩 시간·양안 FPS는 아직 측정하지 않았습니다.
형상 재생성 후 Quest 송신기를 재시작하고 브라우저 페이지를 새로 고침하세요.
