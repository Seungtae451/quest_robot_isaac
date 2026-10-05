# F14 Quest → LeRobot v3 데이터 수집

기본 AR 모드에서는 **첫 오른손 A로 공간을 고정**한 뒤, 양손 컨트롤러를
**오른손→로봇 왼팔, 왼손→로봇 오른팔의 그리퍼 길이 중앙 TCP 반지름 1cm 구와 X·Y축 각각 10° 이내**에 가져가 **다음 A로 녹화 시작**합니다.
오른손 B로 정지, 정지 후 오른손 A로 저장하거나
왼손 X로 폐기합니다. 저장·폐기는 **현재 에피소드만 종료**합니다. 앱은
계속 실행되며, 팔을 천천히 HOME으로 복귀시키고 새 빨간 큐브를 배치한
뒤 다음 시연을 시작합니다.

**`--record`에서는 녹화 중에만 Quest 조작을 적용합니다.** A를 누르기
전에는 팔을 HOME, 그리퍼를 열린 상태로 유지합니다. 대기 중 손을 움직이거나
트리거를 눌러도 로봇은 따라가지 않습니다. 매 녹화 시작 A 버튼 이벤트에 포함된 양손 컨트롤러 자세를
새 기준으로 저장하고 필터도 초기화하므로, 대기 중 움직인 거리가 녹화 시작 때 적용되지 않습니다.

조작 입력은 **컨트롤러 XYZ·회전 변화량과 그리퍼 개폐**를 사용합니다. 그리퍼 길이 중앙을 TCP로 사용하며 수직 자세는 강제하지 않습니다. 오른손은 로봇 왼팔, 왼손은 로봇 오른팔을 조작하고 고정 공간에서 이동·회전 방향을 반전하지 않습니다. A 순간의 자세를 새 기준으로 잡아 시작 직후 이동·회전 점프를 방지합니다.

HOME·테이블·큐브 생성 범위는 그대로 유지합니다. `observation.state`와 `action`에는 **팔 관절 14개 + 그리퍼 2개 전부**를 각각 실제 측정값과 최종 구동 명령으로 저장합니다. body·좌우 손목 카메라 세 개만 저장하며 Quest의 점선 축·구·안내 UI는 저장하지 않습니다. 기존 데이터를 변환하거나 덮어쓰지 않습니다. 변경 적용 시 Isaac과 Quest 송신기를 모두 재시작하고 Quest 페이지를 새로 고침하세요.

## 실행

현재 PC에는 별도 저장 환경 `.venv-lerobot`을 설치했습니다. 새 PC에서는
`env_isaaclab`을 활성화한 뒤 다음 설치를 한 번 실행합니다. 기존 Isaac의
PyTorch/NumPy는 변경하지 않고 저장 프로세스에만 패키지를 추가합니다.

```bash
conda activate env_isaaclab
cd ~/stkim_ws/quest_robot_isaac
bash scripts/setup_recording_env.sh
```

터미널 1: 기존 Isaac을 종료한 뒤 `--record`로 다시 실행합니다.
`--headless`를 추가해도 실제 카메라 RGB는 저장됩니다.

```bash
python scripts/run_isaac_teleop.py --device cuda:0 --record --no-quest-video
```

AR 표시용 모델은 [README의 준비 명령](README_TELEOP.md#실행)으로 한 번 생성합니다.
터미널 2: 송신기를 실행하고 Quest Browser에서 `https://192.168.100.136:8012/`를 엽니다.

```bash
python scripts/run_quest_teleop.py --host-ip 192.168.100.136
```

기본 저장 위치는 **`datasets/f14_cube_pickplace`**, 작업 설명은
`Pick up the red cube and place it in the center box.`입니다. 설명은 모든
프레임에 LeRobot task로 연결됩니다. 다른 작업은 다음처럼 지정합니다.

```bash
python scripts/run_isaac_teleop.py --device cuda:0 --record \
  --dataset-root datasets/my_task \
  --repo-id local/my_task \
  --task "Your task description."
```

`--repo-id`는 로컬 식별자이며 Hub 업로드는 하지 않습니다. 같은 폴더로
재실행하면 기존 에피소드에 이어서 저장합니다. FPS·관절 이름·카메라
해상도가 다르면 새 폴더를 사용하세요. 동일 데이터셋에 두 프로세스가
동시에 쓰는 것은 차단합니다.

## 버튼과 초기화

| 상태 | 버튼 | 결과 |
|---|---|---|
| AR 공간 배치 중 | 오른손 A | 원점·방향 고정, 녹화하지 않음 |
| READY, HOME 도달·교차 대응 손이 각 그리퍼 중앙에서 1cm·X/Y축 각각 10° 이내 | 오른손 A | HOME에서 녹화·teleoperation 시작 |
| RECORDING | 오른손 B | 녹화 정지, 팔 감속 정지, 그리퍼 명령 유지 |
| REVIEW | 오른손 A | 저장 확정과 HOME 복귀를 병렬 실행·새 큐브 |
| REVIEW | 왼손 X | 현재 시연 폐기 후 HOME 복귀·새 큐브 |
| SAVING / RESETTING | — | 저장 확정과 환경 초기화가 모두 끝날 때까지 대기 |
| 초기화 완료 | — | 공간 배치는 유지, 그리퍼 끝에 교차 대응 손 연결 후 A로 다음 시연 |

Quest AR 안내판에 상태·프레임 수·저장한 에피소드 수와 버튼 안내를
표시합니다. **B는 저장하지 않습니다.** 버튼을 길게 누르거나 통신
재전송이 발생해도 동일 시연을 두 번 저장하지 않습니다. 연결 당시
누르고 있던 버튼은 한 번 놓은 뒤 눌러야 인식합니다.

첫 관측은 HOME에서 촬영한 뒤 조작을 허용합니다. 시작 시 실제 팔의 HOME
오차가 0.015rad 미만이고 그리퍼가 충분히 열렸는지 확인합니다. 물리 시뮬레이션의
작은 관절 오차는 실제 측정값으로 저장하며, 데이터의 첫 상태를 강제로 HOME
수치로 덮어쓰지는 않습니다. AR은 `--record`가 없어도 양손 연결 후 A를 눌러야
조작하며 B/X로 정지합니다. 기존 2D 자동 보정 모드는 `--xr-mode immersive`로 선택합니다.

초기화는 그리퍼를 열어 물체를 놓고 기존 팔 속도·가속도 제한
(0.25rad/s, 0.5rad/s²)을 유지하며 HOME으로 돌아갑니다. 드라이브 목표
속도가 충분히 낮고 실제 HOME 위치가 0.5초 동안 안정되면 큐브의 위치,
yaw, 속도를 초기화합니다. 운용 중 로봇 관절 상태를 강제로 쓰거나
HOME으로 순간 이동시키지 않습니다. 이전 UDP 명령은 차단하고 새
환경에서 다음 A 순간의 그리퍼 끝·대응 컨트롤러 기준을 다시 캡처합니다.
공간 원점과 방향은 에피소드마다 다시 잡지 않습니다.

AR은 별도의 측정 pose 스트림을 표시합니다. passthrough 배경, 사용자의 머리 시점,
컨트롤러 표시, 연결 구, UI는 **LeRobot 영상이나 state/action에 저장되지 않습니다**.
body(`front`)/좌우 손목 3개 카메라와 기존 state16/action16 정의는 동일합니다.

큐브는 기존 규칙인 중앙 상자 양쪽, 몸통 쪽 끝 기준 테이블 길이의
1/4~2/3 구간에서 생성합니다. `--scene-seed N`이면 첫 배치는 N,
다음 배치는 N+1, N+2입니다. 생략하면 매번 랜덤입니다.

## 데이터와 시간 정렬

공식 **LeRobot 0.4.4 `LeRobotDataset` API**로 **v3.0** 형식을 저장합니다.
기본 30FPS는 시뮬레이션 시간 기준입니다. 처리 지연으로 실제 시연
시간이 길어져도 시뮬레이션 프레임을 건너뛰거나 중복하지 않습니다.

| 키 | 내용 |
|---|---|
| `observation.images.front` | 본체 원본 RGB, 480×640×3 |
| `observation.images.left_wrist` | 왼손 손목 원본 RGB, 240×320×3 |
| `observation.images.right_wrist` | 오른손 손목 원본 RGB, 240×320×3 |
| `observation.state` | 실제 팔 관절 14개 + 실제 그리퍼 2값, float32 16D |
| `action` | 적용한 팔 드라이브 목표 14개 + 그리퍼 명령 2개, float32 16D |
| `sim_time` | 원본 관측의 시뮬레이션 시간, float32 1D |

Quest에는 기존 BODY(`front`) 카메라 하나만 표시합니다. 추가 정면·측면 카메라는 삭제했습니다. **LeRobot에는 BODY·양손목 원본 카메라 세 개를 기존 mount/해상도 그대로 녹화**하며, Quest 합성 화면과 상태 문구는 저장하지 않습니다.

| `task` | 작업 설명, LeRobot task metadata/index로 저장·복원 |

순서는 왼팔 dof1~7, 오른팔 dof1~7, 왼손·오른손 그리퍼입니다. 팔은
radians와 rev 2.0.1 URDF 부호를 사용합니다. **현재 USD/16D 프로토콜에서
그리퍼 0은 물리적 닫힘, 1은 열림**입니다. 기존 로그의 `semantic 0=open`
표기는 실제 USD와 반대입니다. 데이터는 현재 동작을 그대로 기록하고
`left/right_gripper.open_ratio`로 이름을 붙였습니다. 기존 트리거 조작은
변경하지 않았습니다.

물체 때문에 그리퍼 목표 0까지 닫히지 않아도 실제 상태와 명령을 각각
그대로 저장합니다. 이 차이를 IK 실패로 취급하거나 실제 상태를 목표값으로
덮지 않습니다. IK 실패 시 축소한 성공 위치로 이동하거나, 가능한 해가 없어 감속하는 구간도 실제 동작으로 기록하므로
REVIEW에서 학습에 쓸 시연을 선택하세요.

관측 `o_t`는 **다음 카메라 간격 끝에서 실제 적용한 드라이브 목표**와
짝지었습니다. 30Hz의 다음 절대 관절 목표를 학습하며, 먼 IK 목적지나
이미지를 찍기 전에 실행한 과거 명령을 레이블로 사용하지 않습니다.
B를 누르면 다음 행동과 아직 짝짓지 않은 마지막 관측 한 장은 제외합니다.
추후 정책 재생도 이 시간 정렬과 관절 목표·속도 제한의 의미를 따라야 합니다.

별도 프로세스가 **녹화 중 원본 RGB 저장과 카메라별 H.264 MP4 인코딩을
병렬로 진행**합니다. 공식 LeRobot streaming encoder를 사용해 A=저장 후
전체 PNG를 다시 읽고 인코딩하던 작업을 제거했습니다. B는 영상 끝부분을
마무리하고 REVIEW로 넘어가며, A는 준비된 영상과 관절 데이터·통계를
최종 데이터셋에 확정합니다. X는 준비된 영상을 포함한 현재 임시 시연을
삭제합니다. 이전에 저장한 시연은 유지합니다.

저장 확정은 느린 HOME 복귀와 겹쳐 진행합니다. 저장이 먼저 끝나도 HOME
복귀·새 큐브 배치는 완료되어야 다음 A를 허용합니다. HOME이 먼저 완료되면
남은 저장 작업이 끝날 때까지 조작을 잠급니다. 긴 시연의 관절 통계·메타데이터
처리 시간은 남지만 전체 영상 재인코딩 대기는 없어집니다. Quest JPEG 전송,
글자 오버레이, 합성 화면은 학습에 쓰지 않습니다. 수치는 Parquet,
에피소드·task·통계는 `meta`에 저장됩니다. π0.5 기본 QUANTILES 정규화에
필요한 **q01/q99**도 생성합니다. 저장마다 `finalize()`와 공식 로더 읽기를
완료한 후 성공을 알립니다. [LeRobot v3 문서](https://huggingface.co/docs/lerobot/lerobot-dataset-v3)

```text
datasets/f14_cube_pickplace/
├── data/chunk-*/file-*.parquet
├── videos/observation.images.*/chunk-*/file-*.mp4
└── meta/
    ├── info.json
    ├── stats.json
    ├── tasks.parquet
    ├── episodes/chunk-*/file-*.parquet
    └── f14_sessions/<session_id>.json
```

## Quest 화면 거리

Quest에서 머리를 따라오는 영상 평면은 **1.5m 거리, 높이 1m**로 배치합니다.
이전 immersive 기본 거리는 1m였습니다. `config/teleop_config.py`의
`QUEST_SCREEN_DISTANCE`를 늘리면 더 멀어지고, `QUEST_SCREEN_HEIGHT`로 화면의
크기를 조절합니다. 값을 바꾼 뒤 Quest 송신기를 다시 실행하고 브라우저를
새로고침합니다. 이 설정은 로봇 본체·손목 카메라의 촬영 각도와 별개입니다.

## 확인과 π0.5 연결

저장한 뒤 영상 디코딩, 16D 상태·행동, 에피소드 경계, task, q01/q99와
정책 입력 feature 변환을 검사합니다.

```bash
.venv-lerobot/bin/python scripts/validate_lerobot_dataset.py \
  --root datasets/f14_cube_pickplace --repo-id local/f14_cube_pickplace
```

LeRobot π0.5는 짧은 상태/행동을 내부 최대 차원으로 패딩하고 카메라를
정책 해상도로 변환합니다. 이 데이터는 16D 상태/행동과 세 카메라를 그대로
읽도록 구성했습니다. 저장 환경은 데이터 처리용이며 모델 학습 패키지와
사전 학습 가중치는 설치·다운로드하지 않았습니다. 별도 학습 환경에서 사용할
시작 명령은 아래와 같습니다. 실제 모델 학습은 수행하지 않았습니다.
[π0.5 공식 학습 안내](https://huggingface.co/docs/lerobot/pi05)

```bash
lerobot-train \
  --dataset.repo_id=local/f14_cube_pickplace \
  --dataset.root=/home/cocelo-server01/stkim_ws/quest_robot_isaac/datasets/f14_cube_pickplace \
  --dataset.video_backend=pyav \
  --policy.type=pi05 \
  --policy.pretrained_path=lerobot/pi05_base \
  --policy.push_to_hub=false \
  --output_dir=outputs/pi05_f14
```

학습 환경에서는 모델/tokenizer 접근 권한과 GPU 메모리에 맞는 설정이
필요합니다.

### 가져온 OpenPI의 π0.5 입력 확인

`~/stkim_ws/openpi`의 commit `215abfb`는 LeRobot v2.1 및 datasets 3.x를
사용합니다. 수집한 v3 데이터를 기본 로더에 직접 연결하면 파일 경로와
Parquet feature metadata가 맞지 않습니다. 이 환경의 기본 TorchCodec도
FFmpeg shared library 오류를 내므로 확인 도구는 공식 PyAV 디코더를 지정합니다.

OpenPI 환경을 설치한 상태에서 프로젝트 폴더에서 아래 명령으로 확인합니다.

```bash
../openpi/.venv/bin/python scripts/check_openpi_dataset.py
```

검사는 CPU에서 실행하며 모델 가중치나 학습을 실행하지 않습니다. 첫 실행에
실제 작업 설명 토큰화를 위한 공개 PaliGemma tokenizer 약 4MB만
`outputs/openpi_cache`에 저장합니다. 원본 데이터와 OpenPI 설치 환경은
변경하지 않으며, 다음 결과를 `outputs/openpi_check/<run>`에 생성합니다.

- `lerobot/local/f14_cube_pickplace`: OpenPI v2.1 로더용 사본. 관절·행동 값은
  그대로 유지하고, 영상은 재인코딩 없이 복사합니다. 현재 F14 저장기의
  시연별 MP4 구조를 사용하며, 여러 시연이 공유하는 영상은 별도 분리가 필요합니다.
- `assets/f14_cube_pickplace/norm_stats.json`: 실제 수집 벡터에서 다시 계산한
  OpenPI 정규화 통계. `state`와 `actions` 각각 mean/std/q01/q99가 있습니다.
- `validation.json`: 버전·원본 보존·시연 경계·영상·작업 설명·모델 배치 검사 결과.

F14 매핑은 `dataset/openpi_f14.py`에 있습니다. 팔 관절 14개와 그리퍼 2개를
현재 순서·부호로 유지하고 **절대 드라이브 목표**로 학습합니다. Aloha의
14D 관절 변환이나 delta action mask를 적용하지 않습니다. 본체·양 손목
영상을 모델의 `base_0_rgb`, `left_wrist_0_rgb`, `right_wrist_0_rgb`에 연결하고,
공식 π0.5 전처리로 224×224 변환·작업 설명과 상태 토큰화·16→32D 패딩을 합니다.
정규화→패딩→역정규화 후 원래의 16D 행동이 복원되는지도 확인합니다.

2026-10-03 실제 수집 데이터 검증: **2개 시연, 1,184프레임, 30FPS**.
시연 시작·중간·끝 6곳에서 50프레임 행동 시퀀스가 해당 시연 안에서만
읽히는 것을 확인했고, 실제 OpenPI 학습 로더가 상태 `[2,32]`, 행동
`[2,50,32]`, 세 RGB `[2,224,224,3]`, prompt tokens `[2,200]` 배치를
생성했습니다. 원본 파일의 SHA-256도 검사 전후 모두 같았습니다.
결과: [validation.json](outputs/openpi_check/20261003_213326/validation.json).

기존 검사 명령은 **학습 입력 호환성 검사**이며 학습이나 checkpoint 다운로드를 실행하지 않습니다. 실제 F14 π0.5 LoRA 학습용 실행기는 `scripts/train_openpi_f14.py`에 추가했습니다. 100개 에피소드의 준비된 사본, 두 GPU 설정과 실행 명령은 [OPENPI_TRAINING.md](OPENPI_TRAINING.md)를 참고하세요.

## 실패·종료와 테스트

녹화 중 tracking/명령이 끊기면 자동 REVIEW로 넘어갑니다. 저장 큐가
가득 차거나 I/O가 실패하면 ERROR를 표시하고 불완전한 시연을 성공으로
저장하지 않습니다. 살아 있는 저장 프로세스에서는 X로 현재 시연을
폐기할 수 있습니다. HOME 경로가 막히면 강제로 초기화하지 않고 멈춥니다.

Ctrl+C는 프로그램 종료이며 **저장 승인으로 취급하지 않습니다**. 미확정
시연은 데이터셋 옆 `.<dataset_name>_pending/<session_id>` 임시 폴더에
남습니다. X로 폐기한 시연은 임시 폴더도 삭제합니다. 저장 성공 전에
실패하면 현재 원본 시연을 유지하며, 일반적인 저장 예외는 기존 메타데이터로
복구합니다. 상세 로그는 `outputs/recording/writer_*.log`에 남습니다.

```bash
python -m pytest -q
python scripts/test_recording_pipeline.py
```

전체 테스트는 실제 Isaac 물리·RGB 카메라와 실제 Quest WSS 서버를 사용하고
컨트롤러/버튼 입력만 합성합니다. 녹화·B 정지·A 저장·느린 HOME·새 큐브·
다음 시연 X 폐기·공식 LeRobot 읽기를 검사합니다. 물리 Quest 버튼의 UI
확인은 녹화 모드에서 직접 수행해야 합니다. 결과는
`outputs/recording_pipeline_check/<run>/validation.json`에 저장됩니다.

2026-10-03 검증 결과: 회귀 테스트 **55개 통과**. 실제 Isaac + 합성 Quest
버튼 테스트에서 **97프레임을 LeRobot v3로 저장·재읽기**했고, 저장과 폐기
각각의 HOME 복귀·새 큐브 배치, 기존 저장 데이터 유지가 통과했습니다.
공식 저장 API가 에피소드를 쓴 직후 예외를 발생시키는 테스트에서도 기존
영상·Parquet·메타데이터의 바이트가 모두 보존됨을 확인했습니다.
결과: [validation.json](outputs/recording_pipeline_check/20261003_191923/validation.json).

추가 검증: 회귀 테스트 **64개 통과**. A 이전 팔·그리퍼 HOME 유지,
A 시점의 새 컨트롤러 기준, 두 시연의
첫 상태 일치, 저장 전에 세 영상 인코딩 완료, 저장과 HOME 복귀 병렬 실행을
실제 Isaac에서 확인했습니다. 105프레임의 최종 저장은 약 **0.31초** 걸렸습니다.
이는 영상 인코딩을 녹화 중에 마친 뒤의 저장 시간이며 HOME 복귀 시간은 별도입니다.
결과: [validation.json](outputs/recording_pipeline_check/20261003_202942/validation.json).

## IK 범위 안의 새 큐브 배치

새 에피소드의 큐브는 `config/tabletop_config.py`의 현재 생성 범위에서 선택합니다. HOME·테이블은 이동 전 설정을 유지하며 카메라 설정은 `config/teleop_config.py`를 따릅니다. A 이전 HOME 유지, 그리퍼 중앙 XYZ+회전 조작, 16D 전체 관절 state/action, 세 카메라 저장 형식은 동일합니다. [스폰 조건](TABLETOP_IK_SPAWN.md)을 참고하세요.

## 데이터 수집 전용 테이블·상자 접근 보조

Isaac과 Quest 송신기를 재시작하면 `--record` 녹화 중에만 보조 기능이 기본으로 동작합니다. 7번 링크와 실제 그리퍼 충돌 mesh의 위치·자세를 읽고, 손가락 끝을 포함한 전체 그리퍼의 테이블 간격과 약 0.5초 뒤 하강을 예측합니다. PhysX의 순간적인 constraint velocity 잡음을 피하려고 실제 collider 위치 변화로 속도를 추정하고 짧게 필터링하며, link-7의 실제 linear/angular velocity도 진단 feedback에 포함합니다.

예측 결과는 별도 UDP 진단 메시지로 Quest 송신기에 전달합니다. 테이블 위로 내려가는 XYZ 목표의 Z를 IK 계산 전에 제한하고, 그리퍼 중앙 TCP XYZ+회전 IK와 관절 속도/가속도 제한으로 움직입니다. 위험할 때 먼저 감속/높이 유지하도록 요청하고, 설정 간격 아래로 이미 내려간 경우에만 위쪽 복귀 목표를 만듭니다. 테이블 밖에서 낮게 움직이는 목표에는 바닥 평면 제한을 적용하지 않습니다.

추가로 **테이블 상판과 상자 바닥·네 벽을 각각 충돌 금지 물체**로 검사합니다. 7번 링크와 양쪽 손가락의 충돌 경계를 각각 사용하며, 현재 그리퍼 끝 TCP에서 목표까지의 경로와 회전에 따른 손가락 경계를 검사합니다. 목표가 벽 반대쪽의 빈 공간이어도 경로가 벽을 통과하면 먼저 만나는 벽 앞에서 XYZ 이동을 제한합니다. 접근 속도에 따른 0.5초 여유도 반영합니다. 상자 윗면은 열린 상태로 유지하므로, **먼저 그리퍼 전체를 벽보다 높게 올린 뒤 상자 안으로 이동하고 내려가는 경로**는 허용합니다. 벽에 막히면 위로 올리거나 돌아서 조작하세요. 자동으로 우회 경로를 계획하지 않습니다. 실제 그리퍼 기울기의 경계를 사용하고 QP에서 회전 속도에 따른 모서리 이동도 제한합니다. 그리퍼 버튼과 데이터 형식은 그대로 사용합니다.

- 기본 최소 간격: **2mm**. 예측 시간: **0.5초**. 설정은 `config/table_assist_config.py`입니다.
- Quest 표시의 `TABLE/BOX ASSIST L T…/B…mm`에서 T는 테이블, B는 상자까지의 간격입니다. `BRAKE`는 예측상 테이블 접근 위험을 뜻합니다. Quest 터미널의 `TABLE/BOX LIMIT L/R`은 실제 XYZ 목표 제한입니다. 이 표시는 합성 화면에만 추가하고 녹화 RGB에는 넣지 않습니다.
- HOME 대기·리뷰·리셋 및 일반 `--record` 없는 실행에서는 꺼집니다. 수집 중 끄려면 Isaac 실행에 `--no-table-assist`를 추가하세요.
- 녹화 중 진단 feedback이 0.2초 이상 끊기거나 Isaac 재시작 전 정보이면 팔을 감속/유지합니다. 이때 Quest 터미널에 `TABLE/BOX ASSIST WAITING: braking arms`가 표시됩니다.
- inference 서버와 100회 평가기는 이 기능을 호출하지 않습니다. 정책 출력에 수집 보조를 추가하지 않습니다.
- 외력·보조 torque·impedance를 주입하지 않습니다. **보정 후 실제 적용한 16D action과 실제 측정 16D state**를 기존 LeRobot 형식으로 기록합니다. 숫자는 보정된 실제 동작을 반영하지만, 원래 명령값이나 가상 관절값으로 바꿔 저장하지 않습니다. 별도 보조 힘/플래그를 학습 feature에 추가하지 않습니다.

선형 속도 예측과 보수적인 mesh 경계에 기반한 XYZ 조작 보조이므로 모든 충돌을 보장하여 막는 기능은 아닙니다. 큰 추적 오차나 자세 변화가 있으면 간격을 확인하며 조작하세요. 팔 전체의 관절 경로, 다른 팔과의 충돌, 상자 안에서 손가락을 벌리는 동작은 이 XYZ 경로 제한으로 보호하지 않습니다. 큐브는 잡아야 하는 물체이므로 충돌 금지 대상으로 넣지 않습니다.

독립 물리 검사(Quest/녹화기 없이 테이블 아래 목표와 상자 벽을 통과하는 목표를 요청):

```bash
python scripts/test_table_assist.py --headless --device cuda:0
```

검사 결과는 `outputs/table_box_assist_check/validation.json`에 저장합니다.

2026-10-04 실제 Isaac 검증: 테이블 아래로 10cm 내려가는 목표를 720번 요청했을 때 IK 실패 없이 양손을 제한했고, 최소 간격은 각각 **2.20mm / 2.21mm**, 네 손가락과 테이블 사이의 최대 접촉 힘은 모두 **0N**이었습니다. [물리 검사 결과](outputs/table_assist_check/validation.json). 녹화·저장·HOME 복귀·새 큐브·X 폐기 통합 검사에서도 **362프레임을 공식 LeRobot API로 재읽기**했습니다. [녹화 통합 검사 결과](outputs/recording_pipeline_check/20261004_194017/validation.json). 회귀 테스트 44개도 통과했습니다. 이 결과는 해당 검사 경로에 대한 확인이며 모든 조작 상황의 무접촉을 보장하지 않습니다.

상자 경로 제한 추가 검증: 실제 Isaac에서 닫힌 그리퍼로 **1,440 physics step** 동안 테이블 하강과 벽 통과 목표를 요청했습니다. 양손은 상자 쪽으로 약 39.7mm 이동한 뒤 벽 앞에서 제한됐고, 테이블·바닥·네 벽에 대한 네 손가락의 접촉 힘은 모두 **0N**, IK 실패는 **0회**였습니다. [테이블·상자 물리 검사](outputs/table_box_assist_check/validation.json). 별도 녹화 통합 검사에서는 **345프레임** 저장·공식 재읽기, 저장/폐기 후 HOME·새 큐브 복귀가 통과했습니다. [통합 검사](outputs/recording_pipeline_check/20261004_200012/validation.json). 벽 반대쪽의 안전한 목표까지 가는 경로 차단, 열린 윗면으로 접근, 상자 바닥 제한, 테이블 모서리 진입, 뒤로 물러나기와 inference 제외를 포함한 회귀 검사 **47개**도 통과했습니다.
