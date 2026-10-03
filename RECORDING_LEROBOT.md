# F14 Quest → LeRobot v3 데이터 수집

오른손 A로 녹화 시작, 오른손 B로 정지, 정지 후 오른손 A로 저장하거나
왼손 X로 폐기합니다. 저장·폐기는 **현재 에피소드만 종료**합니다. 앱은
계속 실행되며, 팔을 천천히 HOME으로 복귀시키고 새 빨간 큐브를 배치한
뒤 다음 시연을 시작합니다.

**`--record`에서는 녹화 중에만 Quest 조작을 적용합니다.** A를 누르기
전에는 팔을 HOME, 그리퍼를 열린 상태로 유지합니다. 대기 중 손을 움직이거나
트리거를 눌러도 로봇은 따라가지 않습니다. A를 누른 순간의 컨트롤러 자세가
새 기준이 되므로, 대기 중 움직인 거리가 녹화 시작 때 적용되지 않습니다.

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
python scripts/run_isaac_teleop.py --device cuda:0 --record
```

터미널 2: 송신기를 실행하고 Quest에서 기존 주소를 새로고침합니다.

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
| READY, Quest 보정 완료·HOME 도달 | 오른손 A | HOME에서 녹화·teleoperation 시작 |
| RECORDING | 오른손 B | 녹화 정지, 팔 감속 정지, 그리퍼 명령 유지 |
| REVIEW | 오른손 A | 저장 확정과 HOME 복귀를 병렬 실행·새 큐브 |
| REVIEW | 왼손 X | 현재 시연 폐기 후 HOME 복귀·새 큐브 |
| SAVING / RESETTING | — | 저장 확정과 환경 초기화가 모두 끝날 때까지 대기 |
| 초기화 완료 | — | HOME에서 Quest 재보정, 이후 A로 다음 시연 |

Quest 영상 아래에 상태·프레임 수·저장한 에피소드 수와 버튼 안내를
표시합니다. **B는 저장하지 않습니다.** 버튼을 길게 누르거나 통신
재전송이 발생해도 동일 시연을 두 번 저장하지 않습니다. 연결 당시
누르고 있던 버튼은 한 번 놓은 뒤 눌러야 인식합니다.

첫 관측은 HOME에서 촬영한 뒤 조작을 허용합니다. 시작 시 실제 팔의 HOME
오차가 0.015rad 미만이고 그리퍼가 충분히 열렸는지 확인합니다. 물리 시뮬레이션의
작은 관절 오차는 실제 측정값으로 저장하며, 데이터의 첫 상태를 강제로 HOME
수치로 덮어쓰지는 않습니다. 일반 `--record` 없는 모드의 조작 방식은 유지합니다.

초기화는 그리퍼를 열어 물체를 놓고 기존 팔 속도·가속도 제한
(0.25rad/s, 0.5rad/s²)을 유지하며 HOME으로 돌아갑니다. 드라이브 목표
속도가 충분히 낮고 실제 HOME 위치가 0.5초 동안 안정되면 큐브의 위치,
yaw, 속도를 초기화합니다. 운용 중 로봇 관절 상태를 강제로 쓰거나
HOME으로 순간 이동시키지 않습니다. 이전 UDP 명령은 차단하고 새
환경에서 Quest를 다시 보정합니다.

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
| `task` | 작업 설명, LeRobot task metadata/index로 저장·복원 |

순서는 왼팔 dof1~7, 오른팔 dof1~7, 왼손·오른손 그리퍼입니다. 팔은
radians와 rev 2.0.1 URDF 부호를 사용합니다. **현재 USD/16D 프로토콜에서
그리퍼 0은 물리적 닫힘, 1은 열림**입니다. 기존 로그의 `semantic 0=open`
표기는 실제 USD와 반대입니다. 데이터는 현재 동작을 그대로 기록하고
`left/right_gripper.open_ratio`로 이름을 붙였습니다. 기존 트리거 조작은
변경하지 않았습니다.

물체 때문에 그리퍼 목표 0까지 닫히지 않아도 실제 상태와 명령을 각각
그대로 저장합니다. 이 차이를 IK 실패로 취급하거나 실제 상태를 목표값으로
덮지 않습니다. IK 실패로 팔이 감속하는 구간도 실제 동작으로 기록하므로
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

이는 **학습 입력 호환성 검사**입니다. 실제 OpenPI 학습을 시작하려면
이 F14 data config를 학습 설정에 등록하고 PyAV 지정, π0.5 사전 학습
checkpoint 경로, batch size 등을 설정해야 합니다. 제공한 검사 명령은
학습을 시작하거나 checkpoint를 다운로드하지 않습니다.

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
