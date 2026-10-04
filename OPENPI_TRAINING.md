# 수집한 F14 데이터로 OpenPI π0.5 학습

2026-10-04 확인: 원본 LeRobot v3 데이터는 100개 에피소드, 55,535프레임, 30FPS입니다. 로컬 OpenPI checkout은 `~/stkim_ws/openpi`의 `215abfb`입니다. F14용 실행기는 `scripts/train_openpi_f14.py`입니다. OpenPI 소스/설치 환경과 원본 데이터를 수정하지 않고 기존 JAX 학습 함수를 호출합니다.

현재 PC의 RTX 5080 16GB 두 장으로 **batch 2, FSDP 2에서 실제 학습 3 step과 저장, 이어서 재시작 1 step과 저장을 통과했습니다.** π0.5의 PaliGemma/행동 expert에 LoRA를 적용하고 EMA는 비활성화합니다. 전체 10,000 step 학습과 정책 성능은 아직 검증하지 않았습니다. 공식 [OpenPI 요구사항](https://github.com/Physical-Intelligence/openpi#requirements)은 단일 GPU 기준 LoRA >22.5GB, full fine-tuning >70GB이며 FSDP로 GPU당 메모리를 줄일 수 있다고 안내합니다.

## 지금 시작하기

이미 100개 전체의 학습용 사본과 정규화 통계를 준비했습니다. 별도 compute_norm_stats 명령은 필요 없습니다. Isaac/Quest는 종료한 상태에서 실행합니다.

```bash
cd ~/stkim_ws/quest_robot_isaac
CUDA_VISIBLE_DEVICES=0,1 ../openpi/.venv/bin/python scripts/train_openpi_f14.py \
  --exp-name f14_pi05_100eps \
  --batch-size 2 --steps 10000 --fsdp-devices 2
```

처음 학습할 때 `gs://openpi-assets/checkpoints/pi05_base/params` 가중치를 자동 다운로드합니다. 첫 JAX 컴파일에는 시간이 걸릴 수 있습니다. W&B는 기본 비활성화이며 `--wandb`로 켭니다. 위 설정은 시작 실험값이며 최적 학습량이나 정책 성공률을 보장하는 값은 아닙니다.

현재 머신에는 기본 가중치 다운로드와 체크섬 검증이 완료되어 있습니다. `Trying algorithm ... is taking a while`는 첫 컴파일의 cuDNN 알고리즘 선택 중 나올 수 있으며, 그 메시지만으로 학습 실패를 뜻하지는 않습니다.

반복되던 torchvision 영상 기능 지원 종료 경고만 실행기에서 숨깁니다. PyAV 디코딩은 그대로 사용하고 다른 경고와 오류는 표시합니다. 이미 실행 중인 학습에는 소스 변경이 즉시 적용되지 않으며, 다음 실행부터 적용됩니다.

Checkpoint 경로:

```text
outputs/openpi_training/checkpoints/pi05_f14_lora/f14_pi05_100eps/<step>/
```

1,000 step 간격과 학습 마지막에 저장합니다. 같은 실험을 이어서 학습하려면:

```bash
CUDA_VISIBLE_DEVICES=0,1 ../openpi/.venv/bin/python scripts/train_openpi_f14.py \
  --exp-name f14_pi05_100eps --batch-size 2 --steps 10000 --fsdp-devices 2 --resume
```

기존 실험을 덮어쓰지 않습니다. 새 실험은 다른 `--exp-name`을 사용하세요. 두 GPU를 사용하므로 짝수 batch를 사용합니다. 실패한 실행이 빈 폴더만 만든 경우에도 `--resume`을 붙이면 OpenPI가 체크포인트가 없음을 확인하고 기본 가중치에서 새로 시작합니다.

## GPU 시작 오류 수정

첫 실행 로그의 `NCCL ... Cuda failure 'out of memory'`는 데이터 로딩 이후, 첫 배치 이미지 미리보기에서 발생했습니다. 기존 실행기는 VRAM 85%를 선점하도록 설정했습니다. GPU 1은 화면 출력에도 사용하므로 NCCL용 메모리가 부족해질 수 있습니다.

실행기 기본값을 batch 2와 `XLA_PYTHON_CLIENT_PREALLOCATE=false`로 변경했습니다. W&B가 꺼져 있으면 첫 배치 미리보기를 건너뛰고, 켜져 있으면 전체 이미지를 CPU로 복사한 뒤 인덱싱합니다. 이 호환 처리는 실행 중에만 적용하며 OpenPI checkout을 수정하지 않습니다. 메모리 선점 설정의 의미는 [JAX 문서](https://docs.jax.dev/en/latest/gpu_memory_allocation.html)를 참고하세요.

추가 GPU 검사 도중 Isaac 녹화기가 다시 실행되어 두 GPU에 각각 약 2.4GB와 3.3GB를 사용했고, 모델 초기화와 gradient 계산에서도 메모리 부족이 발생했습니다. 실행기는 Isaac 수신기가 실행 중이면 학습을 시작하기 전에 종료 안내를 출력합니다. `--device cuda:0`이어도 Isaac 렌더링이 두 번째 GPU를 사용할 수 있으므로 반드시 Isaac을 종료하세요.

기본 OpenPI 초기화는 입력 가중치 전체를 각 GPU에 복제한 뒤 FSDP 상태로 분할합니다. 실행기에서는 업로드 시점부터 가중치를 FSDP로 나눠 담습니다. 학습 대상·정밀도·optimizer는 그대로이며 초기화 중 전체 가중치가 중복되는 메모리를 줄입니다.

Isaac 종료 후에는 기존 NCCL `2.26.2+cuda12.2`가 큰 GPU 통신에서 `illegal memory access`를 발생시키는 문제도 재현했습니다. 모델 없이 두 GPU에 분할한 1024×1024 행렬 계산도 실패했고, NCCL 2.31.2로 같은 검사가 정상 통과했습니다. [NVIDIA의 같은 버전/Blackwell 문제 보고](https://github.com/NVIDIA/nccl/issues/2418)도 확인했습니다.

현재 머신에는 NCCL 2.31.2를 `outputs/openpi_runtime`에 별도로 설치했습니다. 실행기는 CUDA를 가져오기 전에 해당 `libnccl.so.2`를 `LD_PRELOAD`로 지정해 한 번 재실행합니다. OpenPI `.venv`의 torch/JAX와 기존 NCCL 패키지는 변경하지 않습니다. 다른 머신이나 runtime 폴더를 삭제한 경우에는 한 번 설치하세요.

```bash
uv pip install --python ../openpi/.venv/bin/python \
  --target outputs/openpi_runtime 'nvidia-nccl-cu12==2.31.2'
```

공개 기본 가중치는 인증 없이 다운로드하고 모든 객체의 크기와 CRC32C(없으면 MD5)를 검증합니다. 불완전한 캐시 파일만 다시 받고, 검증한 임시 파일을 최종 파일로 교체합니다. 준비된 LeRobot 데이터와 정규화 통계는 그대로 사용합니다.

이전 터미널에서 메모리 관련 환경변수를 직접 설정했다면 제거하고 재시작하세요.

```bash
unset XLA_PYTHON_CLIENT_PREALLOCATE XLA_PYTHON_CLIENT_MEM_FRACTION
CUDA_VISIBLE_DEVICES=0,1 ../openpi/.venv/bin/python scripts/train_openpi_f14.py \
  --exp-name f14_pi05_100eps --batch-size 2 --steps 10000 --fsdp-devices 2 --resume
```

## 데이터 준비와 입력 검증

학습 사본은 `outputs/openpi_training/f14_cube_pickplace`입니다. v3→v2.1 사본을 만들고 영상은 재인코딩 없이 복사했습니다. 팔 14개와 좌우 그리퍼 2개를 기존 순서/부호의 절대 드라이브 목표로 사용합니다. Aloha의 관절 변환이나 delta action 변환을 사용하지 않습니다. 모델 내부에는 32D로 패딩합니다. Body/좌우 wrist 영상을 세 이미지 입력으로 사용합니다.

데이터 100개 전체를 내보내고 통계를 계산했으며, 골고루 선택한 10개 에피소드의 시작·중간·끝을 RGB와 행동 시퀀스 검증에 사용했습니다. 변경한 기본 batch 2로 입력 검사를 다시 통과했습니다: state `[2,32]`, actions `[2,50,32]`, 이미지 3개 각각 `[2,224,224,3]`. 이 입력 검사는 가중치나 gradient를 계산하지 않습니다.

```bash
../openpi/.venv/bin/python scripts/train_openpi_f14.py --check-only
```

검사 결과:

- `outputs/openpi_training/f14_cube_pickplace/validation.json`
- `outputs/openpi_training/f14_cube_pickplace/training_config_check.json`
- `outputs/openpi_training/gpu_training_check.json`: 실제 GPU 학습/저장/재시작 검사

데이터를 더 모은 뒤에는 **새 경로**로 새 사본을 준비하고 이후 모든 실행에 같은 경로를 지정합니다. 이미 준비된 사본은 자동 갱신하거나 덮어쓰지 않습니다.

```bash
../openpi/.venv/bin/python scripts/train_openpi_f14.py --prepare-only \
  --prepared-root outputs/openpi_training/f14_next

CUDA_VISIBLE_DEVICES=0,1 ../openpi/.venv/bin/python scripts/train_openpi_f14.py \
  --prepared-root outputs/openpi_training/f14_next \
  --exp-name f14_pi05_next --batch-size 2 --steps 10000 --fsdp-devices 2
```

## Open-loop 평가: GR00T N1.7 공식 프로토콜

공식 OpenPI checkout와 현재 upstream HEAD (`215abfb217dbac7d5f1273282331b9b1866c0479`)를 확인했으며, 녹화된 정답 action과 예측 action을 비교하는 전용 평가기는 없습니다. 따라서 [GR00T N1.7 공식 평가 코드](https://github.com/NVIDIA/Isaac-GR00T/blob/51d4c89f72fda44cbf77285c6a8114b52676b8a1/gr00t/eval/open_loop_eval.py)의 세 함수(`evaluate_single_trajectory`, `parse_action_gr00t`, `plot_trajectory_results`)를 **함수 본문 변경 없이** 사용합니다. 출처·commit·Apache-2.0 라이선스는 `third_party/gr00t_n17_eval`에 보존했습니다.

```bash
cd ~/stkim_ws/quest_robot_isaac
../openpi/.venv/bin/python scripts/open_loop_eval_f14.py \
  --model-path outputs/openpi_training/checkpoints/pi05_f14_lora/f14_pi05_100eps/40000 \
  --traj-ids 0 --steps 200 --execution-horizon 16
```

평가 절차는 GR00T와 같습니다.

1. 에피소드 처음부터 `min(steps, 에피소드 길이)`를 평가합니다. 기본 steps=200입니다.
2. 0, 16, 32, … 프레임에서 **녹화된 해당 시점의** 관측을 모델에 넣습니다. 예측 상태로 다음 관측을 생성하지 않습니다. 정답 action은 입력에서 제외합니다.
3. 매번 모델이 예측한 action chunk에서 앞 16개만 이어 붙입니다. 마지막 chunk는 실제 평가 길이에 맞춰 자릅니다. π0.5의 예측 chunk는 50이므로 execution horizon은 1~50입니다.
4. 역정규화된 원래 단위의 전체 시간×16 action에 대해 MSE와 MAE를 계산합니다. 팔 단위는 rad, 그리퍼는 open_ratio이며 도/퍼센트로 바꾸거나 클리핑하지 않습니다. 여러 에피소드의 요약은 에피소드별 지표의 단순 평균입니다.
5. 원본과 같은 세로 16개 subplot에 정답/예측과 빨간 추론 지점을 표시합니다. 원본 조건대로 전체 state와 평가 action의 shape이 같을 때만 실제 state 선도 표시합니다. 예를 들어 에피소드 일부만 평가하면 state 선은 생략됩니다.

출력은 `outputs/openpi_open_loop/<checkpoint>_<한국시간>`입니다. 에피소드마다 `traj_0.jpeg`, `traj_0.npz`(state/정답/예측), 전체 `metrics.json`이 생성됩니다. action 0~13은 왼팔 7개→오른팔 7개, 14~15는 좌우 그리퍼입니다. `metrics.json`의 action_names에서도 순서를 확인할 수 있습니다. 전체 에피소드는 `--steps 1000000`, 여러 에피소드는 `--traj-ids 0 1 2`로 선택합니다. `--save-plot-path <새 폴더>`로 저장 위치를 지정할 수 있습니다.

실제 40,000 step 체크포인트로 episode 0의 첫 200프레임을 CPU 평가했습니다. MSE `0.00003938077`, MAE `0.003454149`이며 `outputs/openpi_open_loop/40000_20261004_135845`에 저장했습니다. 세 함수 본문이 원본과 문자 단위로 동일한 것, 저장된 정답이 원본 데이터와 동일한 것, 지표 재계산과 마지막 chunk 절단을 확인했습니다(`validation.json`).

모델·데이터 연결 부분만 F14/OpenPI adapter로 대체했습니다. 모델은 OpenPI의 공식 `create_trained_policy`로 복원하고 체크포인트 정규화와 F14 변환을 그대로 사용합니다. π0.5의 기본 denoising 횟수 10을 유지합니다. GR00T 모델의 기본 4와는 다른 모델 설정이며 `--denoising-steps`로 변경할 수 있습니다. 영상은 검증된 PyAV로 읽습니다. `scripts/evaluate_openpi_f14.py`도 같은 새 평가기로 연결됩니다. 이전 단일 관측/50프레임 그래프는 이 새 평가 결과와 혼동하지 마세요.

기본 CPU 평가라 GPU 학습 중에도 실행할 수 있습니다. GPU 평가는 학습 종료 후 `--device gpu`를 추가합니다. 이 데이터는 이미 학습에 사용되었으므로 오차가 작아도 미관측 데이터 일반화나 pick-and-place 성공률을 의미하지 않습니다.

W&B에 에피소드별 비교 그림/MSE/MAE를 올리려면 로그인 후 평가 명령에 `--wandb`를 추가합니다. 별도 프로젝트 `f14-policy-evaluation`으로 기록합니다. 온라인 업로드는 아직 검증하지 않았습니다.

```bash
../openpi/.venv/bin/wandb login
```

## 학습 loss 그래프

현재 실행 중인 학습은 W&B와 로컬 scalar 기록을 켜기 전에 시작해서 과거 loss 전체를 복원할 수 없습니다. 실행 중인 학습은 그대로 유지합니다. **다음 시작/재시작부터** 실행기가 50 step마다 체크포인트 폴더의 `training_metrics.jsonl`에 loss, gradient norm 등의 이력을 추가합니다. W&B 없이도 아래 명령으로 그래프를 만들 수 있습니다.

```bash
../openpi/.venv/bin/python scripts/plot_openpi_training.py \
  outputs/openpi_training/checkpoints/pi05_f14_lora/f14_pi05_100eps/training_metrics.jsonl
```

W&B 학습 진행도도 보고 싶다면 다음에 학습을 이어 시작할 때 `--wandb`를 붙입니다. 이전 실행에 W&B ID가 없으면 새로운 W&B run을 만들고 모델은 기존 체크포인트에서 복원합니다. 과거 W&B 기록을 소급해서 만들지는 않습니다.

```bash
CUDA_VISIBLE_DEVICES=0,1 ../openpi/.venv/bin/python scripts/train_openpi_f14.py \
  --exp-name f14_pi05_100eps --batch-size 2 --steps 50000 --fsdp-devices 2 \
  --resume --wandb
```

## Isaac에서 100회 연속 inference / 성공률 평가

Quest 없이 학습된 모델이 로봇을 직접 제어합니다. 학습 때 사용한 `create_scene`, 테이블·상자·IK 검증 큐브 생성 영역, HOME, 카메라 3개와 실제 측정 관절 상태를 그대로 사용합니다. 모델의 절대 16D action은 기존 관절 이름/부호 변환을 통해 적용하며, 속도·가속도 제한을 유지합니다. 기록 데이터셋을 변경하거나 새 학습 데이터를 저장하지 않습니다.

학습과 기존 Isaac/Quest teleoperation을 종료한 뒤, **터미널 1**에서 정책 서버를 먼저 실행합니다.

```bash
cd ~/stkim_ws/quest_robot_isaac
CUDA_VISIBLE_DEVICES=0 ../openpi/.venv/bin/python scripts/serve_openpi_f14.py \
  --checkpoint latest
```

`latest`는 `f14_pi05_100eps` 실험 폴더에서 저장 완료 표시와 params가 있는 가장 큰 step을 선택합니다. 현재 마지막 체크포인트는 `49999`입니다. 서버 시작 후에는 선택한 모델을 고정하여 100회 내내 같은 모델을 평가합니다. 다른 실험은 `--experiment-dir <실험 폴더>`, 특정 모델은 `--checkpoint <체크포인트 경로>`를 지정합니다. 마지막 저장 모델이 최상의 모델이라는 의미는 아닙니다. OpenPI 공식 정책 복원 및 WebSocket 서버를 사용합니다.

`Policy ready at ws://127.0.0.1:8000` 출력 후 **터미널 2**에서 실행합니다.

```bash
conda activate env_isaaclab
cd ~/IsaacLab_2_3_2/IsaacLab
./isaaclab.sh -p ~/stkim_ws/quest_robot_isaac/scripts/run_isaac_policy_eval.py \
  --device cuda:0 --trials 100 --trial-timeout 60 --execution-horizon 16 \
  --scene-seed 0
```

GUI가 필요 없으면 `--headless`를 추가합니다. 실제 action의 주기는 학습 데이터와 같은 30Hz이고 physics는 60Hz입니다. 매번 예측한 50개 action 중 앞 16개를 적용한 뒤 새 카메라/관절 관측으로 추론합니다. 추론 응답을 기다리는 동안 physics는 정지합니다. 따라서 지연된 명령을 계속 실행하지 않으며, 시간 제한은 **시뮬레이션 시간** 기준입니다. 이 방식은 정책의 시뮬레이션 작업 성능을 평가하며 실제 로봇의 통신 지연을 포함한 실시간 평가와는 다릅니다.

성공/실패 및 리셋 규칙:

- 회전된 큐브의 전체 XY footprint가 상자 안쪽에 있고, **큐브와 상자 바닥 collider 사이에 실제 접촉 힘**이 발생하면 바로 성공입니다. 바닥 높이 근처인지도 확인합니다. 테이블·벽·그리퍼 접촉만으로 성공하지 않습니다. 그리퍼를 놓았는지는 성공 조건에 추가하지 않았습니다.
- 접촉 힘 임계값은 0.001N입니다(큐브 무게 약 0.196N). 잠깐의 접촉 대신 지속된 접촉을 요구하려면 `--success-hold 0.2`처럼 지정합니다. 기본값 0은 첫 접촉을 세는 요청한 방식입니다.
- 성공 또는 60초 timeout, 큐브 추락이 발생하면 해당 시도를 기록하고, 먼저 그리퍼를 열어 놓은 뒤 실제 관절 구동으로 HOME에 천천히 복귀합니다. 로봇 관절 상태를 순간이동시키지 않습니다.
- HOME 안정화 후 이전 큐브를 지우는 효과로 같은 rigid object를 새 랜덤 위치/회전에 reset하고 속도를 0으로 만듭니다. seed는 0, 1, 2, …로 진행하여 재현 가능합니다. 새 큐브가 가라앉은 뒤 다음 시도를 시작합니다. 마지막 시도 후에도 HOME으로 복귀하고 종료합니다.
- timeout과 추락은 성공률 분모에 포함합니다. 정책 서버 오류·사용자 중단·HOME 복귀 실패는 별도 오류로 기록하고 전체 실행을 중단합니다. 완료되지 않은 시도를 정상 실패로 섞지 않습니다.

평가를 실행한 **터미널 2에서 `f`를 누르면 Enter 없이 현재 시도를 수동 실패로 처리**합니다. `manual_failure`로 기록하고 성공률의 분모에 포함한 뒤 HOME 복귀와 다음 큐브 생성으로 진행합니다. 모델 추론을 기다리는 중에도 처리하며, 취소한 시도의 예측은 다음 시도에 적용하지 않습니다. HOME 복귀/큐브 생성 중 누른 `f`는 무시합니다. Ctrl+C는 전체 평가 중단입니다.

출력은 프로젝트의 `outputs/openpi_closed_loop/<한국시간>/`입니다. 각 시도 완료 시 저장하므로 Ctrl+C 후에도 완료된 결과가 남습니다.

- `trials.jsonl`: 시도별 성공/실패 원인, 생성 seed/위치/회전, 초기 실제 관절 상태, 소요 sim 시간, 최종 큐브 위치, 바닥 접촉 힘, 추론 지연, 제한된 action 프레임 수.
- `summary.json`: 완료 횟수, 성공·실패 수, **success_rate_percent**, 95% Wilson 신뢰구간, 원인별 횟수, 평균 성공 시간, 전체 wall 시간, 모델·환경 설정. 완료 100회라면 성공률은 `성공 횟수 / 100 × 100`입니다.

100회 전에 가볍게 시험하려면 `--trials 2`로 실행하세요. 실제 접촉 센서만 검사하는 모드도 있습니다. 정책 서버 없이 실행하며 학습 모델의 성능 측정은 하지 않습니다.

```bash
./isaaclab.sh -p ~/stkim_ws/quest_robot_isaac/scripts/run_isaac_policy_eval.py \
  --device cuda:0 --headless --smoke-test
```

공중/테이블/상자 바닥의 세 실제 PhysX 접촉 검사를 통과했습니다. 정적 바닥의 contact filter는 Xform 대신 실제 collider `/World/CollectionBox/bottom/geometry/mesh`를 사용합니다. [Isaac Lab contact sensor](https://isaac-sim.github.io/IsaacLab/main/source/overview/core-concepts/sensors/contact_sensor.html)와 설치된 Lab 2.3.2의 정적 ground contact 테스트 방식을 따릅니다.

추가로 마지막 체크포인트 `49999`를 실제 GPU 정책 서버에 불러와 Isaac과 연결하고, timeout을 의도적으로 0.1초로 둔 2회 테스트에서 추론·action 적용·HOME 복귀·seed 변경·통계 저장·정상 종료를 확인했습니다. 이는 연결/리셋 검사이며 모델의 작업 성공률 측정이 아닙니다. 기본 60초 제한의 100회 평가는 아직 실행하지 않았습니다. 검사 결과는 `outputs/openpi_closed_loop/20261004_160530/summary.json`에 있습니다.
