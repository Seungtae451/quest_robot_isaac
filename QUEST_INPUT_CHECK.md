# Quest 입력 주기 및 처리 최적화

검사일: 2026-10-03 KST. 주기는 서버에 도착한 **유효 controller WSS 이벤트**로
측정합니다. 헤드셋 내부 센서 샘플링 주기나 네트워크 전체 지연을 뜻하지 않습니다.

## 실제 Quest의 기존 주기

로봇 action을 보내지 않는 pass-through 서버에 실제 헤드셋을 연결해 20초
측정했습니다. [기존 측정 원본](outputs/quest_input_check/measurement.json).

| 항목 | 측정값 |
|---|---:|
| controller 이벤트 | 약 30.30 Hz |
| 이벤트 간격 중앙값 | 33.06 ms |
| 간격 95 percentile | 34.65 ms |
| 가장 긴 간격 | 63.28 ms |
| 측정 창의 유효 이벤트 | 605개 |

head 이벤트는 다른 종류의 이벤트이며 controller rate와 별도로 기록합니다.
1024개 ring buffer를 넘는 controller/head timestamps는 오래된 것부터
제외되므로 rate/jitter는 보관된 최근 구간에 대한 값입니다.

## 실제 Quest 재측정

브라우저 fps를 60으로 명시한 뒤 같은 헤드셋에서 20초 재측정을 마쳤습니다.
[재측정 원본](outputs/quest_input_check/measurement_60hz.json)의 `complete=true`.

| 항목 | 기존 | 변경 후 |
|---|---:|---:|
| 실제 수신 controller 주기 | 30.30 Hz | 62.49 Hz |
| 간격 중앙값 | 33.06 ms | 16.09 ms |
| 간격 95 percentile | 34.65 ms | 17.91 ms |
| 가장 긴 간격 | 63.28 ms | 32.63 ms |

변경 후 서버가 받은 유효 controller 이벤트는 전체 1248개이고, 최근
1024개(약 16.4초)가 최종 rate/jitter 계산에 사용됐습니다. 60Hz 설정은
클라이언트에 대한 요청값이며 실제 callback 주기는 정확히 60.00Hz로
고정되지 않습니다. 센서 주기나 전체 동작 지연의 검증은 아닙니다.

## 변경

- `QUEST_INPUT_HZ=60`: Vuer `MotionControllers`의 지원되는 `fps` 속성을
  명시해 브라우저에 60Hz 스트림을 요청합니다. 실제 수신 주기는 따로 측정합니다.
  adapter에서 controller scene element만 변경하고 upstream 패키지는 수정하지 않습니다.
- `CONTROL_HZ=60`: 최신 입력을 확인하는 루프를 60Hz로 올립니다.
  새로운 controller serial에만 IK와 필터 및 action 전송을 실행합니다.
  고정된 같은 자세라도 새로운 이벤트면 정상 처리합니다. 이벤트가 멈추면
  오래된 action을 재송신하지 않아 receiver watchdog이 감속 정지합니다.
- `FEEDBACK_HZ=60`: 이전 10Hz 관절 조회를 최대 60Hz로 변경해 IK 초기값의
  오래된 measured pose를 줄입니다. 실제 업데이트는 Isaac 루프의 처리율에도 의존합니다.
- 변환한 pose를 controller/head serial이 같을 때 재사용합니다. head 이벤트가
  새로 오면 좌표를 다시 계산하고, fresh/stale 및 invalid 검사는 항상 유지합니다.
- 필터 alpha를 처리 간격 `dt`로 환산합니다.
  `alpha_dt = 1 - (1 - alpha_30Hz) ** (dt * 30)`이므로 입력 주기를 바꿔도
  기존 위치·회전·gripper 필터의 시간 응답을 유지합니다.
- `runtime.jsonl`에 실제 수신 Hz, 처리 tick Hz, IK Hz/소요시간,
  이벤트 도착부터 처리까지의 age, 건너뛴 이벤트 수를 기록합니다.

물리 60Hz, 카메라 30fps, IK 최대 100회, 관절 속도/가속도 제한은 유지했습니다.
이 변경은 관절 한계나 도달 불가능한 6DoF 목표를 해결하는 기능은 아닙니다.

## 검증

- 전체 CPU 검사: **47 passed**. 수신 타임스탬프의 rate/jitter 계산,
  20/30/60/90Hz에서 같은 필터 응답, pose cache와 stale 검사,
  controller fps 속성 및 기존 IK/이동 제한을 확인했습니다.
- 실제 Isaac와 Quest 서버에 약 30Hz **합성 WSS 입력**을 넣는 전체 경로
  검사가 통과했습니다. 처리 tick 중앙값 59.42Hz, IK 28.79Hz였고 중복
  입력에서 계산하지 않았습니다. IK p95의 창별 중앙값은 3.01ms,
  서버 도착 후 처리 age p95의 창별 중앙값은 15.57ms였습니다.
  이 값은 실제 Quest 입력의 수치와 구분해야 합니다.
- IK 실패/감속 정지/복구, tracking loss, 송신기 재시작,
  Isaac 재시작 모두 통과했습니다.
  [검증 결과](outputs/pipeline_check/validation.json),
  [성능 기록](outputs/pipeline_check/timing.jsonl).
- 변경된 브라우저 fps의 실제 헤드셋 재측정에서 62.49Hz 수신을 확인했습니다.
  위 합성 입력 검증은 약 30Hz 입력이며, 실제 헤드셋 재측정은 로봇 action을
  보내지 않는 측정 모드입니다. 실제 teleoperation의 처리 시간은 runtime
  로그로 별도 확인할 수 있습니다.

## 사용

Quest 송신기를 재시작하고 기존 Quest 페이지를 새로고침해 VR에 들어갑니다.
Isaac의 물리/카메라 설정은 바꾸지 않았습니다.

```bash
python scripts/run_quest_teleop.py --host-ip 192.168.100.136
```

터미널의 `input=...Hz`, `solve=...Hz`, `ikP95=...ms`, `inputAgeP95=...ms`와
`outputs/quest_input_check/runtime.jsonl`을 확인합니다. `input`이 여전히
30Hz이면 실제 브라우저/헤드셋 스트림은 요청한 60Hz를 제공하지 않은 것입니다.

단독 측정은 기존 Quest 송신기를 종료한 뒤 실행합니다. 로봇에 명령을 보내지 않습니다.

```bash
python scripts/measure_quest_input.py --host-ip 192.168.100.136 --seconds 20 \
  --output outputs/quest_input_check/measurement_60hz.json
```
