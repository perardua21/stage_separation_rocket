# 구조 개요

## 레이어

```
app_main (src/main.c)
   |  열선 OFF -> 표시장치 -> STAGE_ID로 역할 결정 -> 큐 -> 태스크
   v
[Core 0]  task_sensor ──q_imu/q_baro──> task_estimator ──q_nav──┐
                                                                 │
[Core 1]  task_fsm <─────────────────────────────────────────────┘
             │  ├─ q_act ──> task_actuator ──> bsp_heater (열선)
             │  ├─ q_log ──> task_logger   ──> bsp_storage (microSD)
             │  └─ q_tlm ──> task_telemetry──> bsp_radio (E22)
             │                      │
             └──── q_event <────────┴──── task_health, q_uplink
```

## 모듈 책임

| 폴더 | 책임 | 하면 안 되는 것 |
|---|---|---|
| `src/bsp/` | 핀·버스·칩 접근 | 비행 판정, 상태 전이 |
| `src/flight/` | 상태머신, 판정기, 사출 시퀀스 | 핀 직접 접근, 블로킹 |
| `src/app/` | 태스크 루프. 큐를 오가는 배선 | 판정 로직을 여기 쓰지 않기 |
| `src/common/` | 상태 스냅샷, 패킷 정의 | 하드웨어 의존 |
| `include/` | 타입, 핀, 임계값, 토폴로지 | 구현 |

## 작년(2026) 코드와 달라지는 것

| 항목 | 2026 (Teensy) | 이번 (ESP32-S3) |
|---|---|---|
| 스케줄링 | 협력형 스케줄러, 한 루프에서 tick | FreeRTOS SMP, 코어 2개에 태스크 고정 |
| 데이터 전달 | 공유 state 구조체를 직접 읽고 씀 | 샘플·이벤트는 큐, 현재 상태는 seqlock 스냅샷 |
| 역할 구분 | 빌드 플래그 | STAGE_ID 핀 (같은 바이너리 2장) |
| 사출 | 화약 MOSFET 2채널 | 열선 2채널 + 공통 ARM |
| 자세 | 자체 쿼터니언 적분 | BNO085 내장 융합 |
| 고G | H3LIS331DL 별도 | 없음 (04-open-items 2번) |
| GPS | u-blox M6 | 없음 |

## 왜 큐인가

작년처럼 전역 state 구조체를 여러 태스크가 직접 읽고 쓰면, FreeRTOS에서는
선점이 아무 때나 일어나서 "절반만 갱신된 값"을 읽을 수 있다. 특히
`nav_state_t`처럼 여러 필드가 한 시점을 함께 나타내는 구조체가 위험하다.

그렇다고 모든 것을 큐로 보내면 안 된다. 큐는 흐름(이벤트·샘플)에 맞고,
"현재 상태"처럼 최신값만 의미 있는 데이터에는 맞지 않는다. 그래서 둘로 나눈다.

- 흐름 → 큐 (`q_imu`, `q_baro`, `q_event`, `q_act`, `q_log`, `q_uplink`)
- 현재 값 → seqlock 스냅샷 (`state_snapshot_t`), 또는 길이 1 덮어쓰기 큐 (`q_nav`)

자세한 이유는 `include/rtos_topology.h`와 `src/common/state_snapshot.h` 주석.
