# 상태머신

상태명은 운용개념서 v0.5 4장 표기를 따른다. 접두어 없는 INIT / SAFE / ARMED /
GROUND / FAULT는 두 역할 공통이고, 나머지는 역할로 구분한다.
enum 값은 `include/flight_types.h`.

## 흐름

```
부스터
INIT -> SAFE -> ARMED -> BOOST -> SEPARATING -> COAST -> APOGEE -> DEPLOY -> DESCENT -> GROUND
                                      |
                                      +-> RECOVERY -> APOGEE (분리 실패)

서스테이너
INIT -> SAFE -> ARMED -> RIDE -> SEPARATED -> IGNITING -> BOOST -> COAST -> APOGEE -> DEPLOY -> DESCENT -> GROUND
                           |          |            |
                           +----------+------------+-> RECOVERY -> APOGEE (고장)
```

점화와 분리는 상태가 아니라 사건이다(운용개념서 3장). 다만 펌웨어는 그 사건을
기다리고 타임아웃을 재야 하므로, 기다리는 구간만 상태로 둔다. 사건이 일어난
시각은 상태와 별도로 `t_sep_ms`, `t_ign_ms`에 남긴다.

## 기준 시각

모든 강제전이 타이머는 상태 진입 시각이 아니라 아래 네 시각을 기준으로 한다.
상태 진입 시각은 판정이 늦거나 빠르면 같이 흔들리기 때문이다.

| 기호 | 뜻 |
|---|---|
| `t0_ms` | 이륙 감지 |
| `t_sep_ms` | 분리 확인 |
| `t_ign_ms` | 2단 점화 추력 확인 |
| `t_apogee_ms` | 정점 확정 |

## 부스터 전이

| 상태 | 전이 조건 | 다음 | 강제 / 고장 |
|---|---|---|---|
| INIT | 셀프테스트·기준기압 측정 완료 | SAFE | 실패 -> FAULT |
| SAFE | 아밍 스위치 + 지상국 ARM | ARMED | - |
| ARMED | 가속도 >= 2.0 G 연속 50 ms | BOOST (`t0`) | 지상국 DISARM -> SAFE |
| BOOST | 가속도 <= 2 G 연속 50 ms & `t0`+0.5 s 이후 | SEPARATING | `t0`+1.3 s 강제 |
| SEPARATING | **분리 게이트 3개 모두 참**일 때 분리 열선 발사, 분리 확인 | COAST (`t_sep`) | 재시도 1회 후 `t0`+2.5 s -> RECOVERY(NO_SEP) |
| COAST | `t0`+3.5 s 이후, 고도 < h_max − 3 m 연속 5샘플 & vz <= 0 | APOGEE | `t0`+6.2 s 강제 |
| APOGEE | 정점 기록 + 열선 도통 확인 | DEPLOY | 200 ms 넘으면 그냥 DEPLOY |
| DEPLOY | 열선 시퀀스 종료 | DESCENT | - |
| DESCENT | 5 s 동안 고도 변화 < 2 m | GROUND | `t_apogee`+40 s 강제 |
| GROUND | - | - | 10 s 추가 로깅 후 마감, 비콘 |
| RECOVERY | COAST와 같은 정점 판정 | APOGEE | TODO: 스택 비행 시뮬 |

### 분리 게이트 (팀 결정 2026-11-04)

세 조건을 **모두** 만족해야 분리 열선을 발사한다.

1. `t0` + 1.5 s 경과 — 1단 연소가 끝났을 것
2. **절대고도 >= 100 m** — 새로 추가된 조건
3. 수직속도 > 0 — 아직 상승 중일 것

고도 조건을 넣는 이유: 시간 조건만 쓰면 점화 실패나 조기 연소종료로
제대로 못 올라간 비행에서도 분리가 일어나서, 지면 가까이에서 두 동강이 난다.

> ⚠ 시뮬(OpenRocket)상 분리 고도는 약 65 m다. 100 m를 그대로 쓰면 분리가
> `t0`+2.1 s쯤으로 약 0.6 s 밀리고, 2단 점화도 같이 밀린다.
> 이 PCB를 올릴 기체가 시뮬과 같은 기체인지 확인이 필요하다 (04-open-items 1번).

## 서스테이너 전이

| 상태 | 전이 조건 | 다음 | 강제 / 고장 |
|---|---|---|---|
| RIDE | 분리 확인 | SEPARATED (`t_sep`) | `t0`+2.5 s 미분리 -> RECOVERY(NO_SEP) |
| SEPARATED | **점화 게이트 5개 모두 참** -> 점화 | IGNITING | 기울기 >= 20도 즉시, 창 종료 시 -> RECOVERY |
| IGNITING | 명령 후 1.0 s 안에 가속도 >= 3 G 연속 50 ms | BOOST (`t_ign`) | 미확인 -> RECOVERY(NO_IGN). 재점화 없음 |
| BOOST | 가속도 <= 2 G 연속 50 ms & `t_ign`+1.0 s 이후 | COAST | `t_ign`+2.2 s 강제 |
| COAST | `t_ign`+6.0 s 이후, 부스터와 같은 정점 판정 | APOGEE | `t_ign`+12.0 s 강제 |
| 이후 | 부스터와 같음 | | |

### 점화 게이트

다섯 조건을 **모두** 만족할 때만, **한 번만** 점화한다.

| # | 조건 | 값 | 이유 |
|---|---|---|---|
| 1 | `t_sep` + 0.5 s 이상 | `CFG_S_IGN_WINDOW_MIN_MS` | 부스터와 거리 확보 |
| 2 | `t_sep` + 1.5 s 이내 | `CFG_S_IGN_WINDOW_MAX_MS` | 늦으면 속도를 잃고 기울어진다 |
| 3 | 수직 대비 기울기 < 20도 | `CFG_S_IGN_MAX_TILT_DEG` | 옆으로 날아가는 것을 막는다 |
| 4 | 가속도 적분 속도 > 30 m/s | `CFG_S_IGN_MIN_VEL_MPS` | 기압은 락아웃 중이라 못 쓴다 |
| 5 | 고도 >= 100 m | `CFG_S_IGN_MIN_ALT_M` | 분리 조건과 같은 값 |

조건별 판정값을 그대로 로그에 남긴다. 비행 후에 "왜 쐈는지 / 왜 안 쐈는지"를
숫자로 설명할 수 있어야 하기 때문이다.

## 고장 처리

운용개념서 8장대로 비행 상태와 이상 상태를 다른 축으로 둔다.
센서·통신·저장 이상은 `health_flags`에만 기록하고 상태를 바꾸지 않는다.
상태를 바꾸는 것은 아래뿐이다.

| 상황 | 처리 |
|---|---|
| 초기화 실패 | FAULT. 비행 불가 |
| 치명 센서 고장 (ARMED 이하) | SAFE로 복귀, 아밍 해제 |
| 치명 센서 고장 (발사 이후) | RECOVERY. **열선은 끄지 않는다** |
| 분리 / 점화 실패 | RECOVERY(원인코드) |
| 비행 중 리셋 | 재부팅 시 고도 > 20 m면 바로 RECOVERY |

RECOVERY는 멈추는 상태가 아니다. 2단 점화만 영구 금지하고, 정점 판정과
낙하산 사출은 그대로 수행한다. 어떤 고장이 나도 낙하산은 펴야 한다.
