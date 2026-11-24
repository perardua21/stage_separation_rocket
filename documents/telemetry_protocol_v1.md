# NURA 2단 로켓 무선 프로토콜 v1

작성: 코딩팀 / 대상: 전자팀, 지상국 담당
구현: `include/telemetry_protocol.h`, `src/core/telemetry_protocol.cpp`
테스트: `pio test -e native` (프레임 왕복, CRC, 재동기, 연속 프레임)

---

## 1. 물리 계층

| 항목 | 값 |
|---|---|
| 모듈 | E22-900T30D-V2 (SX1262, 1 W) |
| 모드 | 투명 전송 (M0 = 0, M1 = 0) |
| UART | 9600 bps, 8N1 |
| 노드 | 1단 에비오닉스, 2단 에비오닉스, 지상국 |

E22 투명 모드는 **그냥 UART 바이트 파이프**다. 프레이밍, 주소, 오류검출이
전혀 없고 세 노드가 같은 주파수를 공유한다. 그래서 프레임·주소·CRC·충돌
회피를 전부 이 계층에서 직접 만든다.

### 공중 전송률 — 확인 필요

E22 기본값이 2.4 kbps면 32 바이트 프레임 하나가 약 107 ms를 쓴다.
두 기체가 각 2 Hz로 보내면 초당 428 ms를 점유하는 셈이라 여유가 거의 없다.

- 2.4 kbps를 그대로 쓸 경우: `kTelemetryPeriodMs`를 1000으로 올린다
- 권장: 모듈을 **9.6 kbps 이상**으로 설정 (M0=0, M1=1 설정 모드에서 레지스터 기록)

현재 코드는 500 ms 주기(2 Hz)로 되어 있다. 전자팀에서 모듈 설정값을
알려주면 맞춰 조정한다.

---

## 2. 프레임 형식

고정 헤더 6바이트 + 가변 페이로드 + CRC 2바이트. **리틀엔디안.**

```
 off  size  field
  0     2   sync        0xA5 0x5A
  2     1   version     1
  3     1   source      노드 ID
  4     1   type        프레임 종류
  5     1   payloadLen  페이로드 바이트 수
  6   len   payload
6+len   2   crc16       bytes[2 .. 6+len-1] 에 대한 CRC16-CCITT
```

- CRC 범위에서 sync는 제외한다. 수신기는 sync로 재동기만 하고 검증은 그 뒤 내용에만 한다.
- CRC16-CCITT: 다항식 `0x1021`, 초기값 `0xFFFF`, 리플렉션 없음
- 수신기는 바이트 스트림에서 sync를 찾아 재동기하므로, **중간에 켜도 다음 프레임부터 정상 수신된다**

### 노드 ID

| 값 | 노드 |
|---|---|
| 1 | 1단 에비오닉스 |
| 2 | 2단 에비오닉스 |
| 9 | 지상국 |
| 0xFF | (target 필드에서만) 전체 |

### 프레임 종류

| 값 | 종류 | 방향 | 전체 크기 |
|---|---|---|---|
| 1 | TELEMETRY | 기체 → 지상국 | 32 B |
| 2 | COMMAND | 지상국 → 기체 | 12 B |
| 3 | ACK | (예약, 미사용) | — |

---

## 3. TELEMETRY 페이로드 (24 B)

| off | size | 타입 | 필드 | 단위 |
|---|---|---|---|---|
| 0 | 4 | uint32 | timestampMs | ms (기체 부팅 기준) |
| 4 | 1 | uint8 | state | FlightState |
| 5 | 1 | uint8 | reason | ReasonCode |
| 6 | 1 | uint8 | flags | 아래 표 |
| 7 | 1 | uint8 | reserved | 0 |
| 8 | 4 | float | altitudeM | m (영점 상대) |
| 12 | 2 | int16 | verticalVelocityCmps | m/s × 100 |
| 14 | 2 | int16 | accelerationMilliG | g × 100 |
| 16 | 2 | int16 | maxAltitudeDm | m × 10 |
| 18 | 2 | uint16 | batteryMilliV | mV |
| 20 | 2 | int16 | tiltDeciDeg | deg × 10 |
| 22 | 2 | uint16 | droppedFrames | 개 |

구조체를 그대로 보내지 않고 바이트를 직접 깐다. 패딩·정렬이 컴파일러마다
달라서, 지상국을 다른 툴체인(또는 파이썬)으로 짜도 깨지지 않게 하기 위한
것이다.

### flags 비트

| 비트 | 이름 | 의미 |
|---|---|---|
| 0 | SEPARATED | 분리 감지 래치됨 |
| 1 | HEATER_ARMED | HEAT_ARM 올라가 있음 (화약 시퀀스 진행 중) |
| 2 | IMU_VALID | BNO085 정상 |
| 3 | BARO_VALID | BMP388 정상 |
| 4 | ARMED | Init/Safe 가 아님 |
| 5 | STAGE_LINK_AT_BOOT | 부팅 때 J7 점퍼가 연결되어 있었음 |

**비트 5는 발사 전 점검용이다.** 이게 0이면 단간 점퍼를 꽂지 않은 것이고,
그 기체는 분리 감지가 비활성 상태다. 지상국 화면에서 발사 전에 확인한다.

### 상태 / 이유 코드

`include/flight_types.h`의 `FlightState`, `ReasonCode` enum 순서를 그대로
쓴다. 지상국은 `flightStateName()` / `reasonCodeName()`으로 문자열로
바꾼다. **두 쪽 다 같은 헤더를 쓰므로 enum에 항목을 끼워 넣으면 번호가
밀려 깨진다. 항목은 반드시 끝에 추가한다.**

---

## 4. COMMAND 페이로드 (4 B)

| off | size | 타입 | 필드 |
|---|---|---|---|
| 0 | 1 | uint8 | command |
| 1 | 1 | uint8 | target (노드 ID, 0xFF = 전체) |
| 2 | 2 | uint16 | nonce |

| 값 | 명령 |
|---|---|
| 1 | ARM |
| 2 | DISARM |
| 3 | PING |

### 안전 규칙

1. **화약을 터뜨리는 무선 명령은 프로토콜에 존재하지 않는다.** 사출은
   기체 자신의 상태머신만 결정한다. 지상국이 보낼 수 있는 것은
   ARM / DISARM / PING 뿐이다.
2. ARM / DISARM은 기체가 **지상 상태(Init, Safe, Armed, Ground)** 일 때만
   받는다. 비행 중에는 거부하고 시리얼에 기록한다.
3. 투명 모드에서 충돌을 피할 방법이 없으므로 지상국은 같은 명령을
   **60 ms 간격으로 3번** 보낸다. 수신 측은 `nonce`로 중복을 걸러
   한 번만 처리한다.
4. 물리 ARM 스위치가 없다(전자팀 E 항목). 즉 ARM은 소프트웨어만으로
   이뤄진다. 화약을 꽂은 상태에서 전원을 넣는 작업 순서를 반드시
   정해두어야 한다. → **5절 미결 항목 참조**

---

## 5. 충돌 회피

1단과 2단이 같은 주기로 송신하되 시각을 엇갈리게 한다.

| 노드 | 슬롯 오프셋 | 주기 |
|---|---|---|
| 1단 | 0 ms | 500 ms |
| 2단 | 250 ms | 500 ms |

두 기체의 시계가 동기화되어 있지 않으므로 이건 **완전한 TDMA가 아니라
통계적 분산**이다. 프레임이 겹치면 CRC에서 버려지고 다음 주기에 다시
온다. 텔레메트리는 최신값만 의미가 있으므로(깊이 1 덮어쓰기 큐) 몇 개
빠져도 문제가 없다.

지상국은 `stat` 명령으로 노드별 수신 프레임 수와 CRC 오류 수를 볼 수
있다. **CRC 오류율이 높으면 전송률을 올리거나 주기를 늘려야 한다.**

---

## 6. 지상국

빵판 ESP32 + E22만으로 동작한다. 센서도 SD도 화약도 없다.

```
pio run -e ground -t upload -t monitor
```

핀 배선은 `src/diagnostics/ground_station_main.cpp` 위쪽 상수에서 고친다.
보드가 ESP32-S3면 `platformio.ini`의 `[env:ground]`에서 `board`를 바꾼다.

### 콘솔 명령

| 입력 | 동작 |
|---|---|
| `arm1` / `arm2` / `arma` | 1단 / 2단 / 전체 ARM |
| `dis1` / `dis2` / `disa` | 1단 / 2단 / 전체 DISARM |
| `ping` | 전체 핑 |
| `stat` | 노드별 수신 상태, CRC 오류 수 |

### 출력 예

```
S1    20854 SAFE       NONE               [----B-] alt=   -9.6m vz=  -0.0 a= 0.97g max=   0.0m tilt=  0.0 batt= 7.95V drop=0
S2    21104 ARMED      NONE               [A--IBL] alt=    0.1m vz=   0.1 a= 1.00g max=   0.0m tilt=  0.2 batt= 7.91V drop=0
```

플래그 자리는 순서대로 `A`rmed, `H`eater armed, `S`eparated, `I`MU,
`B`aro, stage `L`ink. `-`는 꺼진 상태다.

---

## 7. 미결 항목 — 전자팀 / 운용 확인 필요

| # | 항목 | 왜 필요한가 |
|---|---|---|
| 1 | E22 공중 전송률 설정값 | 텔레메트리 주기를 정하려면 필요. 2.4 kbps면 2 Hz는 무리 |
| 2 | 점화기 저항 (D 항목) | 미측정. 전류 확인 전에는 점화기 연결 금지 |
| 3 | 물리 ARM 스위치 없음 (E 항목) 대응 작업 순서 | 화약 꽂은 상태에서 전원 투입 순서를 문서로 못박아야 함 |
| 4 | 분리하지 않는 스택 전체 시뮬 | `kApogeeTimeoutFromLaunchMs` 백업 사출 시점 |
| 5 | 1단 모터 연소시간 | `kBurnoutTimeoutMs` |
| 6 | 1단 백업 화약 장착 여부 | `kBoosterBackupChargeFitted` |
| 7 | 2단 모터 유무 | 현재 **없음**으로 구현. 있으면 상태머신 변경 필요 |

---

## 8. 버전 관리

`version` 필드가 1이다. 프레임 구조가 바뀌면 올린다. 수신기는 모르는
버전의 프레임을 버린다.

구조를 바꿀 때는 **기체와 지상국을 같이 올려야 한다.** 한쪽만 올리면
전부 CRC 오류나 버전 불일치로 버려진다. 그래서 변경 시 PR 하나에 양쪽을
같이 담는다.
