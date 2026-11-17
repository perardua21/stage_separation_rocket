# 2단 로켓 에비오닉스 FreeRTOS 벤치 프로토타입

이 프로젝트는 제공된 ESP32-S3 PCB에서 센서 수집, 1단 및 2단 상태머신,
microSD 로그, E22 텔레메트리의 기본 흐름을 확인하기 위한 벤치 전용
프로토타입이다. 현재 빌드는 모든 히터 출력을 전기적으로 LOW에 고정한다.
점화기나 사출 장치를 연결하지 말고 LED 또는 멀티미터로 먼저 검증해야 한다.

## 현재 구현 범위

- ESP32-S3의 FreeRTOS 태스크와 큐를 직접 사용한다.
- BNO085를 100 Hz로, BMP388을 20 Hz로 읽는다.
- 부스터와 서스테이너 상태머신을 J7 단계 식별 점퍼로 선택한다.
- 상태머신이 만든 동일한 스냅샷을 별도 로그 큐와 텔레메트리 큐에 복사한다.
- microSD에 `flight.csv`를 기록하고 E22 UART와 USB 시리얼로 5 Hz 전송한다.
- USB 시리얼의 벤치 명령으로 PCB에 없는 입력을 흉내 낸다.
- 상태머신 코어는 Arduino와 독립적으로 호스트에서 테스트할 수 있다.

## 큐 구조

```text
IMU / barometer / power tasks
        |
        v
   sensor-message queue --> state-machine task --> actuator-command queue --> safe actuator task
                            |
                            +--> log queue ----------> microSD logger task
                            |
                            +--> latest-frame queue -> E22/USB telemetry task

USB console task --> command queue --> state-machine task
```

상태머신의 현재 상태는 상태머신 태스크 하나만 쓴다. 큐는 태스크 사이의
메시지를 운반한다. 로그와 텔레메트리가 `state` 구조체를 차례로 읽는 기존
방식과 달리, 각 소비자는 생성 시점이 고정된 스냅샷을 자기 큐에서 받는다.
텔레메트리는 길이 1 큐를 사용해 항상 최신 프레임만 유지하고, 로그는 길이
64 큐로 짧은 SD 지연을 흡수한다.

## 코드 구조

다시 제공된 기존 NURA 코드와 같은 조립 방식을 유지한다. `src/main.cpp`는
`FlightControllerApp`의 `setup()`과 `loop()`만 호출한다. 앱 클래스는 핀과
큐를 초기화하고 태스크 의존성을 연결하는 composition root이며, 센서 처리,
상태전이, 로그, 텔레메트리, 액추에이터 안전 정책은 각각의 모듈에 들어 있다.

```text
src/main.cpp
  -> app/flight_controller_app
       -> hal/{bno085,bmp388,battery_voltage}_hal
       -> sensors/{imu,barometer,power_sense}_task
       -> missions/flight/fsm_task -> core/flight_state_machine
       -> missions/logging/flight_log_task
       -> missions/telemetry/telemetry_task
       -> missions/system/{actuator,command}_task
       -> hal/bench_actuator_output
```

## PCB 연결

주요 핀은 제공된 HTML의 PCB net에서 복원했다.

| 기능 | ESP32-S3 GPIO | PCB net |
|---|---:|---|
| BMP388 SDA / SCL | 8 / 9 | CPU_BMP_SDA / CPU_BMP_SCL |
| BNO085 CS / MOSI / SCK / MISO | 10 / 11 / 12 / 13 | CPU_BNO_* |
| BNO085 INT / RESET | 14 / 15 | CPU_BNO_INT / BNO_RESET_ASSERT |
| microSD CS / MISO / MOSI / SCK | 42 / 41 / 40 / 39 | CPU_SD_* |
| E22 M0 / M1 / AUX | 4 / 5 / 6 | CPU_E22_* |
| E22 TX / RX | 17 / 18 | CPU_E22_UART_TX / RX |
| 히터 채널 1 / 2 / 공통 허가 | 3 / 7 / 48 | HEAT_IN1 / HEAT_IN2 / HEAT_ARM |
| 배터리 ADC | 1 | CPU_BAT_ADC |
| 단계 식별 J7 | 2 | STAGE_ID |

인계 BOM과 `assembly_extra.csv` 기준으로 J7 개방은 1단 부스터, J7을 GND에
연결한 상태는 2단 서스테이너다. BNO085가
달린 PCB의 `+X` 방향을 로켓 종축으로 가정한다. 두 가정 모두 실제 기구 조립
방향과 대조해 확인해야 한다.

## 빌드와 업로드

PlatformIO가 설치된 환경에서 다음을 실행한다.

```bash
pio run -e bench
pio run -e bench -t upload
pio device monitor -b 115200
```

부팅 후 PCB를 움직이지 않고 10초간 기준고도와 초기 자세를 잡는다. 이후
USB 시리얼에서 다음 벤치 명령을 사용할 수 있다.

```text
arm      SAFE -> ARMED
disarm   ARMED -> SAFE
sep      분리 감지 입력 1회 발생
```

`arm`과 `sep`은 PCB에 없는 입력을 시험하기 위한 벤치 명령이다. 모든 액추에이터
요청은 `[BENCH BLOCKED]`로 출력되고 실제 GPIO는 LOW로 유지된다.

호스트 상태머신 테스트는 다음과 같이 실행한다.

```bash
c++ -std=c++17 -Iinclude -Isrc \
  src/core/flight_state_machine.cpp test/test_state_machine.cpp \
  -o test_runner
./test_runner
```

PCB 납땜과 주변장치만 점검하는 별도 안전 진단 빌드는 다음과 같다.

```bash
pio run -e pcb_bringup
pio run -e pcb_bringup -t upload
pio device monitor -b 115200
```

사용 절차와 결과 해석은
[`documents/pcb_bringup.md`](documents/pcb_bringup.md)에 정리되어 있다.

## 비행 전에 반드시 해결할 항목

현재 PCB와 요구사항 사이의 차이 때문에 이 코드는 비행용으로 빌드되지 않는다.

- 물리 아밍 스위치 입력이 없다.
- 분리 확인 입력이 없다.
- 점화기 및 사출기 연속성 측정 입력이 없다.
- 출력이 2개뿐이라 서스테이너의 점화, 주 사출, 백업 사출 3개를 분리할 수 없다.
- 문서에서 추가 시뮬레이션이 필요하다고 한 RECOVERY 강제 정점 시각이 없다.
- PCB 종축과 BNO085 좌표축 대응이 확정되지 않았다.
- BNO085의 ±8 G 범위가 시뮬레이션 최대 약 9.4 G보다 작다.
- 기압 기준고도, 배터리 ADC 보정, E22 설정과 국내 주파수 규정 검증이 필요하다.

상세한 안전 제약과 검증 계획은
[`documents/flight_logic.md`](documents/flight_logic.md)와
[`documents/pcb_gap_analysis.md`](documents/pcb_gap_analysis.md)에 있다.

2026-11-16까지의 구현 변경, PCB bring-up 결과, BNO085 장애 원인과 해결 과정,
남은 작업은
[`documents/work_summary_2026-11-16.md`](documents/work_summary_2026-11-16.md)에
통합 정리되어 있다.

## 참고 구현

다시 제공된 NURA 코드의 얇은 `main.cpp`, `FlightControllerApp` 조립 계층,
`HAL -> sensor task -> state -> FSM/log/telemetry` 분리와 상태 및 원인 코드
표현을 참고했다. 이번 프로토타입은 협력형 스케줄러 대신 FreeRTOS 태스크를
쓰며, 태스크 경계의 공유 상태를 큐 메시지와 불변 스냅샷으로 바꿨다.
