# 2단 로켓 에비오닉스 프로토타입 작업 및 PCB 진단 정리

작성일: 2026-11-16  
대상 보드: ESP32-S3-DevKitC-1-N32R16V 기반 NURA 2단 로켓 에비오닉스 PCB

## 1. 문서 목적

이 문서는 제공된 PCB 배치 자료, 상태머신 설계서, 기존 NURA 에비오닉스 코드를
바탕으로 진행한 FreeRTOS 벤치 프로토타입 구현과 실제 PCB bring-up 결과를
정리한다. 특히 BNO085가 초기화되지 않았던 원인, 진단 과정, 최종 해결 방법을
재현 가능한 형태로 기록한다.

현재 펌웨어는 **벤치 검증용**이다. 점화기, 사출 화약, 모터 또는 실제 전개
장치를 연결해서는 안 된다. `bench`와 `pcb_bringup` 환경은 히터 관련 출력을
LOW로 유지한다.

## 2. 현재 상태 요약

| 항목 | 현재 상태 | 확인 내용 |
|---|---|---|
| ESP32-S3 부팅 | 완료 | N32R16V의 OPI Flash/PSRAM 설정으로 정상 부팅 |
| FreeRTOS 구조 | 완료 | 센서, FSM, 로그, 텔레메트리, 명령, 액추에이터 태스크 분리 |
| 태스크 간 데이터 전달 | 완료 | 공유 상태 순차 참조 대신 Queue와 불변 Snapshot 사용 |
| J7 단 식별 | 확인 | 개방=1단 BOOSTER, GND 점퍼=2단 SUSTAINER |
| 5 V 감지 | 확인 | Q6 오픈드레인 회로로 GPIO47 LOW가 5 V 존재 |
| 배터리 ADC | 확인 | 약 8.17 V 측정, 최종 보정은 멀티미터 비교 필요 |
| BNO085 | **독립 시험 PASS** | SPI/SHTP 및 가속도·선형가속도·자세값 수신 성공 |
| BMP388 | PASS | I2C 주소 0x77, chip ID 0x50 및 측정값 수신 |
| microSD | FAIL | 카드 선택 단계에서 실패, 카드·소켓·SPI 경로 점검 필요 |
| E22 | 부분 확인 | AUX HIGH 및 UART 송신 확인, 실제 RF는 상대 지상국 필요 |
| LED/부저 | 수동 확인 | 코드가 순서대로 구동하며 사람이 확인해야 함 |
| 히터/사출 출력 | SKIP/차단 | 안전상 LOW 고정, 더미 부하 절차 전에는 시험 금지 |
| 비행 투입 | 불가 | 물리 아밍·분리 감지·연속성·출력 채널 등 PCB 보완 필요 |

## 3. 참고 자료와 반영 사항

- PCB 부품 및 net 정보: `pcb_layout_reference.html`
- 상태전이 조건: 제공된 2단 로켓 상태머신 설계 문서
- 기존 코드 구조: `2026-nura-avionics`의 얇은 `main.cpp`, 앱 조립 계층,
  HAL과 센서 태스크 분리 방식
- 진단 인계 자료: PCB BOM, 부품 역할, J7 극성, BNO085 리셋 회로 확인
- 요구사항 자료: 전자팀/NURA 요구사항과 상태전이 임계값 검토

기존 코드처럼 `main.cpp`에는 앱 시작을 위한 최소 코드만 두고, 실제 기능은
`app`, `hal`, `sensors`, `missions`, `core`에 분리했다. 센서의 물리적인 읽기는
센서 태스크가 아니라 HAL이 담당한다.

## 4. FreeRTOS 데이터 구조

```text
IMU / BMP388 / power task
           |
           v
    sensor-message queue
           |
           v
    state-machine task
       |       |       |
       |       |       +--> actuator-command queue --> safe actuator task
       |       +----------> telemetry latest-frame queue --> E22 / USB
       +------------------> logging queue --> microSD

USB command task --> command queue --> state-machine task
```

상태머신 태스크 하나만 현재 비행 상태와 전이 카운터를 수정한다. 로그와
텔레메트리는 상태 구조체를 서로 다른 시점에 직접 읽지 않고, 상태머신이 만든
동일한 `FlightSnapshot` 복사본을 각자의 큐에서 받는다.

- 센서 큐: 센서 측정 사건 전달
- 로그 큐: SD 지연을 흡수할 수 있도록 여러 프레임 저장
- 텔레메트리 큐: 길이 1, 항상 최신 프레임 유지
- 액추에이터 큐: 상태머신의 출력 요청을 안전 태스크로 전달
- 명령 큐: 벤치용 `arm`, `disarm`, `sep` 명령 전달

## 5. 주요 코드 구조

```text
src/main.cpp
  -> app/flight_controller_app
       -> hal/bno085_hal
       -> hal/bmp388_hal
       -> hal/battery_voltage_hal
       -> sensors/imu_task
       -> sensors/barometer_task
       -> sensors/power_sense_task
       -> missions/flight/fsm_task
       -> core/flight_state_machine
       -> missions/logging/flight_log_task
       -> missions/telemetry/telemetry_task
       -> missions/system/command_task
       -> missions/system/actuator_task
```

추가 진단 파일은 실제 앱과 분리되어 있다.

- `src/diagnostics/pcb_bringup_main.cpp`: PCB 전체 안전 진단
- `src/diagnostics/bno085_standalone_main.cpp`: 기존 HAL/태스크를 사용하지 않는
  BNO085 단독 시험
- `lib/Adafruit BNO08x/`: SHTP 무한 재시도를 방지하도록 프로젝트에 고정한
  BNO08x 드라이버

## 6. PCB 핀과 확인된 논리

| 기능 | GPIO | 확인된 조건 |
|---|---:|---|
| BNO085 CS/MOSI/SCK/MISO | 10/11/12/13 | SPI mode 3, 1 MHz |
| BNO085 INT | 14 | Active-Low, 읽을 패킷이 있을 때 LOW |
| BNO085 reset control | 15 | HIGH=리셋, LOW=리셋 해제 |
| BMP388 SDA/SCL | 8/9 | I2C, 주소 0x77 |
| microSD SCK/MOSI/MISO/CS | 39/40/41/42 | 별도 SPI 버스 |
| E22 M0/M1/AUX | 4/5/6 | normal mode에서 M0/M1 LOW |
| E22 TX/RX | 17/18 | UART 9600 baud |
| J7 stage ID | 2 | 개방=BOOSTER, LOW=SUSTAINER |
| 배터리 ADC | 1 | 분압비 적용 |
| MAIN5V_PRESENT | 47 | LOW=5 V 존재 |
| HEAT_IN1/IN2/ARM | 3/7/48 | 벤치에서 항상 LOW |

### BNO085 리셋 회로

GPIO15는 BNO085의 `RST_N`에 직접 연결되지 않고 Q2를 구동한다. 독립 A/B 시험으로
실제 조립 보드의 동작을 확인한 결과는 다음과 같다.

```text
GPIO15 HIGH -> Q2 구동 -> BNO_RST_N LOW -> 리셋
GPIO15 LOW  -> Q2 해제 -> BNO_RST_N HIGH -> 정상 동작
```

## 7. BNO085 문제와 해결 과정

### 7.1 최초 증상

PCB 전체 진단이 다음 위치에서 더 진행되지 않았다.

```text
[RUN] BNO085 SPI
```

초기 진단 코드의 3초 제한은 `imu.begin()` 이후의 샘플 수신에만 적용되어 있었다.
Adafruit BNO08x 내부의 SHTP 쓰기 재시도 루프가 계속 `0`을 반환하면 초기화
함수가 끝나지 않아 BMP388 이후 검사가 실행되지 않았다.

### 7.2 진단 코드 격리

`pcb_bringup`에서 BNO085 검사를 별도 FreeRTOS 태스크로 실행하고 전체 초기화에
8초 제한을 적용했다. BNO085가 멈춰도 해당 태스크를 종료하고 BMP388, SD,
E22 검사를 계속 진행하도록 만들었다.

추가로 기존 HAL과 비행 코드를 전혀 사용하지 않는 `bno085_test` 환경을 만들고
다음을 직접 시험했다.

1. GPIO 및 SPI 직접 설정
2. 두 리셋 극성 A/B 시험
3. INT LOW 부팅 신호 확인
4. SPI/SHTP 헤더 수신
5. 제품 ID 요청
6. 가속도, 선형가속도, game rotation vector 활성화
7. 100 ms 간격 센서값 출력

### 7.3 실제 원인

핵심 문제는 납땜이나 SPI 배선이 아니라 **리셋 극성과 리셋 시점**이었다.

초기 구조는 BNO085를 먼저 수동 리셋한 뒤 `begin_SPI()`를 호출했다.
BNO085는 리셋 해제 직후 INT를 LOW로 만들고 부팅 SHTP 패킷을 준비하지만,
드라이버의 SPI/SHTP callback이 준비되기 전에 리셋하면 이 초기 통신 시점을
놓칠 수 있었다. 그 결과 `sh2_getProdIds()`가 `SH2_ERR_TIMEOUT(-6)`으로
실패했다.

### 7.4 최종 해결

리셋 핀을 BNO08x 드라이버에 전달하고 다음 순서로 변경했다.

```text
SPI 객체와 SHTP callback 준비
-> GPIO15 HIGH로 리셋
-> GPIO15 LOW로 리셋 해제
-> 300 ms 부팅 대기
-> INT 및 부팅 SHTP 패킷 처리
-> 제품 ID 확인
-> 센서 report 활성화
```

또한 `shtp.c`의 HAL 쓰기 루프를 최초 시도 포함 최대 8회로 제한했다. 8회 모두
쓸 수 없는 경우 `SH2_ERR_TIMEOUT`을 반환하므로 센서 하나가 전체 시스템을
무기한 정지시키지 않는다.

### 7.5 최종 검증 결과

독립 테스트에서 다음 데이터를 연속 수신했다.

```text
accel[m/s2]=(0.344, -0.422, 9.461)
linear[m/s2]=(0.023, 0.008, -0.016)
quat[wxyz]=(0.99957, -0.02228, -0.01727, 0.00037)
valid[A/L/Q]=1/1/1
```

가속도 크기는 약 9.48 m/s²로 정지 상태의 중력가속도와 유사하다. SHTP 패킷의
크기와 시퀀스가 연속적으로 증가했으며 세 report가 모두 유효했다. 따라서
BNO085 전원, Q2 리셋 회로, INT, CS/SCK/MOSI/MISO, SPI 모드, SHTP 및 센서
출력은 정상으로 판단한다.

초기화 중 단일 `SPI write timed out`이 표시되더라도 이후 report 세 개가 모두
`OK`이고 `[PASS] BNO085 initialized`가 나오면 제한된 재시도 중 다음 시도에서
성공한 것이다. 초기화 직후 한 번의 `reset detected` 역시 방금 수행한 동기화
리셋 사건이다. 비행 중 반복 발생하는 경우에만 전원/리셋 이상으로 취급한다.

## 8. PCB bring-up 결과

마지막 전체 진단에서 확인된 결과는 다음과 같다. BNO085 수정 후 전체 진단은
다시 실행해 PASS로 바뀌는지 최종 확인해야 한다.

| 검사 | 결과 | 해석 |
|---|---|---|
| Stage ID | BOOSTER | J7 개방을 정상적으로 1단으로 판별 |
| Main 5 V | PASS | GPIO47 active-low 감지 정상 |
| LED/부저 | WARN | 사람이 실제 점등/소리를 확인해야 함 |
| 배터리 ADC | PASS | 약 8.168 V |
| BNO085 | 독립 시험 PASS | 전체 bring-up 재검증 필요 |
| BMP388 | PASS | 약 79.6 kPa, 22.69 °C, 계산고도 약 1992 m |
| microSD | FAIL | 4 MHz에서 5회 모두 카드 선택/마운트 실패 |
| E22 UART | WARN | AUX HIGH와 UART 송신만 확인, RF 미검증 |
| 히터 출력 | SKIP | 안전상 LOW 고정 |

BMP388의 79.6 kPa는 약 2,000 m 고도에 해당한다. 실제 시험 장소가 그 고도가
아니라면 통신 성공과 별개로 센서 정확도, 기준 기압, 모듈 상태를 비교해야 한다.

## 9. microSD 남은 점검

현재 오류는 파일 쓰기보다 앞선 카드 선택 단계에서 발생한다.

```text
sdSelectCard(): Select Failed
f_mount failed: (3) The physical drive cannot work
```

권장 확인 순서:

1. 카드를 끝까지 다시 삽입한다.
2. 32 GB 이하의 알려진 정상 카드를 FAT32로 포맷해 교체 시험한다.
3. J4 핀 1과 3 사이에서 약 5 V를 확인한다.
4. 전원을 끄고 J4 소켓 납땜과 아래 경로의 연속성을 확인한다.

| J4 핀 | 신호 | 경로 |
|---:|---|---|
| 4 | SCK | R14 -> U5 -> GPIO39 |
| 5 | MISO | U5 -> GPIO41 |
| 6 | MOSI | R13 -> U5 -> GPIO40 |
| 7 | CS | R12 -> U4 -> GPIO42 |

## 10. 빌드 환경과 실행 방법

### 실제 벤치 프로토타입

```bash
pio run -e bench
pio run -e bench -t upload
pio device monitor -b 115200
```

### PCB 전체 안전 진단

```bash
pio run -e pcb_bringup
pio run -e pcb_bringup -t upload
pio device monitor -b 115200
```

### BNO085 독립 시험

```bash
pio run -e bno085_test
pio run -e bno085_test -t upload
pio device monitor -b 115200
```

세 환경 모두 컴파일 성공을 확인했다. ESP32-S3-N32R16V에 맞춰 OPI Flash와
OPI PSRAM 설정을 적용했으며, 잘못된 quad-flash 설정에서 발생하던 재부팅도
수정했다.

## 11. 비행 전 남은 핵심 과제

현재 코드는 센서와 상태머신의 벤치 프로토타입이며 비행 승인을 의미하지 않는다.

- microSD 카드 및 SPI 경로 수리/검증
- E22 두 대를 이용한 실제 RF 패킷 송수신 및 지상국 연동
- 대회 규정에 맞는 텔레메트리 패킷 필드, 크기, 주기 확정
- BMP388 기준고도와 실제 기압 비교 및 보정
- 배터리 ADC 멀티미터 비교 보정
- BNO085 좌표축과 로켓 종축의 실제 장착 방향 확정
- 물리 아밍 입력 추가
- 1단/2단 분리 감지 입력 추가
- 점화기/사출기 연속성 측정 회로 추가
- 서스테이너 점화, 주 사출, 백업 사출을 위한 독립 출력 채널 확보
- 더미 부하를 사용한 출력 시험 절차와 승인 기준 수립
- OpenRocket CSV 재생 및 상태전이 HIL 시험
- BNO085 ±8 G 범위와 예상 최대 약 9.4 G의 불일치 해결

## 12. 다음 작업 권장 순서

1. 수정된 `pcb_bringup`을 다시 업로드해 BNO085 PASS를 전체 진단에서 확인한다.
2. 정상 FAT32 카드 교체 후 microSD를 재시험한다.
3. BMP388을 알려진 기압계 및 실제 높이 변화와 비교한다.
4. E22 상대 무전기와 빵판 지상국으로 UART/RF 왕복 패킷을 검증한다.
5. 요구사항의 텔레메트리 패킷 형식을 코드와 지상국에 동시에 적용한다.
6. 센서값과 상태전이를 기록한 뒤 OpenRocket 결과와 비교한다.
7. PCB 안전 입력과 부족한 출력 채널을 수정하기 전까지 실제 점화 장치를
   연결하지 않는다.

## 13. 관련 문서

- `README.md`: 프로젝트 구조와 실행 개요
- `documents/flight_logic.md`: 상태전이 조건과 검증 계획
- `documents/pcb_gap_analysis.md`: PCB와 요구사항 차이
- `documents/pcb_bringup.md`: PCB 안전 진단 절차
- `src/diagnostics/bno085_standalone_main.cpp`: BNO085 독립 시험 코드

