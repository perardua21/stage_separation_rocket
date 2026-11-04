# stage_separation_rocket

2단 분리 로켓 비행 컴퓨터 펌웨어 — **구조 설계 초안**

이 저장소는 아직 동작하는 펌웨어가 아니다. 폴더 구조, 모듈 경계, FreeRTOS
태스크·큐 토폴로지, 상태머신 뼈대만 정의한 초안이다. `TODO:`가 붙은 값은
확정되지 않은 것이므로 비행 빌드에 쓰지 않는다.

## 읽는 순서

1. `docs/00-structure.md`   — 레이어와 모듈 경계
2. `docs/01-pinmap.md`      — PCB 넷 ↔ 기능 (GPIO 번호 미확정)
3. `docs/02-state-machine.md` — 상태 정의와 전이 조건
4. `docs/03-rtos.md`        — 태스크·큐 설계, 상태 공유 방식
5. `docs/04-open-items.md`  — **먼저 읽을 것.** 설계가 막혀 있는 항목

## 기준 하드웨어

TUSI Economic Final PCB (140 x 80 mm, SHA-256 2d06e74f...) 부품표 기준.

| 기능 | 부품 | 인터페이스 |
|---|---|---|
| MCU | ESP32-S3-DevKitC-1-N32R16V | ESP-IDF + FreeRTOS |
| 자세·가속도 | BNO085 (Adafruit PID4754) | SPI |
| 기압 | BMP390 (Adafruit PID4816) | I2C 0x77 |
| 저장 | microSD (Adafruit PID254) | SPI |
| 통신 | E22-900T30D-V2 (SX1262 1W) | UART + M0/M1/AUX |
| 열선 출력 | 2채널 (J8, J9) + 공통 ARM | GPIO |
| 단 식별 | STAGE ID STRAP (J7) | GPIO 입력 |
| 상태 표시 | 부저 TMB12A05, LED 적/녹 | GPIO |
| 전원 감시 | 배터리 ADC, 5V 존재 감지 | ADC / GPIO |

GPS는 없다. 고G 가속도계도 없다(BNO085 하나).

## 빌드

```bash
pio run -e flight          # 비행 빌드
pio run -e bench           # 벤치 빌드 (열선 출력 강제 차단)
pio test -e native         # 호스트 단위 테스트
```

## 설계 원칙

1. 하드웨어 핀은 `src/bsp/`만 만진다.
2. 열선 출력은 `task_actuator` 하나만 구동한다. FSM은 큐로 요청만 한다.
3. 상태 전이는 `task_fsm`만 한다. 다른 태스크는 이벤트를 큐로 올린다.
4. 통신이 끊겨도 사출과 로깅은 독자적으로 끝낸다.
5. 임계값은 `include/flight_config.h`에만 둔다. 코드에 숫자를 직접 쓰지 않는다.
6. 출처가 없는 숫자는 `TODO:`로 표시하고 비행 빌드에서 쓰지 않는다.
