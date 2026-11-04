# 핀맵

출처: TUSI Economic Final PCB 부품표 (SHA-256 `2d06e74f7b4438a6`).

> **GPIO 번호는 아직 없다.** 부품표에는 ESP32-S3-DevKitC-1 모듈의 물리 핀
> 번호(1~44)와 넷 이름만 있다. KiCad 회로도에서 모듈 핀 → GPIO 매핑을 확인한
> 뒤 `include/board_pins.h`의 `PIN_UNASSIGNED`를 채운다.

## 넷 ↔ 기능

| 기능 | 넷 이름 | 모듈 핀 | 방향 | 비고 |
|---|---|---|---|---|
| BNO085 CS | `/CPU_BNO_CS_GPIO` | 16 | 출력 | R6 10k 풀업 |
| BNO085 MOSI | `/CPU_BNO_MOSI_GPIO` | 17 | 출력 | R3~R5 33R 직렬 |
| BNO085 SCK | `/CPU_BNO_SCK_GPIO` | 18 | 출력 | |
| BNO085 MISO | `/CPU_BNO_MISO` | 19 | 입력 | |
| BNO085 INT | `/CPU_BNO_INT` | 20 | 입력 | 데이터 준비 알림 |
| BNO085 RESET | `/BNO_RESET_ASSERT` | 8 | 출력 | Q2(2N7002) 구동 — 극성 확인 |
| BMP390 SDA | `/CPU_BMP_SDA` | 12 | 양방향 | 모듈에 풀업 내장, 주소 0x77 |
| BMP390 SCL | `/CPU_BMP_SCL` | 15 | 출력 | 400 kHz |
| SD CS | `/CPU_SD_CS_GPIO` | 28 | 출력 | R15 10k 풀업 |
| SD MISO | `/CPU_SD_MISO` | 29 | 입력 | |
| SD MOSI | `/CPU_SD_MOSI_GPIO` | 30 | 출력 | R12~R14 33R |
| SD SCK | `/CPU_SD_SCK_GPIO` | 31 | 출력 | |
| E22 TX | `/CPU_E22_UART_TX` | 10 | 출력 | MCU→모듈 |
| E22 RX | `/CPU_E22_UART_RX` | 11 | 입력 | 모듈→MCU |
| E22 M0 | `/CPU_E22_M0` | 4 | 출력 | R16 10k 풀다운 |
| E22 M1 | `/CPU_E22_M1` | 5 | 출력 | R17 10k 풀다운 |
| E22 AUX | `/CPU_E22_AUX` | 6 | 입력 | 송신 중 표시 |
| 열선 ARM | `/HEAT_ARM` | 38 | 출력 | Q12→Q15 공통 전원 |
| 열선 CH1 | `/HEAT_IN1` | 13 | 출력 | Q10 AON7524 → J8 |
| 열선 CH2 | `/HEAT_IN2` | 7 | 출력 | Q11 AON7524 → J9 |
| 단 식별 | `/STAGE_ID` | 27 | 입력 | J7, R29 10k 풀업 |
| 부저 | `/BUZZ_GPIO` | 40 | 출력 | Q3 → TMB12A05 |
| LED 적 | `/LED_RED_GPIO` | 32 | 출력 | R18 1k |
| LED 녹 | `/LED_GREEN_GPIO` | 9 | 출력 | R19 1k |
| 배터리 ADC | `/CPU_BAT_ADC` | 26 | 입력(ADC) | R24/R25 = 5.545:1 |
| 5V 감지 | `/MAIN5V_PRESENT` | 39 | 입력 | Q6 |

## 전원 구조

```
J1 (3S / XT60) → F10 15A → Q14 역극성 → HEAT_VIN → Q15 → HEAT_POWER → J8/J9
                └ F1 2A → Q1 역극성 → U3(DFR1015) → 5V_MAIN ┬→ 5V_MCU → U1
                                                             ├→ U2(TLV75533) → 3V3_SENSOR
                                                             ├→ J4 microSD
                                                             └→ J5 E22
```

- `U6`, `U11` (TPS3839G33): 센서 3V3 / MCU 3V3 전압 감시
- `U4`, `U5` (SN74CB3Q3245): 전원이 흔들릴 때 MCU ↔ 센서/SD/무선 신호를
  하드웨어가 알아서 끊는다. 펌웨어가 제어하지 않지만, 전원이 올라온 뒤에
  센서를 초기화해야 하는 이유가 된다.

## 핀 번호를 채울 때 확인할 것

- GPIO 26~37은 내부 Flash/PSRAM이 쓴다 (N32R16V). 외부에 쓰면 안 된다.
- GPIO 0, 3, 45, 46은 strapping 핀이다. **열선 출력에 절대 쓰지 않는다.**
- 열선 게이트에는 하드웨어 10k 풀다운이 이미 있다(R44, R45). 부팅 중
  GPIO가 뜨는 구간에도 OFF로 유지되는지 실측으로 확인한다.
