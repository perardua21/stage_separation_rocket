/**
 * @file board_pins.h
 * @brief TUSI Economic Final PCB 핀 정의.
 *
 * 넷 이름은 PCB 부품표에서 그대로 가져왔다.
 * GPIO 번호는 아직 모른다. 부품표에는 ESP32-S3-DevKitC-1 모듈의
 * 물리 핀 번호(1~44)만 있고 GPIO 매핑이 없다.
 * KiCad 회로도에서 확인한 뒤 PIN_UNASSIGNED를 바꿀 것.
 */
#pragma once
#include <stdint.h>

#define PIN_UNASSIGNED ((int)-1)

/* ── BNO085 자세 센서 (SPI) ────────────────────────────────
 * 모듈 핀 16=CS, 17=MOSI, 18=SCK, 19=MISO, 20=INT, 8=RESET
 * R3/R4/R5 33R 직렬, R6 10k CS 풀업, Q2가 리셋 구동 */
#define PIN_BNO_CS      PIN_UNASSIGNED  /* TODO: /CPU_BNO_CS_GPIO  (모듈 16) */
#define PIN_BNO_MOSI    PIN_UNASSIGNED  /* TODO: /CPU_BNO_MOSI_GPIO(모듈 17) */
#define PIN_BNO_SCK     PIN_UNASSIGNED  /* TODO: /CPU_BNO_SCK_GPIO (모듈 18) */
#define PIN_BNO_MISO    PIN_UNASSIGNED  /* TODO: /CPU_BNO_MISO     (모듈 19) */
#define PIN_BNO_INT     PIN_UNASSIGNED  /* TODO: /CPU_BNO_INT      (모듈 20) */
#define PIN_BNO_RESET   PIN_UNASSIGNED  /* TODO: /BNO_RESET_ASSERT (모듈 8), Q2 구동이라 극성 확인 */

/* ── BMP390 기압계 (I2C, 주소 0x77) ───────────────────────
 * 모듈 핀 12=SDA, 15=SCL. 모듈에 풀업 내장 */
#define PIN_BMP_SDA     PIN_UNASSIGNED  /* TODO: /CPU_BMP_SDA (모듈 12) */
#define PIN_BMP_SCL     PIN_UNASSIGNED  /* TODO: /CPU_BMP_SCL (모듈 15) */
#define BMP390_I2C_ADDR 0x77

/* ── microSD (SPI, BNO와 별도 핀) ──────────────────────────
 * 모듈 핀 28=CS, 29=MISO, 30=MOSI, 31=SCK. R12~R14 33R, R15 10k CS 풀업 */
#define PIN_SD_CS       PIN_UNASSIGNED  /* TODO: /CPU_SD_CS_GPIO   (모듈 28) */
#define PIN_SD_MISO     PIN_UNASSIGNED  /* TODO: /CPU_SD_MISO      (모듈 29) */
#define PIN_SD_MOSI     PIN_UNASSIGNED  /* TODO: /CPU_SD_MOSI_GPIO (모듈 30) */
#define PIN_SD_SCK      PIN_UNASSIGNED  /* TODO: /CPU_SD_SCK_GPIO  (모듈 31) */

/* ── E22-900T30D-V2 무선 (UART + 모드핀) ──────────────────
 * 모듈 핀 10=TX, 11=RX, 4=M0, 5=M1, 6=AUX
 * M0/M1에 10k 풀다운(R16,R17) → 기본 00 = 전송 모드 */
#define PIN_E22_TX      PIN_UNASSIGNED  /* TODO: /CPU_E22_UART_TX (모듈 10) */
#define PIN_E22_RX      PIN_UNASSIGNED  /* TODO: /CPU_E22_UART_RX (모듈 11) */
#define PIN_E22_M0      PIN_UNASSIGNED  /* TODO: /CPU_E22_M0  (모듈 4) */
#define PIN_E22_M1      PIN_UNASSIGNED  /* TODO: /CPU_E22_M1  (모듈 5) */
#define PIN_E22_AUX     PIN_UNASSIGNED  /* TODO: /CPU_E22_AUX (모듈 6) */

/* ── 열선 출력 ─────────────────────────────────────────────
 * HEAT_ARM(모듈 38)이 공통 전원 스위치(Q15 SiRA99DP)를 연다.
 * HEAT_IN1(13) / HEAT_IN2(7)이 각 채널 MOSFET(Q10/Q11 AON7524)을 연다.
 * 게이트에 10k 풀다운(R44,R45) + 100R 직렬(R42,R43)이 있어 부팅 시 OFF.
 * ARM이 LOW면 채널을 올려도 전류가 흐르지 않는다 = 2중 차단. */
#define PIN_HEAT_ARM    PIN_UNASSIGNED  /* TODO: /HEAT_ARM (모듈 38) */
#define PIN_HEAT_CH1    PIN_UNASSIGNED  /* TODO: /HEAT_IN1 (모듈 13) → J8 */
#define PIN_HEAT_CH2    PIN_UNASSIGNED  /* TODO: /HEAT_IN2 (모듈 7)  → J9 */

/* ── 단 식별 스트랩 (J7) ──────────────────────────────────
 * R29 10k 풀업. 점퍼를 꽂으면 GND로 떨어진다. */
#define PIN_STAGE_ID    PIN_UNASSIGNED  /* TODO: /STAGE_ID (모듈 27) */

/* ── 표시 ──────────────────────────────────────────────── */
#define PIN_BUZZER      PIN_UNASSIGNED  /* TODO: /BUZZ_GPIO      (모듈 40), Q3 구동 */
#define PIN_LED_RED     PIN_UNASSIGNED  /* TODO: /LED_RED_GPIO   (모듈 32) */
#define PIN_LED_GREEN   PIN_UNASSIGNED  /* TODO: /LED_GREEN_GPIO (모듈 9) */

/* ── 전원 감시 ─────────────────────────────────────────────
 * 배터리 분압 R24 100k / R25 22k → 비 = (100+22)/22 = 5.545
 * Q4/Q5가 분압을 켜고 꺼서 대기 전류를 줄인다. U13 TMUX1511이 ADC를 보호. */
#define PIN_BATT_ADC    PIN_UNASSIGNED  /* TODO: /CPU_BAT_ADC (모듈 26) */
#define PIN_5V_PRESENT  PIN_UNASSIGNED  /* TODO: /MAIN5V_PRESENT (모듈 39) */
#define BATT_DIVIDER_RATIO  5.545f

/* ── ESP32-S3-WROOM 계열 금지 핀 (참고) ───────────────────
 * 내부 Flash/PSRAM: GPIO 26~37
 * Strapping: GPIO 0, 3, 45, 46 — 열선 출력에 절대 쓰지 말 것
 * GPIO 번호를 채울 때 위 핀과 겹치지 않는지 반드시 확인한다. */
