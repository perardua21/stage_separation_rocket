#pragma once

#include <stdint.h>

namespace BoardPins {

// ESP32-S3-DevKitC-1-N32R16V pin mapping reconstructed from the supplied PCB
// HTML. The comments name the PCB nets so this file can be checked against the
// schematic before hardware tests.
constexpr uint8_t kE22M0 = 4;       // CPU_E22_M0
constexpr uint8_t kE22M1 = 5;       // CPU_E22_M1
constexpr uint8_t kE22Aux = 6;      // CPU_E22_AUX
constexpr uint8_t kHeater2 = 7;     // HEAT_IN2
constexpr uint8_t kBmpSda = 8;      // CPU_BMP_SDA
constexpr uint8_t kBmpScl = 9;      // CPU_BMP_SCL
constexpr uint8_t kBnoCs = 10;      // CPU_BNO_CS_GPIO
constexpr uint8_t kBnoMosi = 11;    // CPU_BNO_MOSI_GPIO
constexpr uint8_t kBnoSck = 12;     // CPU_BNO_SCK_GPIO
constexpr uint8_t kBnoMiso = 13;    // CPU_BNO_MISO
constexpr uint8_t kBnoInt = 14;     // CPU_BNO_INT
constexpr uint8_t kBnoReset = 15;   // BNO_RESET_ASSERT
constexpr uint8_t kLedGreen = 16;   // LED_GREEN_GPIO
constexpr uint8_t kE22Tx = 17;      // CPU_E22_UART_TX
constexpr uint8_t kE22Rx = 18;      // CPU_E22_UART_RX
constexpr uint8_t kBuzzer = 21;     // BUZZ_GPIO
constexpr uint8_t kBatteryAdc = 1;  // CPU_BAT_ADC
// J7 is open on stage 1 (booster) and shunted to GND on stage 2
// (sustainer), per assembly_extra.csv and the handoff BOM.
constexpr uint8_t kStageId = 2;     // STAGE_ID, LOW = sustainer
constexpr uint8_t kHeater1 = 3;     // HEAT_IN1
constexpr uint8_t kSdSck = 39;      // CPU_SD_SCK_GPIO
constexpr uint8_t kSdMosi = 40;     // CPU_SD_MOSI_GPIO
constexpr uint8_t kSdMiso = 41;     // CPU_SD_MISO
constexpr uint8_t kSdCs = 42;       // CPU_SD_CS_GPIO
constexpr uint8_t kLedRed = 38;     // LED_RED_GPIO
constexpr uint8_t kHeaterArm = 48;  // HEAT_ARM, common high-side enable
// Q6 is an open-drain detector. 5 V present turns Q6 on and pulls this input
// LOW. GPIO47 must provide its internal pull-up (native 1.8 V domain).
constexpr uint8_t kMain5vPresent = 47;  // active LOW

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kE22Baud = 9600;
constexpr float kBatteryDividerRatio = 122.0f / 22.0f;

}  // namespace BoardPins
