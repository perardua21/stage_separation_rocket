#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <math.h>

#include "board_pins.h"
#include "hal/battery_voltage_hal.h"
#include "hal/bmp388_hal.h"
#include "hal/bno085_hal.h"

#if defined(NURA_PCB_BRINGUP)

#if !defined(NURA_BENCH_SAFE)
#error "PCB bring-up requires the safe-output build flag."
#endif

namespace {
constexpr uint32_t kSensorTimeoutMs = 3000;
constexpr uint32_t kImuTaskTimeoutMs = 8000;
constexpr uint32_t kVisualTestMs = 500;
constexpr uint32_t kSdSpiHz = 4000000;
constexpr uint8_t kSdMountAttempts = 5;
constexpr char kSdTestPath[] = "/pcb_bringup_test.txt";
constexpr char kSdTestPayload[] = "NURA_PCB_BRINGUP_OK";

enum class Result : uint8_t { Pass, Warn, Fail, Skip };

struct Summary {
  uint16_t passed = 0;
  uint16_t warned = 0;
  uint16_t failed = 0;
  uint16_t skipped = 0;
};

Summary summary;
Bno085Hal imu;
Bmp388Hal barometer;
BatteryVoltageHal battery;
SPIClass sdSpi(HSPI);
HardwareSerial e22(1);

struct ImuProbeReport {
  Result result = Result::Fail;
  char detail[160] = {};
};

struct ImuProbeContext {
  QueueHandle_t resultQueue = nullptr;
};

const char* label(Result result) {
  switch (result) {
    case Result::Pass: return "PASS";
    case Result::Warn: return "WARN";
    case Result::Fail: return "FAIL";
    case Result::Skip: return "SKIP";
  }
  return "FAIL";
}

void report(Result result, const char* item, const char* detail) {
  switch (result) {
    case Result::Pass: ++summary.passed; break;
    case Result::Warn: ++summary.warned; break;
    case Result::Fail: ++summary.failed; break;
    case Result::Skip: ++summary.skipped; break;
  }
  Serial.printf("[%s] %-18s %s\n", label(result), item, detail);
}

void forcePyroSafe() {
  pinMode(BoardPins::kHeater1, OUTPUT);
  pinMode(BoardPins::kHeater2, OUTPUT);
  pinMode(BoardPins::kHeaterArm, OUTPUT);
  digitalWrite(BoardPins::kHeater1, LOW);
  digitalWrite(BoardPins::kHeater2, LOW);
  digitalWrite(BoardPins::kHeaterArm, LOW);
}

void testDigitalInputs() {
  pinMode(BoardPins::kStageId, INPUT_PULLUP);
  pinMode(BoardPins::kMain5vPresent, INPUT_PULLUP);
  const bool stage2 = digitalRead(BoardPins::kStageId) == LOW;
  const bool main5vPresent = digitalRead(BoardPins::kMain5vPresent) == LOW;
  Serial.printf("[INFO] stage_id           %s (J7 %s)\n",
                stage2 ? "SUSTAINER" : "BOOSTER",
                stage2 ? "closed; stage 2" : "open; stage 1");
  report(main5vPresent ? Result::Pass : Result::Warn,
         "main_5v_present",
         main5vPresent
             ? "active-low detector is LOW; 5 V rail detected"
             : "detector is HIGH; 5 V rail not detected, check Q6/R27");
}

void testIndicators() {
  pinMode(BoardPins::kLedGreen, OUTPUT);
  pinMode(BoardPins::kLedRed, OUTPUT);
  pinMode(BoardPins::kBuzzer, OUTPUT);

  digitalWrite(BoardPins::kLedGreen, HIGH);
  delay(kVisualTestMs);
  digitalWrite(BoardPins::kLedGreen, LOW);
  digitalWrite(BoardPins::kLedRed, HIGH);
  delay(kVisualTestMs);
  digitalWrite(BoardPins::kLedRed, LOW);
  digitalWrite(BoardPins::kBuzzer, HIGH);
  delay(200);
  digitalWrite(BoardPins::kBuzzer, LOW);
  report(Result::Warn, "led_and_buzzer",
         "confirm green, red, and buzzer operation visually");
}

void testBattery() {
  battery.begin();
  float voltageV = 0.0f;
  if (!battery.read(voltageV) || !isfinite(voltageV)) {
    report(Result::Fail, "battery_adc", "read failed or non-finite");
    return;
  }
  char detail[80];
  snprintf(detail, sizeof(detail), "measured %.3f V; compare with a multimeter",
           voltageV);
  const Result result = voltageV > 0.05f && voltageV < 30.0f
                            ? Result::Pass
                            : Result::Warn;
  report(result, "battery_adc", detail);
}

void imuProbeTask(void* argument) {
  auto* context = static_cast<ImuProbeContext*>(argument);
  ImuProbeReport reportData;

  if (!imu.begin()) {
    reportData.result = Result::Fail;
    snprintf(reportData.detail, sizeof(reportData.detail),
             "SPI initialization failed; check TP7 RST_N, TP6 INT, CS/SCK/MISO/MOSI");
    xQueueSend(context->resultQueue, &reportData, portMAX_DELAY);
    vTaskSuspend(nullptr);
  }

  Bno085Reading reading;
  const uint32_t started = millis();
  bool accelerationReceived = false;
  while (millis() - started < kSensorTimeoutMs) {
    Bno085Reading candidate;
    if (imu.read(candidate, millis())) {
      reading = candidate;
      accelerationReceived = true;
      // Accelerometer and game-rotation reports are separate SHTP packets.
      // Do not finish on the first acceleration packet; allow the quaternion
      // report to arrive before deciding whether attitude is valid.
      if (reading.orientationValid) break;
    }
    delay(10);
  }
  if (!accelerationReceived) {
    reportData.result = Result::Fail;
    snprintf(reportData.detail, sizeof(reportData.detail),
             "initialized but no sample received within %lu ms",
             static_cast<unsigned long>(kSensorTimeoutMs));
    xQueueSend(context->resultQueue, &reportData, portMAX_DELAY);
    vTaskSuspend(nullptr);
  }

  const float magnitude = sqrtf(reading.accelXMps2 * reading.accelXMps2 +
                                reading.accelYMps2 * reading.accelYMps2 +
                                reading.accelZMps2 * reading.accelZMps2);
  snprintf(reportData.detail, sizeof(reportData.detail),
           "accel=(%.2f, %.2f, %.2f) m/s2, norm=%.2f, attitude=%s",
           reading.accelXMps2, reading.accelYMps2, reading.accelZMps2,
           magnitude, reading.orientationValid ? "valid" : "not-yet-valid");
  const bool plausible = isfinite(magnitude) && magnitude > 1.0f &&
                         magnitude < 30.0f && reading.orientationValid;
  reportData.result = plausible ? Result::Pass : Result::Warn;
  xQueueSend(context->resultQueue, &reportData, portMAX_DELAY);
  vTaskSuspend(nullptr);
}

void testImu() {
  QueueHandle_t resultQueue = xQueueCreate(1, sizeof(ImuProbeReport));
  if (resultQueue == nullptr) {
    report(Result::Fail, "BNO085", "could not allocate diagnostic result queue");
    return;
  }

  ImuProbeContext context{resultQueue};
  TaskHandle_t task = nullptr;
  const BaseType_t created = xTaskCreatePinnedToCore(
      imuProbeTask, "bno085_probe", 8192, &context, 1, &task, 0);
  if (created != pdPASS || task == nullptr) {
    vQueueDelete(resultQueue);
    report(Result::Fail, "BNO085", "could not start isolated diagnostic task");
    return;
  }

  ImuProbeReport probeReport;
  const BaseType_t received = xQueueReceive(
      resultQueue, &probeReport, pdMS_TO_TICKS(kImuTaskTimeoutMs));

  // The worker suspends itself after posting its result, so deleting it here
  // is safe. If the vendor driver is stuck, this also prevents one bad sensor
  // from blocking all subsequent PCB checks.
  vTaskDelete(task);
  vQueueDelete(resultQueue);

  if (received != pdTRUE) {
    const bool intWasLow = digitalRead(BoardPins::kBnoInt) == LOW;
    pinMode(BoardPins::kBnoCs, OUTPUT);
    digitalWrite(BoardPins::kBnoCs, HIGH);
    pinMode(BoardPins::kBnoReset, OUTPUT);
    digitalWrite(BoardPins::kBnoReset, HIGH);  // Q2 on: hold BNO085 in reset
    char detail[160];
    snprintf(detail, sizeof(detail),
             "initialization timed out after %lu ms (INT=%s); held in reset; check TP7/TP6 and SPI soldering",
             static_cast<unsigned long>(kImuTaskTimeoutMs),
             intWasLow ? "LOW" : "HIGH");
    report(Result::Fail, "BNO085", detail);
    return;
  }

  report(probeReport.result, "BNO085", probeReport.detail);
}

void testBarometer() {
  if (!barometer.begin()) {
    char detail[96];
    snprintf(detail, sizeof(detail),
             "I2C init failed or wrong chip at 0x77 (chip ID=0x%02X; BMP388=0x50)",
             barometer.chipId());
    report(Result::Fail, "BMP388", detail);
    return;
  }

  Bmp388Reading reading;
  const uint32_t started = millis();
  bool received = false;
  while (millis() - started < kSensorTimeoutMs) {
    if (barometer.read(reading, millis())) {
      received = true;
      break;
    }
    delay(20);
  }
  if (!received) {
    report(Result::Fail, "BMP388", "initialized but no sample received");
    return;
  }

  char detail[120];
  snprintf(detail, sizeof(detail), "pressure=%.1f Pa, temp=%.2f C, altitude=%.1f m",
           reading.pressurePa, reading.temperatureC,
           reading.absoluteAltitudeM);
  const bool plausible = isfinite(reading.pressurePa) &&
                         isfinite(reading.temperatureC) &&
                         reading.pressurePa > 30000.0f &&
                         reading.pressurePa < 110000.0f &&
                         reading.temperatureC > -40.0f &&
                         reading.temperatureC < 85.0f;
  report(plausible ? Result::Pass : Result::Warn, "BMP388", detail);
}

void testSdCard() {
  sdSpi.begin(BoardPins::kSdSck, BoardPins::kSdMiso, BoardPins::kSdMosi,
              BoardPins::kSdCs);
  bool mounted = false;
  uint8_t attempts = 0;
  for (; attempts < kSdMountAttempts; ++attempts) {
    if (SD.begin(BoardPins::kSdCs, sdSpi, kSdSpiHz)) {
      mounted = true;
      break;
    }
    SD.end();
    delay(250);
  }
  if (!mounted) {
    char detail[96];
    snprintf(detail, sizeof(detail),
             "mount failed after %u attempts at %lu Hz; check card/socket/SPI",
             kSdMountAttempts, static_cast<unsigned long>(kSdSpiHz));
    report(Result::Fail, "microSD", detail);
    return;
  }

  SD.remove(kSdTestPath);
  File file = SD.open(kSdTestPath, FILE_WRITE);
  if (!file) {
    report(Result::Fail, "microSD", "test file open failed");
    return;
  }
  file.print(kSdTestPayload);
  file.flush();
  file.close();

  file = SD.open(kSdTestPath, FILE_READ);
  String actual;
  while (file && file.available()) actual += static_cast<char>(file.read());
  if (file) file.close();
  const bool matched = actual == kSdTestPayload;
  report(matched ? Result::Pass : Result::Fail, "microSD",
         matched ? "write/read verification passed"
                 : "read-back payload mismatch");
}

void testE22() {
  pinMode(BoardPins::kE22M0, OUTPUT);
  pinMode(BoardPins::kE22M1, OUTPUT);
  pinMode(BoardPins::kE22Aux, INPUT);
  digitalWrite(BoardPins::kE22M0, LOW);
  digitalWrite(BoardPins::kE22M1, LOW);
  e22.begin(BoardPins::kE22Baud, SERIAL_8N1, BoardPins::kE22Rx,
            BoardPins::kE22Tx);
  delay(100);
  const bool auxHigh = digitalRead(BoardPins::kE22Aux) == HIGH;
  e22.println("NURA PCB BRINGUP UART TEST");
  e22.flush();
  report(auxHigh ? Result::Warn : Result::Fail, "E22_UART",
         auxHigh
             ? "AUX HIGH and test bytes sent; verify with a peer radio"
             : "AUX LOW; check module power, mode pins, and soldering");
}

void printSummary() {
  report(Result::Skip, "heater_outputs",
         "HEAT_ARM/IN1/IN2 held LOW; test only with dummy loads");
  Serial.println();
  Serial.println("========== PCB BRING-UP SUMMARY ==========");
  Serial.printf("PASS=%u WARN=%u FAIL=%u SKIP=%u\n", summary.passed,
                summary.warned, summary.failed, summary.skipped);
  Serial.println("WARN requires visual or external-instrument confirmation.");
  Serial.println("Do not connect igniters or deployment charges.");
  Serial.println("==========================================");
}
}  // namespace

void setup() {
  forcePyroSafe();
  Serial.begin(BoardPins::kSerialBaud);
  const uint32_t waitStarted = millis();
  while (!Serial && millis() - waitStarted < 3000) delay(10);
  delay(300);

  Serial.println();
  Serial.println("NURA PCB bring-up diagnostic (SAFE OUTPUTS)");
  Serial.println("Keep all igniters and deployment devices disconnected.");
  Serial.println();

  Serial.println("[RUN] digital inputs");
  Serial.flush();
  testDigitalInputs();
  Serial.println("[RUN] LEDs and buzzer");
  Serial.flush();
  testIndicators();
  Serial.println("[RUN] battery ADC");
  Serial.flush();
  testBattery();
  Serial.println("[RUN] BNO085 SPI");
  Serial.flush();
  testImu();
  Serial.println("[RUN] BMP388 I2C (expected chip ID 0x50)");
  Serial.flush();
  testBarometer();
  Serial.println("[RUN] microSD SPI");
  Serial.flush();
  testSdCard();
  Serial.println("[RUN] E22 UART");
  Serial.flush();
  testE22();
  printSummary();
}

void loop() {
  forcePyroSafe();
  delay(1000);
}

#endif  // NURA_PCB_BRINGUP
