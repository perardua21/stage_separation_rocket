#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>

// Standalone BNO085 test. This file intentionally does not include or call
// board_pins.h, the project HAL, FreeRTOS application tasks, or flight code.
namespace {
constexpr uint8_t kPinCs = 10;
constexpr uint8_t kPinMosi = 11;
constexpr uint8_t kPinSck = 12;
constexpr uint8_t kPinMiso = 13;
constexpr uint8_t kPinInt = 14;
constexpr uint8_t kPinReset = 15;

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kReportIntervalUs = 10000;
constexpr uint32_t kPrintIntervalMs = 100;
constexpr uint8_t kMaxEventsPerLoop = 16;

// Pass GPIO15 to the driver so the decisive reset happens after the SPI HAL
// callbacks are installed. The vendored driver handles this board's Q2-
// inverted reset polarity through the project-wide board build flag.
Adafruit_BNO08x bno085(kPinReset);
bool initialized = false;
uint32_t lastPrintMs = 0;
uint32_t lastStatusMs = 0;
const char* failureReason = "setup has not completed";

struct LatestValues {
  float accelX = 0.0f;
  float accelY = 0.0f;
  float accelZ = 0.0f;
  float linearX = 0.0f;
  float linearY = 0.0f;
  float linearZ = 0.0f;
  float quatW = 1.0f;
  float quatX = 0.0f;
  float quatY = 0.0f;
  float quatZ = 0.0f;
  bool accelValid = false;
  bool linearValid = false;
  bool quaternionValid = false;
};

LatestValues latest;

bool tryResetPolarity(uint8_t assertedLevel, const char* polarityName) {
  const uint8_t releasedLevel = assertedLevel == LOW ? HIGH : LOW;
  pinMode(kPinReset, OUTPUT);
  Serial.printf("[RESET %s] assert GPIO15=%s for 20 ms\n", polarityName,
                assertedLevel == LOW ? "LOW" : "HIGH");
  digitalWrite(kPinReset, assertedLevel);
  delay(20);
  digitalWrite(kPinReset, releasedLevel);
  Serial.printf("[RESET %s] release GPIO15=%s; waiting for INT LOW\n",
                polarityName, releasedLevel == LOW ? "LOW" : "HIGH");

  // BNO085 INT is active-low. After a valid reset/boot it should request an
  // SHTP transaction and remain LOW until the host services it.
  const uint32_t started = millis();
  while (millis() - started < 1500) {
    if (digitalRead(kPinInt) == LOW) {
      Serial.printf("[PASS] %s produced INT LOW after %lu ms\n", polarityName,
                    static_cast<unsigned long>(millis() - started));
      return true;
    }
    delay(1);
  }
  return false;
}

bool enableReports() {
  const bool accel =
      bno085.enableReport(SH2_ACCELEROMETER, kReportIntervalUs);
  const bool linear =
      bno085.enableReport(SH2_LINEAR_ACCELERATION, kReportIntervalUs);
  const bool quaternion =
      bno085.enableReport(SH2_GAME_ROTATION_VECTOR, kReportIntervalUs);

  Serial.printf("[REPORT] accel=%s linear_accel=%s game_quaternion=%s\n",
                accel ? "OK" : "FAIL", linear ? "OK" : "FAIL",
                quaternion ? "OK" : "FAIL");
  return accel && linear && quaternion;
}

void printLatest() {
  Serial.printf(
      "[DATA %10lu ms] accel[m/s2]=(%.3f, %.3f, %.3f) "
      "linear[m/s2]=(%.3f, %.3f, %.3f) quat[wxyz]=(%.5f, %.5f, %.5f, %.5f) "
      "valid[A/L/Q]=%u/%u/%u INT=%s\n",
      static_cast<unsigned long>(millis()), latest.accelX, latest.accelY,
      latest.accelZ, latest.linearX, latest.linearY, latest.linearZ,
      latest.quatW, latest.quatX, latest.quatY, latest.quatZ,
      latest.accelValid, latest.linearValid, latest.quaternionValid,
      digitalRead(kPinInt) == LOW ? "LOW" : "HIGH");
}
}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  const uint32_t serialStarted = millis();
  while (!Serial && millis() - serialStarted < 3000) delay(10);
  delay(300);

  Serial.println();
  Serial.println("========== BNO085 STANDALONE SPI TEST ==========");
  Serial.println("No project HAL/tasks/flight code are used.");
  Serial.println("Pins: CS=10 MOSI=11 SCK=12 MISO=13 INT=14 RESET=15");
  Serial.println("Reset polarity A/B test: LOW->HIGH first, then HIGH->LOW");
  Serial.println("SHTP write attempts are limited to 8.");
  Serial.println("begin_SPI will issue a second, driver-synchronized reset.");

  pinMode(kPinCs, OUTPUT);
  digitalWrite(kPinCs, HIGH);
  pinMode(kPinInt, INPUT_PULLUP);
  SPI.begin(kPinSck, kPinMiso, kPinMosi, kPinCs);

  Serial.println("[RUN] reset polarity A: assert LOW, release HIGH");
  bool interruptSeen =
      tryResetPolarity(LOW, "A: LOW(assert) -> HIGH(release)");
  if (!interruptSeen) {
    Serial.println("[WARN] polarity A produced no INT; trying inverted polarity B");
    interruptSeen =
        tryResetPolarity(HIGH, "B: HIGH(assert) -> LOW(release)");
  }
  Serial.printf("[INFO] after reset: RESET=%s INT=%s CS=%s\n",
                digitalRead(kPinReset) == LOW ? "LOW" : "HIGH",
                digitalRead(kPinInt) == LOW ? "LOW" : "HIGH",
                digitalRead(kPinCs) == LOW ? "LOW" : "HIGH");

  if (!interruptSeen) {
    failureReason = "INT stayed HIGH with both reset polarities";
    Serial.printf("[FAIL] %s\n", failureReason);
    Serial.println("begin_SPI was not called because the BNO085 did not signal boot readiness.");
    return;
  }

  Serial.println("[RUN] begin_SPI");
  Serial.flush();
  if (!bno085.begin_SPI(kPinCs, kPinInt, &SPI)) {
    failureReason = "begin_SPI returned false after INT asserted";
    Serial.println("[FAIL] begin_SPI returned false");
    Serial.println("Check BNO085 power, reset, INT, CS, SCK, MISO, and MOSI.");
    return;
  }
  Serial.println("[PASS] begin_SPI");

  if (!enableReports()) {
    failureReason = "one or more sensor reports could not be enabled";
    Serial.println("[FAIL] one or more sensor reports could not be enabled");
    return;
  }

  initialized = true;
  Serial.println("[PASS] BNO085 initialized; streaming values every 100 ms");
  Serial.println("================================================");
}

void loop() {
  if (!initialized) {
    if (millis() - lastStatusMs >= 2000) {
      lastStatusMs = millis();
      Serial.printf("[FAILED/IDLE] reason=%s | RESET=%s INT=%s CS=%s; press RESET to retry\n",
                    failureReason,
                    digitalRead(kPinReset) == LOW ? "LOW" : "HIGH",
                    digitalRead(kPinInt) == LOW ? "LOW" : "HIGH",
                    digitalRead(kPinCs) == LOW ? "LOW" : "HIGH");
    }
    delay(10);
    return;
  }

  if (bno085.wasReset()) {
    Serial.println("[WARN] BNO085 reset detected; re-enabling reports");
    if (!enableReports()) {
      initialized = false;
      Serial.println("[FAIL] report recovery failed");
      return;
    }
  }

  sh2_SensorValue_t event;
  for (uint8_t eventCount = 0;
       eventCount < kMaxEventsPerLoop && digitalRead(kPinInt) == LOW;
       ++eventCount) {
    if (!bno085.getSensorEvent(&event)) break;
    switch (event.sensorId) {
      case SH2_ACCELEROMETER:
        latest.accelX = event.un.accelerometer.x;
        latest.accelY = event.un.accelerometer.y;
        latest.accelZ = event.un.accelerometer.z;
        latest.accelValid = true;
        break;
      case SH2_LINEAR_ACCELERATION:
        latest.linearX = event.un.linearAcceleration.x;
        latest.linearY = event.un.linearAcceleration.y;
        latest.linearZ = event.un.linearAcceleration.z;
        latest.linearValid = true;
        break;
      case SH2_GAME_ROTATION_VECTOR:
        latest.quatW = event.un.gameRotationVector.real;
        latest.quatX = event.un.gameRotationVector.i;
        latest.quatY = event.un.gameRotationVector.j;
        latest.quatZ = event.un.gameRotationVector.k;
        latest.quaternionValid = true;
        break;
      default:
        break;
    }
  }

  if (millis() - lastPrintMs >= kPrintIntervalMs) {
    lastPrintMs = millis();
    printLatest();
  }
  delay(1);
}
