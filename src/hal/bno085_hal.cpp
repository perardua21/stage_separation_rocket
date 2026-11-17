#include "hal/bno085_hal.h"

#include <SPI.h>

#include "board_pins.h"

namespace {
constexpr uint32_t kReportPeriodUs = 10000;
constexpr uint8_t kMaxEventsPerRead = 16;
}  // namespace

Bno085Hal::Bno085Hal() : device_(BoardPins::kBnoReset) {}

bool Bno085Hal::begin() {
  pinMode(BoardPins::kBnoCs, OUTPUT);
  digitalWrite(BoardPins::kBnoCs, HIGH);
  SPI.begin(BoardPins::kBnoSck, BoardPins::kBnoMiso, BoardPins::kBnoMosi,
            BoardPins::kBnoCs);
  // begin_SPI resets the BNO085 only after its SPI/SHTP callbacks are ready.
  // The vendored driver applies the Q2-inverted reset waveform.
  if (!device_.begin_SPI(BoardPins::kBnoCs, BoardPins::kBnoInt, &SPI)) {
    return false;
  }
  const bool reportsEnabled = enableReports();
  // The reset observed during begin_SPI is expected and has already been
  // handled. Clear it so only later, unexpected sensor resets trigger recovery.
  (void)device_.wasReset();
  return reportsEnabled;
}

bool Bno085Hal::enableReports() {
  return device_.enableReport(SH2_ACCELEROMETER, kReportPeriodUs) &&
         device_.enableReport(SH2_LINEAR_ACCELERATION, kReportPeriodUs) &&
         device_.enableReport(SH2_GAME_ROTATION_VECTOR, kReportPeriodUs);
}

bool Bno085Hal::read(Bno085Reading& reading, uint32_t nowMs) {
  if (device_.wasReset()) {
    latest_ = Bno085Reading{};
    if (!enableReports()) return false;
  }

  sh2_SensorValue_t event;
  bool accelerationUpdated = false;
  // Adafruit_BNO08x::getSensorEvent waits up to 500 ms for INT. Calling it in
  // an unbounded drain loop at a 100 Hz report rate can therefore run forever:
  // another packet arrives before the wait expires. Only service the device
  // while its active-low INT says data is ready, and cap work per task cycle.
  for (uint8_t eventCount = 0;
       eventCount < kMaxEventsPerRead &&
       digitalRead(BoardPins::kBnoInt) == LOW;
       ++eventCount) {
    if (!device_.getSensorEvent(&event)) break;
    switch (event.sensorId) {
      case SH2_ACCELEROMETER:
        latest_.accelXMps2 = event.un.accelerometer.x;
        latest_.accelYMps2 = event.un.accelerometer.y;
        latest_.accelZMps2 = event.un.accelerometer.z;
        accelerationUpdated = true;
        break;
      case SH2_LINEAR_ACCELERATION:
        latest_.linearAccelXMps2 = event.un.linearAcceleration.x;
        latest_.linearAccelYMps2 = event.un.linearAcceleration.y;
        latest_.linearAccelZMps2 = event.un.linearAcceleration.z;
        break;
      case SH2_GAME_ROTATION_VECTOR:
        latest_.quaternionW = event.un.gameRotationVector.real;
        latest_.quaternionX = event.un.gameRotationVector.i;
        latest_.quaternionY = event.un.gameRotationVector.j;
        latest_.quaternionZ = event.un.gameRotationVector.k;
        latest_.orientationValid = true;
        break;
      default:
        break;
    }
  }
  if (!accelerationUpdated) return false;
  latest_.sampleMs = nowMs;
  reading = latest_;
  return true;
}
