#include "app/flight_controller_app.h"

#include <Arduino.h>

#include "board_pins.h"

namespace {
constexpr BaseType_t kStateCore = 1;
constexpr BaseType_t kIoCore = 0;
}

bool FlightControllerApp::setup(uint32_t nowMs) {
  (void)nowMs;
  Serial.begin(BoardPins::kSerialBaud);
  delay(500);

  actuatorOutput_.begin();
  pinMode(BoardPins::kLedGreen, OUTPUT);
  pinMode(BoardPins::kLedRed, OUTPUT);
  pinMode(BoardPins::kBuzzer, OUTPUT);
  pinMode(BoardPins::kMain5vPresent, INPUT_PULLUP);
  pinMode(BoardPins::kStageId, INPUT_PULLUP);

  const VehicleRole role = digitalRead(BoardPins::kStageId) == LOW
                               ? VehicleRole::Sustainer
                               : VehicleRole::Booster;
  Serial.printf("NURA two-stage bench prototype; role=%s; outputs=BLOCKED\n",
                role == VehicleRole::Booster ? "BOOSTER" : "SUSTAINER");
  Serial.println("10 s baseline calibration; commands: arm | disarm | sep");

  if (!channels_.create()) failSafe("queue allocation");

  // FlightControllerApp is the composition root. Task objects own their
  // internal logic; this method only wires dependencies and starts them.
  fsmTask_.emplace(role, channels_, stats_);
  if (!imuTask_.start(kStateCore) ||
      !barometerTask_.start(kStateCore) ||
      !powerSenseTask_.start(kStateCore) ||
      !fsmTask_->start(kStateCore) ||
      !actuatorTask_.start(kStateCore) ||
      !flightLogTask_.start(kIoCore) ||
      !telemetryTask_.start(kIoCore) ||
      !commandTask_.start(kIoCore)) {
    failSafe("task start");
  }
  return true;
}

void FlightControllerApp::loop(uint32_t nowMs) {
  (void)nowMs;
  // Arduino's loop task is not part of the flight pipeline.
  vTaskDelay(portMAX_DELAY);
}

[[noreturn]] void FlightControllerApp::failSafe(const char* reason) {
  actuatorOutput_.allOff();
  Serial.printf("fatal initialization failure: %s\n", reason);
  abort();
}
