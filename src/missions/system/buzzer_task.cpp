#include "missions/system/buzzer_task.h"

#include <Arduino.h>

#include "board_pins.h"

namespace {
constexpr uint32_t kPeriodMs = 50;

void beep(uint32_t onMs) {
  digitalWrite(BoardPins::kBuzzer, HIGH);
  delay(onMs);
  digitalWrite(BoardPins::kBuzzer, LOW);
}
}  // namespace

void BuzzerTask::playRoleCue(VehicleRole role) {
  if (role == VehicleRole::Booster) {
    beep(600);  // 1단: 길게 한 번
  } else {
    beep(120);  // 2단: 짧게 두 번
    delay(150);
    beep(120);
  }
}

bool BuzzerTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "buzzer", 2048, this, 1, nullptr,
                                 core) == pdPASS;
}

void BuzzerTask::entry(void* context) {
  static_cast<BuzzerTask*>(context)->run();
}

void BuzzerTask::run() {
  uint32_t phaseMs = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const FlightState state = static_cast<FlightState>(stats_.currentState);
    bool on = false;

    switch (state) {
      case FlightState::Armed:
        on = phaseMs % 2000 < 80;
        break;
      case FlightState::Ground:
        on = phaseMs % 1000 < 300;
        break;
      case FlightState::Fault:
        on = (phaseMs % 1000) < 500 && (phaseMs % 150) < 75;
        break;
      default:
        on = false;
        break;
    }

    digitalWrite(BoardPins::kBuzzer, on ? HIGH : LOW);
    phaseMs += kPeriodMs;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kPeriodMs));
  }
}
