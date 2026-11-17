#include "missions/telemetry/telemetry_task.h"

#include <Arduino.h>

#include "board_pins.h"
#include "missions/logging/flight_log_task.h"

namespace { constexpr uint32_t kTelemetryPeriodMs = 200; }

bool TelemetryTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "telemetry", 4096, this, 2, nullptr,
                                 core) == pdPASS;
}

void TelemetryTask::entry(void* context) {
  static_cast<TelemetryTask*>(context)->run();
}

void TelemetryTask::run() {
  pinMode(BoardPins::kE22M0, OUTPUT);
  pinMode(BoardPins::kE22M1, OUTPUT);
  pinMode(BoardPins::kE22Aux, INPUT);
  digitalWrite(BoardPins::kE22M0, LOW);
  digitalWrite(BoardPins::kE22M1, LOW);
  radio_.begin(BoardPins::kE22Baud, SERIAL_8N1, BoardPins::kE22Rx,
               BoardPins::kE22Tx);

  FlightSnapshot frame;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    // A depth-one overwrite queue is a latest-value mailbox. Telemetry can be
    // slow without blocking or reading mutable FSM state concurrently.
    if (xQueuePeek(channels_.telemetry, &frame, 0) == pdTRUE) {
      writeCsvRow(Serial, frame);
      if (digitalRead(BoardPins::kE22Aux) == HIGH) writeCsvRow(radio_, frame);
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kTelemetryPeriodMs));
  }
}
