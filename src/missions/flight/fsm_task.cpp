#include "missions/flight/fsm_task.h"

#include <Arduino.h>

#include "board_pins.h"

bool FsmTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "fsm", 6144, this, 5, nullptr, core) ==
         pdPASS;
}

void FsmTask::entry(void* context) { static_cast<FsmTask*>(context)->run(); }

void FsmTask::run() {
  SensorMessage message;
  SensorSample sample;
  uint32_t lastBarometerMs = 0;
  for (;;) {
    if (xQueueReceive(channels_.sensor, &message, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    if (message.source == SensorSource::Barometer) {
      sample.altitudeM = message.altitudeM;
      sample.verticalVelocityMps = message.verticalVelocityMps;
      sample.barometerValid = message.valid;
      lastBarometerMs = message.timestampMs;
      continue;
    }
    if (message.source == SensorSource::Power) {
      if (message.valid) sample.batteryV = message.batteryV;
      continue;
    }

    sample.timestampMs = message.timestampMs;
    sample.accelerationG = message.accelerationG;
    sample.tiltDeg = message.tiltDeg;
    sample.integratedVelocityMps = message.integratedVelocityMps;
    sample.imuValid = message.valid;
    sample.barometerValid = sample.barometerValid &&
                            (message.timestampMs - lastBarometerMs <= 250U);

    // Commands are events. Consume all queued commands, retaining the newest
    // one for this sensor-driven state-machine update.
    ControlCommand pending = ControlCommand::None;
    ControlCommand command;
    while (xQueueReceive(channels_.command, &command, 0) == pdTRUE) {
      pending = command;
    }

    StateMachineUpdate update = machine_.step(sample, pending);
    update.snapshot.droppedLogFrames = stats_.droppedLogFrames;
    update.snapshot.droppedTelemetryFrames = stats_.droppedTelemetryFrames;

    if (update.actuatorCommandValid &&
        xQueueSend(channels_.actuator, &update.actuator, 0) != pdTRUE) {
      // The output task has the fail-safe policy; a full actuator queue is
      // still surfaced through the bench console by subsequent commands.
    }
    if (update.snapshot.state != FlightState::Init &&
        update.snapshot.state != FlightState::Safe &&
        xQueueSend(channels_.log, &update.snapshot, 0) != pdTRUE) {
      ++stats_.droppedLogFrames;
    }
    if (xQueueOverwrite(channels_.telemetry, &update.snapshot) != pdTRUE) {
      ++stats_.droppedTelemetryFrames;
    }

    digitalWrite(BoardPins::kLedGreen,
                 update.snapshot.state == FlightState::Safe ||
                         update.snapshot.state == FlightState::Ground
                     ? HIGH
                     : LOW);
    digitalWrite(BoardPins::kLedRed,
                 update.snapshot.state == FlightState::Recovery ||
                         update.snapshot.state == FlightState::Fault
                     ? HIGH
                     : LOW);
  }
}
