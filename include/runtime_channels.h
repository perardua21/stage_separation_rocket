#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "flight_types.h"

struct RuntimeStats {
  volatile uint32_t droppedLogFrames = 0;
  volatile uint32_t droppedTelemetryFrames = 0;
  volatile uint32_t droppedSensorSamples = 0;
};

struct RuntimeChannels {
  QueueHandle_t sensor = nullptr;
  QueueHandle_t command = nullptr;
  QueueHandle_t actuator = nullptr;
  QueueHandle_t log = nullptr;
  QueueHandle_t telemetry = nullptr;

  bool create() {
    sensor = xQueueCreate(32, sizeof(SensorMessage));
    command = xQueueCreate(8, sizeof(ControlCommand));
    actuator = xQueueCreate(8, sizeof(ActuatorCommand));
    log = xQueueCreate(64, sizeof(FlightSnapshot));
    telemetry = xQueueCreate(1, sizeof(FlightSnapshot));
    return sensor && command && actuator && log && telemetry;
  }
};
