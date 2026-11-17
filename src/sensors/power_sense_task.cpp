#include "sensors/power_sense_task.h"

#include <Arduino.h>

namespace { constexpr uint32_t kPeriodMs = 100; }

bool PowerSenseTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "power", 3072, this, 2, nullptr,
                                 core) == pdPASS;
}

void PowerSenseTask::entry(void* context) {
  static_cast<PowerSenseTask*>(context)->run();
}

void PowerSenseTask::run() {
  battery_.begin();
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    SensorMessage message;
    message.source = SensorSource::Power;
    message.timestampMs = millis();
    message.valid = battery_.read(message.batteryV);
    if (xQueueSend(channels_.sensor, &message, 0) != pdTRUE) {
      ++stats_.droppedSensorSamples;
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kPeriodMs));
  }
}
