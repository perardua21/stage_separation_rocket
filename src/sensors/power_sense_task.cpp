#include "sensors/power_sense_task.h"

#include <Arduino.h>

namespace {
constexpr uint32_t kIdlePeriodMs = 100;
// 화약 발사 구간. 전류가 확 흐를 때 배터리 레일이 얼마나 내려앉는지
// 보려면 이 정도는 되어야 한다. 로그는 100 Hz 로 남으므로 매 로그 행에
// 새 전압이 들어간다.
constexpr uint32_t kFastPeriodMs = 10;
}  // namespace

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
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(stats_.fastPowerSampling
                                             ? kFastPeriodMs
                                             : kIdlePeriodMs));
  }
}
