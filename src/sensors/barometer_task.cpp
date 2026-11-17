#include "sensors/barometer_task.h"

#include <Arduino.h>

namespace {
constexpr uint32_t kPeriodMs = 50;
constexpr uint32_t kBaselineDurationMs = 10000;
}

bool BarometerTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "barometer", 4096, this, 3, nullptr,
                                 core) == pdPASS;
}

void BarometerTask::entry(void* context) {
  static_cast<BarometerTask*>(context)->run();
}

void BarometerTask::run() {
  const bool ready = barometer_.begin();
  const uint32_t baselineStartMs = millis();
  uint32_t baselineSamples = 0;
  float baselineSumM = 0.0f;
  float baselineAltitudeM = 0.0f;
  float filteredAltitudeM = 0.0f;
  float previousAltitudeM = 0.0f;
  uint32_t previousSampleMs = 0;

  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const uint32_t nowMs = millis();
    Bmp388Reading reading;
    if (ready && barometer_.read(reading, nowMs)) {
      bool valid = false;
      float verticalVelocityMps = 0.0f;
      if (nowMs - baselineStartMs < kBaselineDurationMs) {
        baselineSumM += reading.absoluteAltitudeM;
        ++baselineSamples;
      } else if (baselineSamples > 0) {
        if (baselineAltitudeM == 0.0f) {
          baselineAltitudeM = baselineSumM / baselineSamples;
        }
        const float relativeAltitudeM =
            reading.absoluteAltitudeM - baselineAltitudeM;
        filteredAltitudeM =
            0.25f * relativeAltitudeM + 0.75f * filteredAltitudeM;
        if (previousSampleMs != 0 && reading.sampleMs > previousSampleMs) {
          verticalVelocityMps =
              (filteredAltitudeM - previousAltitudeM) /
              ((reading.sampleMs - previousSampleMs) / 1000.0f);
        }
        previousAltitudeM = filteredAltitudeM;
        previousSampleMs = reading.sampleMs;
        valid = true;
      }

      SensorMessage message;
      message.source = SensorSource::Barometer;
      message.timestampMs = reading.sampleMs;
      message.altitudeM = filteredAltitudeM;
      message.verticalVelocityMps = verticalVelocityMps;
      message.valid = valid;
      if (xQueueSend(channels_.sensor, &message, 0) != pdTRUE) {
        ++stats_.droppedSensorSamples;
      }
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kPeriodMs));
  }
}
