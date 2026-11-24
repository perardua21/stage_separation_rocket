#include "sensors/barometer_task.h"

#include <Arduino.h>

namespace {
constexpr uint32_t kPeriodMs = 50;
constexpr uint32_t kBaselineDurationMs = 10000;
// ARM 때 다시 영점을 잡을 때는 센서가 이미 안정된 상태이므로 2 초면 된다.
constexpr uint32_t kRezeroDurationMs = 2000;
}  // namespace

bool BarometerTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "barometer", 4096, this, 3, nullptr,
                                 core) == pdPASS;
}

void BarometerTask::entry(void* context) {
  static_cast<BarometerTask*>(context)->run();
}

void BarometerTask::run() {
  const bool ready = barometer_.begin();

  // 1) 전원 투입 직후의 초기 영점. 이게 끝날 때까지 기압은 invalid 다.
  const uint32_t initialStartMs = millis();
  bool haveBaseline = false;
  uint32_t initialSamples = 0;
  float initialSumM = 0.0f;
  float baselineAltitudeM = 0.0f;

  // 2) ARM 때의 재영점. 수집하는 동안에도 직전 영점으로 계속 valid 값을
  //    내보낸다. 새 평균이 완성되는 순간에만 영점을 교체한다. 그래서
  //    재영점 중에 발사되더라도 고도가 invalid 로 비는 구간이 없다.
  bool rezeroing = false;
  uint32_t rezeroStartMs = 0;
  uint32_t rezeroSamples = 0;
  float rezeroSumM = 0.0f;

  float filteredAltitudeM = 0.0f;
  float previousAltitudeM = 0.0f;
  uint32_t previousSampleMs = 0;

  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const uint32_t nowMs = millis();

    if (stats_.rezeroBarometer) {
      stats_.rezeroBarometer = false;
      if (haveBaseline) {
        rezeroing = true;
        rezeroStartMs = nowMs;
        rezeroSamples = 0;
        rezeroSumM = 0.0f;
        Serial.println("[BARO] re-zeroing (2 s)");
      }
    }
    if (stats_.abortRezeroBarometer) {
      stats_.abortRezeroBarometer = false;
      if (rezeroing) {
        rezeroing = false;
        Serial.println("[BARO] re-zero aborted (launch); keeping old zero");
      }
    }

    Bmp388Reading reading;
    if (ready && barometer_.read(reading, nowMs)) {
      bool valid = false;
      float verticalVelocityMps = 0.0f;

      if (!haveBaseline) {
        initialSumM += reading.absoluteAltitudeM;
        ++initialSamples;
        if (nowMs - initialStartMs >= kBaselineDurationMs &&
            initialSamples > 0) {
          baselineAltitudeM = initialSumM / initialSamples;
          haveBaseline = true;
          Serial.printf("[BARO] zero = %.2f m abs\n", baselineAltitudeM);
        }
      } else {
        if (rezeroing) {
          rezeroSumM += reading.absoluteAltitudeM;
          ++rezeroSamples;
          if (nowMs - rezeroStartMs >= kRezeroDurationMs && rezeroSamples > 0) {
            baselineAltitudeM = rezeroSumM / rezeroSamples;
            rezeroing = false;
            // 영점이 튀므로 필터와 속도 이력을 같이 초기화한다. 그러지
            // 않으면 교체 직후 한 샘플에 큰 가짜 수직속도가 나온다.
            filteredAltitudeM = reading.absoluteAltitudeM - baselineAltitudeM;
            previousAltitudeM = filteredAltitudeM;
            previousSampleMs = 0;
            Serial.printf("[BARO] re-zero done = %.2f m abs\n",
                          baselineAltitudeM);
          }
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
