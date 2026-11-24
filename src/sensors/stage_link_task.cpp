#include "sensors/stage_link_task.h"

#include <Arduino.h>

#include "board_pins.h"
#include "flight_config.h"

namespace {
constexpr uint32_t kPeriodMs = 10;
constexpr uint32_t kRepeatMs = 1000;
}  // namespace

bool StageLinkTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "stage_link", 2560, this, 4, nullptr,
                                 core) == pdPASS;
}

void StageLinkTask::entry(void* context) {
  static_cast<StageLinkTask*>(context)->run();
}

void StageLinkTask::run() {
  // 부팅 확인. 이 시간 내내 LOW 여야 두 단이 실제로 붙어 있는 것이다.
  bool linkPresent = true;
  const uint32_t checkStartMs = millis();
  while (millis() - checkStartMs < FlightConfig::kStageLinkBootCheckMs) {
    if (digitalRead(BoardPins::kStageId) == HIGH) {
      linkPresent = false;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stats_.stageLinkPresentAtBoot = linkPresent;
  if (!linkPresent) {
    // 벤치에서 점퍼 없이 돌리는 정상적인 상황이기도 하다. 다만 발사 전
    // 점검에서는 이 메시지가 보이면 안 된다.
    Serial.println(
        "[SEP] J7 is HIGH at boot: stage link NOT connected. "
        "Separation detection DISABLED. (bench ok / pre-flight NOT ok)");
    // 감지기를 켜지 않는다. 핀이 이미 HIGH 라 켜면 즉시 분리로 래치된다.
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  Serial.println("[SEP] stage link connected (J7 LOW); detector armed");

  bool latched = false;
  uint32_t highSinceMs = 0;
  uint32_t lastSendMs = 0;

  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const uint32_t nowMs = millis();

    if (!latched) {
      if (digitalRead(BoardPins::kStageId) == HIGH) {
        if (highSinceMs == 0) highSinceMs = nowMs;
        if (nowMs - highSinceMs >= FlightConfig::kSeparationDebounceMs) {
          latched = true;
          Serial.printf("[SEP] SEPARATED at %lu ms\n",
                        static_cast<unsigned long>(nowMs));
        }
      } else {
        highSinceMs = 0;
      }
    }

    // 한 번 걸린 래치는 풀지 않는다. 끊어진 선이 흔들려 다시 닿아도
    // "분리되지 않음"으로 되돌아가면 안 된다.
    // 큐가 가득 차 메시지가 버려질 수 있으므로 1 초마다 다시 보낸다.
    if (latched && (lastSendMs == 0 || nowMs - lastSendMs >= kRepeatMs)) {
      lastSendMs = nowMs;
      SensorMessage message;
      message.source = SensorSource::StageLink;
      message.timestampMs = nowMs;
      message.valid = true;
      message.separationDetected = true;
      if (xQueueSend(channels_.sensor, &message, 0) != pdTRUE) {
        ++stats_.droppedSensorSamples;
      }
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kPeriodMs));
  }
}
