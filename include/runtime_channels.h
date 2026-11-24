#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "flight_types.h"

// 큐로 넘기기에 적합하지 않은 소수의 교차 태스크 플래그를 모아 둔다. 모두
// 한 태스크만 쓰고 한 태스크만 지우는 단일 워드라 락이 필요 없다.
struct RuntimeStats {
  volatile uint32_t droppedLogFrames = 0;
  volatile uint32_t droppedTelemetryFrames = 0;
  volatile uint32_t droppedSensorSamples = 0;

  // FSM 이 ARM 시점에 세우고 BarometerTask 가 소비한다. 전원을 켠 직후
  // 10 초 평균으로 잡은 영점은 BMP388 이 안정되기 전 값이 섞여 실측에서
  // 약 -9.6 m 의 오차가 남았다. 발사 직전 ARM 때 다시 잡아야 정점 판정의
  // 상대 고도가 맞는다.
  volatile bool rezeroBarometer = false;

  // 재영점 수집이 끝나기 전에 이륙이 잡히면 FSM 이 세운다. BarometerTask 는
  // 모으던 값을 버리고 직전 영점을 그대로 유지한다.
  volatile bool abortRezeroBarometer = false;

  // FSM 이 매 주기 갱신한다. BuzzerTask 가 상태별 소리를 내는 데 쓴다.
  // 지상국이 없으므로 소리가 유일한 상태 표시 수단이다.
  volatile uint8_t currentState = static_cast<uint8_t>(FlightState::Init);

  // StageLinkTask 가 부팅 점검 결과를 남긴다. 텔레메트리에 실려 나가므로
  // 지상국에서 "점퍼 꽂는 것을 잊었는지"를 발사 전에 확인할 수 있다.
  volatile bool stageLinkPresentAtBoot = false;

  // 화약이 터지는 순간 전원 레일이 얼마나 출렁이는지 보려면 10 Hz 로는
  // 부족하다. FSM 이 Apogee 구간에서 세우고, PowerSenseTask 가 100 Hz 로
  // 올린다.
  volatile bool fastPowerSampling = false;
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
