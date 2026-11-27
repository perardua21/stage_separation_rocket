#include "missions/flight/fsm_task.h"

#include <Arduino.h>

#include "board_pins.h"

namespace {
// IMU 메시지가 이 시간 이상 끊기면 기압계가 상태머신을 구동한다.
// ImuTask 는 imu_.read() 가 실패하면 메시지를 아예 보내지 않는다. 즉 BNO085
// 가 비행 중 멈추면 상태머신이 같이 멈춰 정점 판정도, 사출도 일어나지 않는다.
// 기압계만 살아 있어도 정점 판정과 타임아웃은 돌아가야 한다.
constexpr uint32_t kImuStaleMs = 200;
#if defined(NURA_BENCH_SAFE)
constexpr bool isForwardImuStale(uint32_t barometerMs, uint32_t imuMs) {
  const uint32_t ageMs = barometerMs - imuMs;
  // A small backward step appears in the upper half of the millis() range.
  return ageMs > kImuStaleMs && ageMs < 0x80000000u;
}

static_assert(!isForwardImuStale(1884u, 1885u));
static_assert(!isForwardImuStale(1200u, 1000u));
static_assert(isForwardImuStale(1201u, 1000u));
static_assert(isForwardImuStale(200u, 0xffffffffu));  // 201 ms across rollover
#endif
}  // namespace

bool FsmTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "fsm", 6144, this, 5, nullptr, core) ==
         pdPASS;
}

void FsmTask::entry(void* context) { static_cast<FsmTask*>(context)->run(); }

void FsmTask::run() {
  SensorMessage message;
  SensorSample sample;
#if defined(NURA_BENCH_SAFE)
  sample.tiltDeg = -1.0f;  // No usable diagnostic tilt yet.
#endif
  uint32_t lastBarometerMs = 0;
  uint32_t lastImuMs = 0;
  bool imuLossReported = false;

  for (;;) {
    if (xQueueReceive(channels_.sensor, &message, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    if (message.source == SensorSource::Power) {
      if (message.valid) sample.batteryV = message.batteryV;
      continue;
    }
    if (message.source == SensorSource::StageLink) {
      // 단방향 래치. 한 번 true 가 되면 내려가지 않는다.
      if (message.separationDetected) sample.separationDetected = true;
      continue;
    }

    if (message.source == SensorSource::Barometer) {
      sample.altitudeM = message.altitudeM;
      sample.verticalVelocityMps = message.verticalVelocityMps;
      sample.barometerValid = message.valid;
      lastBarometerMs = message.timestampMs;

      // IMU 가 한 번도 안 왔으면 아직 부팅 중이다. 그대로 기다린다.
      if (lastImuMs == 0) continue;
      // IMU 가 살아 있으면 상태머신은 IMU 주기로만 돈다.
#if defined(NURA_BENCH_SAFE)
      if (!isForwardImuStale(message.timestampMs, lastImuMs)) continue;
#else
      if (message.timestampMs - lastImuMs <= kImuStaleMs) continue;
#endif

      // 여기부터는 IMU 가 끊긴 경우다. accelerationG 등 IMU 값은 마지막으로
      // 받은 값을 그대로 두고, imuValid 만 내려서 가속도 기반 판정
      // (이륙·연소종료)을 막는다. 고도 기반 정점 판정과 타임아웃은 계속
      // 동작한다.
      if (!imuLossReported) {
        imuLossReported = true;
#if defined(NURA_BENCH_SAFE)
        Serial.printf("[FSM] IMU stale: baro=%lu ms, last_imu=%lu ms, diff=%lu ms\n",
                      static_cast<unsigned long>(message.timestampMs),
                      static_cast<unsigned long>(lastImuMs),
                      static_cast<unsigned long>(message.timestampMs - lastImuMs));
#else
        Serial.printf("[FSM] IMU stale at %lu ms; barometer now driving\n",
                      static_cast<unsigned long>(message.timestampMs));
#endif
      }
      sample.timestampMs = message.timestampMs;
      sample.imuValid = false;
#if defined(NURA_BENCH_SAFE)
      sample.tiltDeg = -1.0f;  // Do not log the cached angle as a fresh sample.
#endif
    } else {
      lastImuMs = message.timestampMs;
      imuLossReported = false;
      sample.timestampMs = message.timestampMs;
      sample.accelerationG = message.accelerationG;
      sample.tiltDeg = message.tiltDeg;
      sample.integratedVelocityMps = message.integratedVelocityMps;
      sample.imuValid = message.valid;
      sample.barometerValid = sample.barometerValid &&
                              (message.timestampMs - lastBarometerMs <= 250U);
    }

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
    stats_.currentState = static_cast<uint8_t>(update.snapshot.state);
    // 화약이 터지는 구간에서는 배터리 레일을 100 Hz 로 받는다. 전압이
    // 얼마나 내려앉는지가 재설계 판단 근거가 된다.
    stats_.fastPowerSampling = update.snapshot.state == FlightState::Apogee;

    if (update.stateChanged) {
      // ARM 에 들어간 순간 기압 영점을 다시 잡는다. 전원 투입 직후 10 초
      // 평균은 BMP388 이 안정되기 전 값이 섞여 실측 -9.6 m 오차가 남았다.
      if (update.snapshot.state == FlightState::Armed) {
        stats_.rezeroBarometer = true;
      }
      // 재영점 수집 중에 발사되면 상승 중 기압이 영점에 섞인다. 이륙이
      // 잡히는 순간 수집을 취소시켜 직전 영점을 그대로 쓰게 한다.
      if (update.snapshot.state == FlightState::Launch ||
          update.snapshot.state == FlightState::Ride) {
        stats_.abortRezeroBarometer = true;
      }
      Serial.printf("[FSM] %s -> %s (%s) at %lu ms\n",
                    update.snapshot.role == VehicleRole::Booster ? "B" : "S",
                    flightStateName(update.snapshot.state),
                    reasonCodeName(update.snapshot.reason),
                    static_cast<unsigned long>(update.snapshot.timestampMs));
    }

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
                 update.snapshot.state == FlightState::Armed ||
                         update.snapshot.state == FlightState::Apogee ||
                         update.snapshot.state == FlightState::Fault
                     ? HIGH
                     : LOW);
  }
}
