#include "missions/telemetry/telemetry_task.h"

#include <Arduino.h>
#include <math.h>

#include "board_pins.h"
#include "flight_config.h"
#include "missions/logging/flight_log_task.h"

namespace {
constexpr uint32_t kPollPeriodMs = 20;

int16_t scaleToInt16(float value, float scale) {
  const float scaled = value * scale;
  if (!isfinite(scaled)) return 0;
  if (scaled > 32767.0f) return 32767;
  if (scaled < -32768.0f) return -32768;
  return static_cast<int16_t>(scaled);
}

uint16_t scaleToUint16(float value, float scale) {
  const float scaled = value * scale;
  if (!isfinite(scaled) || scaled < 0.0f) return 0;
  if (scaled > 65535.0f) return 65535;
  return static_cast<uint16_t>(scaled);
}
}  // namespace

bool TelemetryTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "telemetry", 4096, this, 2, nullptr,
                                 core) == pdPASS;
}

void TelemetryTask::entry(void* context) {
  static_cast<TelemetryTask*>(context)->run();
}

void TelemetryTask::sendTelemetry(const FlightSnapshot& frame) {
  using namespace telemetry;

  TelemetryPayload payload;
  payload.timestampMs = frame.timestampMs;
  payload.state = static_cast<uint8_t>(frame.state);
  payload.reason = static_cast<uint8_t>(frame.reason);
  payload.flags = 0;
  if (frame.separationSeen) payload.flags |= StatusFlag::kSeparated;
  if (frame.heaterArmed) payload.flags |= StatusFlag::kHeaterArmed;
  if (frame.sensor.imuValid) payload.flags |= StatusFlag::kImuValid;
  if (frame.sensor.barometerValid) payload.flags |= StatusFlag::kBarometerValid;
  if (frame.state != FlightState::Init && frame.state != FlightState::Safe) {
    payload.flags |= StatusFlag::kArmed;
  }
  if (stats_.stageLinkPresentAtBoot) {
    payload.flags |= StatusFlag::kStageLinkPresentAtBoot;
  }
  payload.altitudeM = frame.sensor.altitudeM;
  payload.verticalVelocityCmps =
      scaleToInt16(frame.sensor.verticalVelocityMps, 100.0f);
  payload.accelerationMilliG = scaleToInt16(frame.sensor.accelerationG, 100.0f);
  payload.maxAltitudeDm = scaleToInt16(frame.maxAltitudeM, 10.0f);
  payload.batteryMilliV = scaleToUint16(frame.sensor.batteryV, 1000.0f);
  payload.tiltDeciDeg = scaleToInt16(frame.sensor.tiltDeg, 10.0f);
  payload.droppedFrames = scaleToUint16(
      static_cast<float>(frame.droppedLogFrames + frame.droppedTelemetryFrames),
      1.0f);

  const NodeId node = role_ == VehicleRole::Booster ? NodeId::Stage1
                                                    : NodeId::Stage2;
  uint8_t buffer[kMaxFrameBytes];
  const size_t length = encodeTelemetry(node, payload, buffer, sizeof(buffer));
  if (length == 0) return;

  // AUX LOW 는 모듈이 아직 바쁘다는 뜻이다. 그 때는 이번 슬롯을 건너뛴다.
  if (digitalRead(BoardPins::kE22Aux) != HIGH) return;
  radio_.write(buffer, length);
}

void TelemetryTask::handleCommand(const telemetry::CommandPayload& payload) {
  using namespace telemetry;

  const uint8_t self = static_cast<uint8_t>(
      role_ == VehicleRole::Booster ? NodeId::Stage1 : NodeId::Stage2);
  if (payload.target != kTargetAll && payload.target != self) return;

  // 같은 명령이 여러 번 도착해도 한 번만 처리한다.
  if (haveNonce_ && payload.nonce == lastNonce_) return;
  haveNonce_ = true;
  lastNonce_ = payload.nonce;

  const auto state = static_cast<FlightState>(stats_.currentState);
  const bool onGround =
      state == FlightState::Init || state == FlightState::Safe ||
      state == FlightState::Armed || state == FlightState::Ground;

  ControlCommand command = ControlCommand::None;
  switch (static_cast<RemoteCommand>(payload.command)) {
    case RemoteCommand::Arm:
      if (!FlightConfig::kAcceptRemoteArm || !onGround) {
        Serial.println("[RADIO] ARM rejected (in flight or disabled)");
        return;
      }
      command = ControlCommand::Arm;
      break;
    case RemoteCommand::Disarm:
      if (!onGround) {
        Serial.println("[RADIO] DISARM rejected (in flight)");
        return;
      }
      command = ControlCommand::Disarm;
      break;
    case RemoteCommand::Ping:
      Serial.println("[RADIO] ping");
      return;
    default:
      return;
  }

  xQueueSend(channels_.command, &command, 0);
  Serial.printf("[RADIO] command accepted: %u\n",
                static_cast<unsigned>(payload.command));
}

void TelemetryTask::pollRadio() {
  telemetry::ParsedFrame frame;
  // 한 번에 다 비우지 않고 상한을 둔다. 잡음으로 UART 가 계속 차 있을 때
  // 이 태스크가 CPU 를 독점하지 않게 한다.
  for (int i = 0; i < 128 && radio_.available(); ++i) {
    if (!reader_.push(static_cast<uint8_t>(radio_.read()), frame)) continue;
    if (frame.source != telemetry::NodeId::Ground) continue;
    telemetry::CommandPayload payload;
    if (telemetry::decodeCommand(frame, payload)) handleCommand(payload);
  }
}

void TelemetryTask::run() {
  pinMode(BoardPins::kE22M0, OUTPUT);
  pinMode(BoardPins::kE22M1, OUTPUT);
  pinMode(BoardPins::kE22Aux, INPUT);
  // M0=0, M1=0 : 투명 전송 모드
  digitalWrite(BoardPins::kE22M0, LOW);
  digitalWrite(BoardPins::kE22M1, LOW);
  radio_.begin(BoardPins::kE22Baud, SERIAL_8N1, BoardPins::kE22Rx,
               BoardPins::kE22Tx);

  // 1단과 2단이 같은 순간에 송신하지 않도록 슬롯을 밀어 둔다.
  const uint32_t slotOffsetMs = role_ == VehicleRole::Booster
                                    ? FlightConfig::kStage1SlotOffsetMs
                                    : FlightConfig::kStage2SlotOffsetMs;
  vTaskDelay(pdMS_TO_TICKS(slotOffsetMs));

  FlightSnapshot frame;
  uint32_t lastSendMs = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    pollRadio();

    const uint32_t nowMs = millis();
    if (nowMs - lastSendMs >= FlightConfig::kTelemetryPeriodMs) {
      lastSendMs = nowMs;
      // 깊이 1 의 덮어쓰기 큐는 최신값 우편함이다. 텔레메트리가 느려도
      // FSM 을 막지 않고, FSM 상태를 동시에 읽지도 않는다.
      if (xQueuePeek(channels_.telemetry, &frame, 0) == pdTRUE) {
        sendTelemetry(frame);
        writeCsvRow(Serial, frame);  // USB 콘솔용 사람이 읽는 형식
      }
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kPollPeriodMs));
  }
}
