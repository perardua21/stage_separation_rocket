#pragma once

#include "flight_types.h"

// 1단 화약 한 발로 단 분리와 1단 낙하산을 동시에 처리하는 구성의 상태머신.
// 2단 자체 점화는 없다.
//
//   1단: Init -> Safe -> Armed -> Launch -> Coast -> Apogee -> Deploy -> Ground
//   2단: Init -> Safe -> Armed -> Ride   -> Coast -> Apogee -> Deploy -> Ground
//
// 두 단이 붙은 채 정점까지 올라가므로 정점 판정 기준 시각은 양쪽 모두 이륙
// 시각(T0)이다.
class FlightStateMachine {
 public:
  explicit FlightStateMachine(VehicleRole role);

  StateMachineUpdate step(const SensorSample& sample,
                          ControlCommand command = ControlCommand::None);
  FlightSnapshot snapshot() const;

 private:
  StateMachineUpdate makeUpdate(const SensorSample& sample) const;
  void transition(StateMachineUpdate& update, FlightState next,
                  ReasonCode reason, uint32_t nowMs);
  void commandActuator(StateMachineUpdate& update, ActuatorChannel channel,
                       bool enabled, uint32_t nowMs);
  void resetConfirmationCounters();
  void updateApogee(StateMachineUpdate& update, const SensorSample& sample);
  void runDeploySequence(StateMachineUpdate& update,
                         const SensorSample& sample);
  void advanceDeployStep(uint8_t step, uint32_t nowMs);

  ActuatorChannel primaryChannel() const;
  ActuatorChannel backupChannel() const;  // 없으면 None

  VehicleRole role_;
  FlightState state_ = FlightState::Init;
  ReasonCode reason_ = ReasonCode::None;
  SensorSample lastSample_{};
  uint32_t stateEnteredMs_ = 0;
  uint32_t launchMs_ = 0;
  uint32_t separationMs_ = 0;
  uint32_t apogeeMs_ = 0;
  float maxAltitudeM_ = 0.0f;
  uint8_t launchCount_ = 0;
  uint8_t burnoutCount_ = 0;
  uint8_t apogeeCount_ = 0;
  bool separationSeen_ = false;
  bool heaterArmed_ = false;

  // 사출 시퀀스 진행 단계. transition(Apogee) 에서 0 으로 초기화된다.
  uint8_t deployStep_ = 0;
  uint32_t deployStepMs_ = 0;

  uint32_t landingWindowStartMs_ = 0;
  float landingMinAltitudeM_ = 0.0f;
  float landingMaxAltitudeM_ = 0.0f;
};
