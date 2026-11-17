#pragma once

#include "flight_types.h"

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
  void updateApogee(StateMachineUpdate& update, const SensorSample& sample,
                    uint32_t earliestMs, uint32_t timeoutMs,
                    uint32_t referenceMs);
  void updateDeployment(StateMachineUpdate& update,
                        const SensorSample& sample);

  VehicleRole role_;
  FlightState state_ = FlightState::Init;
  ReasonCode reason_ = ReasonCode::None;
  SensorSample lastSample_{};
  uint32_t stateEnteredMs_ = 0;
  uint32_t launchMs_ = 0;
  uint32_t separationMs_ = 0;
  uint32_t ignitionMs_ = 0;
  uint32_t apogeeMs_ = 0;
  float maxAltitudeM_ = 0.0f;
  float preSeparationAltitudeM_ = 0.0f;
  uint8_t launchCount_ = 0;
  uint8_t burnoutCount_ = 0;
  uint8_t ignitionCount_ = 0;
  uint8_t apogeeCount_ = 0;
  bool ignitionPermanentlyInhibited_ = true;
  bool ignitionOutputActive_ = false;
  uint32_t ignitionCommandMs_ = 0;
  bool pulseStarted_ = false;
  bool secondPulseStarted_ = false;
  uint32_t secondPulseStartMs_ = 0;
  bool firstPulseStopped_ = false;
  bool secondPulseStopped_ = false;
  uint32_t landingWindowStartMs_ = 0;
  float landingMinAltitudeM_ = 0.0f;
  float landingMaxAltitudeM_ = 0.0f;
};
