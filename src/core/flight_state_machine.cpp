#include "core/flight_state_machine.h"

#include <algorithm>

#include "flight_config.h"

namespace {
bool elapsed(uint32_t now, uint32_t start, uint32_t duration) {
  return static_cast<uint32_t>(now - start) >= duration;
}
}  // namespace

FlightStateMachine::FlightStateMachine(VehicleRole role) : role_(role) {}

StateMachineUpdate FlightStateMachine::makeUpdate(
    const SensorSample& sample) const {
  StateMachineUpdate update;
  update.snapshot.timestampMs = sample.timestampMs;
  update.snapshot.role = role_;
  update.snapshot.state = state_;
  update.snapshot.reason = reason_;
  update.snapshot.sensor = sample;
  update.snapshot.maxAltitudeM = maxAltitudeM_;
  update.snapshot.ignitionPermanentlyInhibited =
      ignitionPermanentlyInhibited_;
  return update;
}

void FlightStateMachine::transition(StateMachineUpdate& update,
                                    FlightState next, ReasonCode reason,
                                    uint32_t nowMs) {
  if (state_ == next) return;
  state_ = next;
  reason_ = reason;
  stateEnteredMs_ = nowMs;
  resetConfirmationCounters();
  update.stateChanged = true;
  update.snapshot.state = state_;
  update.snapshot.reason = reason_;

  if (next == FlightState::Launch || next == FlightState::Ride) launchMs_ = nowMs;
  if (next == FlightState::Separated ||
      (role_ == VehicleRole::Booster && next == FlightState::Coast)) {
    separationMs_ = nowMs;
  }
  if (next == FlightState::Boost) ignitionMs_ = nowMs;
  if (next == FlightState::Recovery) ignitionPermanentlyInhibited_ = true;
  if (next == FlightState::Apogee) {
    apogeeMs_ = nowMs;
    pulseStarted_ = false;
    secondPulseStarted_ = false;
    firstPulseStopped_ = false;
    secondPulseStopped_ = false;
  }
  if (next == FlightState::Deploy) {
    landingWindowStartMs_ = nowMs;
    landingMinAltitudeM_ = lastSample_.altitudeM;
    landingMaxAltitudeM_ = lastSample_.altitudeM;
  }
}

void FlightStateMachine::commandActuator(StateMachineUpdate& update,
                                         ActuatorChannel channel, bool enabled,
                                         uint32_t nowMs) {
  update.actuatorCommandValid = true;
  update.actuator.timestampMs = nowMs;
  update.actuator.channel = channel;
  update.actuator.enabled = enabled;
}

void FlightStateMachine::resetConfirmationCounters() {
  launchCount_ = 0;
  burnoutCount_ = 0;
  ignitionCount_ = 0;
  apogeeCount_ = 0;
}

void FlightStateMachine::updateApogee(StateMachineUpdate& update,
                                      const SensorSample& sample,
                                      uint32_t earliestMs,
                                      uint32_t timeoutMs,
                                      uint32_t referenceMs) {
  if (sample.barometerValid &&
      !elapsed(sample.timestampMs, separationMs_,
               FlightConfig::kBarometerLockoutMs)) {
    return;
  }
  if (sample.barometerValid) {
    maxAltitudeM_ = std::max(maxAltitudeM_, sample.altitudeM);
  }
  if (elapsed(sample.timestampMs, referenceMs, timeoutMs)) {
    transition(update, FlightState::Apogee, ReasonCode::ApogeeTimeout,
               sample.timestampMs);
    return;
  }
  if (!sample.barometerValid) {
    return;
  }
  if (!elapsed(sample.timestampMs, referenceMs, earliestMs)) {
    apogeeCount_ = 0;
    return;
  }
  const bool descending = sample.altitudeM <= maxAltitudeM_ - FlightConfig::kApogeeDropM &&
                          sample.verticalVelocityMps <= 0.0f;
  apogeeCount_ = descending ? static_cast<uint8_t>(apogeeCount_ + 1) : 0;
  if (apogeeCount_ >= FlightConfig::kApogeeSamples) {
    transition(update, FlightState::Apogee, ReasonCode::ApogeeDetected,
               sample.timestampMs);
  }
}

void FlightStateMachine::updateDeployment(StateMachineUpdate& update,
                                          const SensorSample& sample) {
  const uint32_t sinceApogee = sample.timestampMs - apogeeMs_;
  if (role_ == VehicleRole::Booster) {
    if (!pulseStarted_) {
      pulseStarted_ = true;
      commandActuator(update, ActuatorChannel::Heater1, true,
                      sample.timestampMs);
      return;
    }
    if (!firstPulseStopped_ && sinceApogee >= FlightConfig::kPyroPulseMs) {
      firstPulseStopped_ = true;
      commandActuator(update, ActuatorChannel::Heater1, false,
                      sample.timestampMs);
      return;
    }
    if (!secondPulseStarted_ && sinceApogee >= FlightConfig::kPyroPulseMs) {
      secondPulseStarted_ = true;
      secondPulseStartMs_ = sample.timestampMs;
      commandActuator(update, ActuatorChannel::Heater2, true,
                      sample.timestampMs);
      return;
    }
    if (!secondPulseStopped_ &&
        elapsed(sample.timestampMs, secondPulseStartMs_,
                FlightConfig::kPyroPulseMs)) {
      secondPulseStopped_ = true;
      commandActuator(update, ActuatorChannel::Heater2, false,
                      sample.timestampMs);
      return;
    }
    if (secondPulseStopped_) {
      transition(update, FlightState::Deploy, reason_, sample.timestampMs);
    }
  } else {
    // The supplied PCB has only two heater channels. Heater1 is permanently
    // reserved for stage-2 ignition; it is never reused for backup deployment.
    if (!pulseStarted_) {
      pulseStarted_ = true;
      commandActuator(update, ActuatorChannel::Heater2, true,
                      sample.timestampMs);
      return;
    }
    if (!firstPulseStopped_ && sinceApogee >= FlightConfig::kPyroPulseMs) {
      firstPulseStopped_ = true;
      commandActuator(update, ActuatorChannel::Heater2, false,
                      sample.timestampMs);
      return;
    }
    if (firstPulseStopped_) {
      transition(update, FlightState::Deploy, reason_, sample.timestampMs);
    }
  }
}

StateMachineUpdate FlightStateMachine::step(const SensorSample& sample,
                                            ControlCommand command) {
  lastSample_ = sample;
  StateMachineUpdate update = makeUpdate(sample);

  if (command == ControlCommand::Disarm && state_ == FlightState::Armed) {
    transition(update, FlightState::Safe, ReasonCode::None,
               sample.timestampMs);
    return update;
  }
  if (command == ControlCommand::BenchSeparationDetected) {
    lastSample_.separationDetected = true;
  }

  switch (state_) {
    case FlightState::Init:
      if (sample.imuValid && sample.barometerValid) {
        transition(update, FlightState::Safe, ReasonCode::None,
                   sample.timestampMs);
      }
      break;
    case FlightState::Safe:
      if (command == ControlCommand::BenchArm) {
        transition(update, FlightState::Armed, ReasonCode::None,
                   sample.timestampMs);
      }
      break;
    case FlightState::Armed:
      if (sample.imuValid) {
        launchCount_ = sample.accelerationG >= FlightConfig::kLaunchThresholdG
                           ? static_cast<uint8_t>(launchCount_ + 1)
                           : 0;
        if (launchCount_ >= FlightConfig::kLaunchSamples) {
          transition(update,
                     role_ == VehicleRole::Booster ? FlightState::Launch
                                                   : FlightState::Ride,
                     ReasonCode::LaunchDetected, sample.timestampMs);
          maxAltitudeM_ = sample.altitudeM;
        }
      }
      break;
    case FlightState::Launch:
      if (sample.imuValid &&
          elapsed(sample.timestampMs, launchMs_,
                  FlightConfig::kBoosterBurnoutEarliestMs)) {
        burnoutCount_ = sample.accelerationG <= FlightConfig::kBurnoutThresholdG
                            ? static_cast<uint8_t>(burnoutCount_ + 1)
                            : 0;
      }
      if (burnoutCount_ >= FlightConfig::kBurnoutSamples) {
        transition(update, FlightState::Separating,
                   ReasonCode::BurnoutDetected, sample.timestampMs);
      } else if (elapsed(sample.timestampMs, launchMs_,
                         FlightConfig::kBoosterBurnoutTimeoutMs)) {
        transition(update, FlightState::Separating,
                   ReasonCode::BurnoutTimeout, sample.timestampMs);
      }
      break;
    case FlightState::Separating:
      if (sample.separationDetected ||
          command == ControlCommand::BenchSeparationDetected) {
        transition(update, FlightState::Coast,
                   ReasonCode::SeparationDetected, sample.timestampMs);
      } else if (elapsed(sample.timestampMs, launchMs_,
                         FlightConfig::kSeparationTimeoutFromLaunchMs)) {
        transition(update, FlightState::Recovery, ReasonCode::NoSeparation,
                   sample.timestampMs);
      }
      break;
    case FlightState::Ride:
      preSeparationAltitudeM_ = sample.altitudeM;
      if (sample.separationDetected ||
          command == ControlCommand::BenchSeparationDetected) {
        transition(update, FlightState::Separated,
                   ReasonCode::SeparationDetected, sample.timestampMs);
      } else if (elapsed(sample.timestampMs, launchMs_,
                         FlightConfig::kSeparationTimeoutFromLaunchMs)) {
        transition(update, FlightState::Recovery, ReasonCode::NoSeparation,
                   sample.timestampMs);
      }
      break;
    case FlightState::Separated: {
      const uint32_t sinceSep = sample.timestampMs - separationMs_;
      if (sample.tiltDeg >= FlightConfig::kIgnitionMaxTiltDeg) {
        transition(update, FlightState::Recovery, ReasonCode::Tilt,
                   sample.timestampMs);
        break;
      }
      const bool inWindow = sinceSep >= FlightConfig::kIgnitionWindowOpenMs &&
                            sinceSep <= FlightConfig::kIgnitionWindowCloseMs;
      if (inWindow &&
          sample.integratedVelocityMps > FlightConfig::kIgnitionMinVelocityMps &&
          preSeparationAltitudeM_ >= FlightConfig::kIgnitionMinAltitudeM) {
        ignitionPermanentlyInhibited_ = false;
        ignitionOutputActive_ = true;
        ignitionCommandMs_ = sample.timestampMs;
        transition(update, FlightState::Igniting,
                   ReasonCode::IgnitionCommanded, sample.timestampMs);
        commandActuator(update, ActuatorChannel::Heater1, true,
                        sample.timestampMs);
      } else if (sinceSep > FlightConfig::kIgnitionWindowCloseMs) {
        const ReasonCode reason =
            preSeparationAltitudeM_ < FlightConfig::kIgnitionMinAltitudeM
                ? ReasonCode::LowAltitude
                : ReasonCode::LowVelocity;
        transition(update, FlightState::Recovery, reason, sample.timestampMs);
      }
      break;
    }
    case FlightState::Igniting:
      if (sample.imuValid) {
        ignitionCount_ = sample.accelerationG >= FlightConfig::kIgnitionConfirmG
                             ? static_cast<uint8_t>(ignitionCount_ + 1)
                             : 0;
      }
      if (ignitionCount_ >= FlightConfig::kIgnitionSamples) {
        ignitionPermanentlyInhibited_ = true;
        transition(update, FlightState::Boost, ReasonCode::IgnitionDetected,
                   sample.timestampMs);
      } else if (elapsed(sample.timestampMs, stateEnteredMs_,
                         FlightConfig::kIgnitionConfirmTimeoutMs)) {
        commandActuator(update, ActuatorChannel::Heater1, false,
                        sample.timestampMs);
        ignitionOutputActive_ = false;
        ignitionPermanentlyInhibited_ = true;
        transition(update, FlightState::Recovery, ReasonCode::NoIgnition,
                   sample.timestampMs);
      }
      break;
    case FlightState::Boost:
      if (ignitionOutputActive_ &&
          elapsed(sample.timestampMs, ignitionCommandMs_,
                  FlightConfig::kPyroPulseMs)) {
        commandActuator(update, ActuatorChannel::Heater1, false,
                        sample.timestampMs);
        ignitionOutputActive_ = false;
      }
      if (sample.imuValid &&
          elapsed(sample.timestampMs, ignitionMs_,
                  FlightConfig::kSustainerBurnoutEarliestMs)) {
        burnoutCount_ = sample.accelerationG <= FlightConfig::kBurnoutThresholdG
                            ? static_cast<uint8_t>(burnoutCount_ + 1)
                            : 0;
      }
      if (burnoutCount_ >= FlightConfig::kBurnoutSamples) {
        transition(update, FlightState::Coast, ReasonCode::BoostBurnout,
                   sample.timestampMs);
      } else if (elapsed(sample.timestampMs, ignitionMs_,
                         FlightConfig::kSustainerBurnoutTimeoutMs)) {
        transition(update, FlightState::Coast, ReasonCode::BoostTimeout,
                   sample.timestampMs);
      }
      break;
    case FlightState::Coast:
      if (role_ == VehicleRole::Booster) {
        updateApogee(update, sample,
                     FlightConfig::kBoosterApogeeEarliestFromLaunchMs,
                     FlightConfig::kBoosterApogeeTimeoutFromLaunchMs,
                     launchMs_);
      } else {
        updateApogee(update, sample,
                     FlightConfig::kSustainerApogeeEarliestFromIgnitionMs,
                     FlightConfig::kSustainerApogeeTimeoutFromIgnitionMs,
                     ignitionMs_);
      }
      break;
    case FlightState::Recovery:
      // Recovery forced-transition times are deliberately not invented. Until
      // the missing simulations are supplied, only the documented barometric
      // apogee rule is accepted here.
      if (sample.barometerValid) {
        maxAltitudeM_ = std::max(maxAltitudeM_, sample.altitudeM);
        const bool descending =
            sample.altitudeM <= maxAltitudeM_ - FlightConfig::kApogeeDropM &&
            sample.verticalVelocityMps <= 0.0f;
        apogeeCount_ = descending ? static_cast<uint8_t>(apogeeCount_ + 1) : 0;
        if (apogeeCount_ >= FlightConfig::kApogeeSamples) {
          transition(update, FlightState::Apogee,
                     ReasonCode::ApogeeDetected, sample.timestampMs);
        }
      }
      break;
    case FlightState::Apogee:
      updateDeployment(update, sample);
      break;
    case FlightState::Deploy: {
      if (sample.barometerValid) {
        landingMinAltitudeM_ = std::min(landingMinAltitudeM_, sample.altitudeM);
        landingMaxAltitudeM_ = std::max(landingMaxAltitudeM_, sample.altitudeM);
      }
      if (elapsed(sample.timestampMs, landingWindowStartMs_,
                  FlightConfig::kLandingWindowMs)) {
        if (landingMaxAltitudeM_ - landingMinAltitudeM_ <
            FlightConfig::kLandingRangeM) {
          transition(update, FlightState::Ground, ReasonCode::Landed,
                     sample.timestampMs);
          break;
        }
        landingWindowStartMs_ = sample.timestampMs;
        landingMinAltitudeM_ = sample.altitudeM;
        landingMaxAltitudeM_ = sample.altitudeM;
      }
      const uint32_t timeout = role_ == VehicleRole::Booster
                                   ? FlightConfig::kBoosterLandingTimeoutMs
                                   : FlightConfig::kSustainerLandingTimeoutMs;
      if (elapsed(sample.timestampMs, apogeeMs_, timeout)) {
        transition(update, FlightState::Ground, ReasonCode::LandingTimeout,
                   sample.timestampMs);
      }
      break;
    }
    case FlightState::Ground:
    case FlightState::Fault:
      break;
  }

  update.snapshot = snapshot();
  update.snapshot.timestampMs = sample.timestampMs;
  update.snapshot.sensor = sample;
  return update;
}

FlightSnapshot FlightStateMachine::snapshot() const {
  FlightSnapshot result;
  result.timestampMs = lastSample_.timestampMs;
  result.role = role_;
  result.state = state_;
  result.reason = reason_;
  result.sensor = lastSample_;
  result.maxAltitudeM = maxAltitudeM_;
  result.ignitionPermanentlyInhibited = ignitionPermanentlyInhibited_;
  return result;
}

const char* flightStateName(FlightState state) {
  switch (state) {
    case FlightState::Init: return "INIT";
    case FlightState::Safe: return "SAFE";
    case FlightState::Armed: return "ARMED";
    case FlightState::Launch: return "LAUNCH";
    case FlightState::Separating: return "SEPARATING";
    case FlightState::Ride: return "RIDE";
    case FlightState::Separated: return "SEPARATED";
    case FlightState::Igniting: return "IGNITING";
    case FlightState::Boost: return "BOOST";
    case FlightState::Coast: return "COAST";
    case FlightState::Recovery: return "RECOVERY";
    case FlightState::Apogee: return "APOGEE";
    case FlightState::Deploy: return "DEPLOY";
    case FlightState::Ground: return "GROUND";
    case FlightState::Fault: return "FAULT";
  }
  return "UNKNOWN";
}

const char* reasonCodeName(ReasonCode reason) {
  switch (reason) {
    case ReasonCode::None: return "NONE";
    case ReasonCode::InitFailed: return "INIT_FAILED";
    case ReasonCode::LaunchDetected: return "LAUNCH_DETECTED";
    case ReasonCode::BurnoutDetected: return "BURNOUT_DETECTED";
    case ReasonCode::BurnoutTimeout: return "BURNOUT_TIMEOUT";
    case ReasonCode::SeparationDetected: return "SEPARATION_DETECTED";
    case ReasonCode::NoSeparation: return "NO_SEP";
    case ReasonCode::Tilt: return "TILT";
    case ReasonCode::LowVelocity: return "LOW_VEL";
    case ReasonCode::LowAltitude: return "LOW_ALT";
    case ReasonCode::IgnitionCommanded: return "IGNITION_COMMANDED";
    case ReasonCode::IgnitionDetected: return "IGNITION_DETECTED";
    case ReasonCode::NoIgnition: return "NO_IGN";
    case ReasonCode::BoostBurnout: return "BOOST_BURNOUT";
    case ReasonCode::BoostTimeout: return "BOOST_TIMEOUT";
    case ReasonCode::ApogeeDetected: return "APOGEE_DETECTED";
    case ReasonCode::ApogeeTimeout: return "APOGEE_TIMEOUT";
    case ReasonCode::Landed: return "LANDED";
    case ReasonCode::LandingTimeout: return "LANDING_TIMEOUT";
  }
  return "UNKNOWN";
}
