#include "core/flight_state_machine.h"

#include <algorithm>

#include "flight_config.h"

namespace {
bool elapsed(uint32_t now, uint32_t start, uint32_t duration) {
  return static_cast<uint32_t>(now - start) >= duration;
}
}  // namespace

FlightStateMachine::FlightStateMachine(VehicleRole role) : role_(role) {}

ActuatorChannel FlightStateMachine::primaryChannel() const {
  // 1단은 CH1 한 발이 분리와 낙하산을 같이 한다. 2단은 CH2 가 낙하산이다.
  return role_ == VehicleRole::Booster ? ActuatorChannel::Heater1
                                       : ActuatorChannel::Heater2;
}

ActuatorChannel FlightStateMachine::backupChannel() const {
  // 백업 화약을 실제로 장착했을 때만 1단 CH2 를 쓴다.
  if (role_ == VehicleRole::Booster &&
      FlightConfig::kBoosterBackupChargeFitted) {
    return ActuatorChannel::Heater2;
  }
  return ActuatorChannel::None;
}

StateMachineUpdate FlightStateMachine::makeUpdate(
    const SensorSample& sample) const {
  StateMachineUpdate update;
  update.snapshot.timestampMs = sample.timestampMs;
  update.snapshot.role = role_;
  update.snapshot.state = state_;
  update.snapshot.reason = reason_;
  update.snapshot.sensor = sample;
  update.snapshot.maxAltitudeM = maxAltitudeM_;
  update.snapshot.separationSeen = separationSeen_;
  update.snapshot.heaterArmed = heaterArmed_;
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

  if (next == FlightState::Launch || next == FlightState::Ride) {
    launchMs_ = nowMs;
  }
  if (next == FlightState::Apogee) {
    apogeeMs_ = nowMs;
    deployStep_ = 0;
    deployStepMs_ = nowMs;
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
  apogeeCount_ = 0;
}

void FlightStateMachine::updateApogee(StateMachineUpdate& update,
                                      const SensorSample& sample) {
  if (sample.barometerValid) {
    maxAltitudeM_ = std::max(maxAltitudeM_, sample.altitudeM);
  }

  // 백업 경로. 기압계가 죽어도 반드시 사출한다.
  if (elapsed(sample.timestampMs, launchMs_,
              FlightConfig::kApogeeTimeoutFromLaunchMs)) {
    transition(update, FlightState::Apogee, ReasonCode::ApogeeTimeout,
               sample.timestampMs);
    return;
  }

  if (!sample.barometerValid) {
    // 2단 한정 보조 경로. 기압을 못 믿는 상황에서 단간 선이 끊겼다면 1단이
    // 정점으로 판단해 화약을 쏜 것이므로, 그 시점을 정점으로 받아들인다.
    if (role_ == VehicleRole::Sustainer && separationSeen_) {
      transition(update, FlightState::Apogee, ReasonCode::SeparationDetected,
                 sample.timestampMs);
    }
    return;
  }

  // 최소 비행시간 가드. 상승 초반의 압력 요동으로 조기 사출되는 것을 막는다.
  if (!elapsed(sample.timestampMs, launchMs_,
               FlightConfig::kApogeeEarliestFromLaunchMs)) {
    apogeeCount_ = 0;
    return;
  }

  // 최소 고도 가드. 기체가 제대로 올라간 적이 없으면 기압 기반 정점 판정을
  // 쓰지 않는다. 발사대 근처에서 화약이 터지는 것을 막는다.
  if (maxAltitudeM_ < FlightConfig::kMinApogeeAltitudeM) {
    apogeeCount_ = 0;
    return;
  }

  const bool descending =
      sample.altitudeM <= maxAltitudeM_ - FlightConfig::kApogeeDropM &&
      sample.verticalVelocityMps <= 0.0f;
  apogeeCount_ = descending ? static_cast<uint8_t>(apogeeCount_ + 1) : 0;
  if (apogeeCount_ >= FlightConfig::kApogeeSamples) {
    transition(update, FlightState::Apogee, ReasonCode::ApogeeDetected,
               sample.timestampMs);
  }
}

void FlightStateMachine::advanceDeployStep(uint8_t step, uint32_t nowMs) {
  deployStep_ = step;
  deployStepMs_ = nowMs;
}

// 사출 시퀀스
//   0  2단만: 분리가 끝날 시간을 준다
//   1  HEAT_ARM 을 올리고 전원이 안정될 때까지 대기
//   2  주 채널 ON, 펄스 유지
//   3  주 채널 OFF, 백업이 있으면 간격 대기
//   4  백업 채널 ON, 펄스 유지
//   5  백업 채널 OFF
//   6  HEAT_ARM 내리고 Deploy 로
//
// 한 번의 step() 은 액추에이터 명령을 하나만 실어 보낼 수 있으므로 각
// 단계는 명령을 내고 즉시 반환한다. FSM 주기가 10 ms 라 시퀀스 자체가
// 늦어지는 양은 무시할 수준이다.
void FlightStateMachine::runDeploySequence(StateMachineUpdate& update,
                                           const SensorSample& sample) {
  const uint32_t now = sample.timestampMs;
  const uint32_t sinceStep = now - deployStepMs_;

  switch (deployStep_) {
    case 0:
      if (role_ == VehicleRole::Sustainer &&
          sinceStep < FlightConfig::kSustainerDeployDelayMs) {
        return;
      }
      advanceDeployStep(1, now);
      heaterArmed_ = true;
      commandActuator(update, ActuatorChannel::CommonArm, true, now);
      return;

    case 1:
      if (sinceStep < FlightConfig::kHeaterArmSettleMs) return;
      advanceDeployStep(2, now);
      commandActuator(update, primaryChannel(), true, now);
      return;

    case 2:
      if (sinceStep < FlightConfig::kPyroPulseMs) return;
      advanceDeployStep(3, now);
      commandActuator(update, primaryChannel(), false, now);
      return;

    case 3:
      if (backupChannel() == ActuatorChannel::None) {
        advanceDeployStep(6, now);
        return;
      }
      if (sinceStep < FlightConfig::kHeaterGapMs) return;
      advanceDeployStep(4, now);
      commandActuator(update, backupChannel(), true, now);
      return;

    case 4:
      if (sinceStep < FlightConfig::kPyroPulseMs) return;
      advanceDeployStep(5, now);
      commandActuator(update, backupChannel(), false, now);
      return;

    case 5:
      advanceDeployStep(6, now);
      return;

    default:
      heaterArmed_ = false;
      commandActuator(update, ActuatorChannel::CommonArm, false, now);
      transition(update, FlightState::Deploy, reason_, now);
      return;
  }
}

StateMachineUpdate FlightStateMachine::step(const SensorSample& sample,
                                            ControlCommand command) {
  lastSample_ = sample;
  StateMachineUpdate update = makeUpdate(sample);

  // 분리는 한 번 보이면 래치한다. 끊어진 선이 우연히 다시 닿아도 되돌리지
  // 않는다. 지금 구성에서 분리 신호는 기록용이고, 기압계 고장 시 2단의
  // 보조 정점 판단에만 쓰인다.
  if (sample.separationDetected ||
      command == ControlCommand::BenchSeparationDetected) {
    if (!separationSeen_) separationMs_ = sample.timestampMs;
    separationSeen_ = true;
    lastSample_.separationDetected = true;
  }

  if (command == ControlCommand::Disarm && state_ == FlightState::Armed) {
    transition(update, FlightState::Safe, ReasonCode::None,
               sample.timestampMs);
    update.snapshot = snapshot();
    update.snapshot.timestampMs = sample.timestampMs;
    return update;
  }

  switch (state_) {
    case FlightState::Init:
      if (sample.imuValid && sample.barometerValid) {
        transition(update, FlightState::Safe, ReasonCode::None,
                   sample.timestampMs);
      }
      break;

    case FlightState::Safe:
      if (command == ControlCommand::Arm) {
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

    // 두 단이 붙어 있으므로 1단과 2단이 같은 연소종료를 본다.
    case FlightState::Launch:
    case FlightState::Ride:
      if (sample.imuValid &&
          elapsed(sample.timestampMs, launchMs_,
                  FlightConfig::kBurnoutEarliestMs)) {
        burnoutCount_ = sample.accelerationG <= FlightConfig::kBurnoutThresholdG
                            ? static_cast<uint8_t>(burnoutCount_ + 1)
                            : 0;
      }
      if (burnoutCount_ >= FlightConfig::kBurnoutSamples) {
        transition(update, FlightState::Coast, ReasonCode::BurnoutDetected,
                   sample.timestampMs);
      } else if (elapsed(sample.timestampMs, launchMs_,
                         FlightConfig::kBurnoutTimeoutMs)) {
        transition(update, FlightState::Coast, ReasonCode::BurnoutTimeout,
                   sample.timestampMs);
      }
      break;

    case FlightState::Coast:
      updateApogee(update, sample);
      break;

    case FlightState::Apogee:
      runDeploySequence(update, sample);
      break;

    case FlightState::Deploy: {
      if (!sample.barometerValid) {
        // 기압이 없으면 고도 변화량이 0 으로 보여 "움직이지 않음 = 착지"로
        // 오판한다. 창을 계속 초기화해 두고 아래의 절대 타임아웃에만
        // 의존한다.
        landingWindowStartMs_ = sample.timestampMs;
        landingMinAltitudeM_ = sample.altitudeM;
        landingMaxAltitudeM_ = sample.altitudeM;
        if (elapsed(sample.timestampMs, apogeeMs_,
                    FlightConfig::kLandingTimeoutMs)) {
          transition(update, FlightState::Ground, ReasonCode::LandingTimeout,
                     sample.timestampMs);
        }
        break;
      }
      landingMinAltitudeM_ = std::min(landingMinAltitudeM_, sample.altitudeM);
      landingMaxAltitudeM_ = std::max(landingMaxAltitudeM_, sample.altitudeM);
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
      if (elapsed(sample.timestampMs, apogeeMs_,
                  FlightConfig::kLandingTimeoutMs)) {
        transition(update, FlightState::Ground, ReasonCode::LandingTimeout,
                   sample.timestampMs);
      }
      break;
    }

    // 지금 구성에서는 진입하지 않는 상태들.
    case FlightState::Separating:
    case FlightState::Separated:
    case FlightState::Igniting:
    case FlightState::Boost:
    case FlightState::Recovery:
    case FlightState::Ground:
    case FlightState::Fault:
      break;
  }

  const bool actuatorValid = update.actuatorCommandValid;
  const ActuatorCommand actuator = update.actuator;
  const bool stateChanged = update.stateChanged;

  update.snapshot = snapshot();
  update.snapshot.timestampMs = sample.timestampMs;
  update.snapshot.sensor = lastSample_;
  update.stateChanged = stateChanged;
  update.actuatorCommandValid = actuatorValid;
  update.actuator = actuator;
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
  result.separationSeen = separationSeen_;
  result.heaterArmed = heaterArmed_;
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
