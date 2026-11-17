#pragma once

#include <stdint.h>

enum class VehicleRole : uint8_t { Booster = 0, Sustainer = 1 };

enum class FlightState : uint8_t {
  Init,
  Safe,
  Armed,
  Launch,
  Separating,
  Ride,
  Separated,
  Igniting,
  Boost,
  Coast,
  Recovery,
  Apogee,
  Deploy,
  Ground,
  Fault,
};

enum class ReasonCode : uint8_t {
  None,
  InitFailed,
  LaunchDetected,
  BurnoutDetected,
  BurnoutTimeout,
  SeparationDetected,
  NoSeparation,
  Tilt,
  LowVelocity,
  LowAltitude,
  IgnitionCommanded,
  IgnitionDetected,
  NoIgnition,
  BoostBurnout,
  BoostTimeout,
  ApogeeDetected,
  ApogeeTimeout,
  Landed,
  LandingTimeout,
};

enum class ControlCommand : uint8_t {
  None,
  BenchArm,
  Disarm,
  BenchSeparationDetected,
};

enum class ActuatorChannel : uint8_t {
  None,
  Heater1,
  Heater2,
  CommonArm,
};

struct SensorSample {
  uint32_t timestampMs = 0;
  float accelerationG = 1.0f;
  float altitudeM = 0.0f;
  float verticalVelocityMps = 0.0f;
  float tiltDeg = 0.0f;
  float integratedVelocityMps = 0.0f;
  float batteryV = 0.0f;
  bool imuValid = false;
  bool barometerValid = false;
  bool separationDetected = false;
};

enum class SensorSource : uint8_t { Imu, Barometer, Power };

// Queue message produced by one sensor task. The FSM task merges messages into
// its private SensorSample; producers never share mutable sensor state.
struct SensorMessage {
  SensorSource source = SensorSource::Imu;
  uint32_t timestampMs = 0;
  float accelerationG = 1.0f;
  float tiltDeg = 0.0f;
  float integratedVelocityMps = 0.0f;
  float altitudeM = 0.0f;
  float verticalVelocityMps = 0.0f;
  float batteryV = 0.0f;
  bool valid = false;
};

struct ActuatorCommand {
  uint32_t timestampMs = 0;
  ActuatorChannel channel = ActuatorChannel::None;
  bool enabled = false;
};

struct FlightSnapshot {
  uint32_t timestampMs = 0;
  VehicleRole role = VehicleRole::Booster;
  FlightState state = FlightState::Init;
  ReasonCode reason = ReasonCode::None;
  SensorSample sensor{};
  float maxAltitudeM = 0.0f;
  bool ignitionPermanentlyInhibited = true;
  uint32_t droppedLogFrames = 0;
  uint32_t droppedTelemetryFrames = 0;
};

struct StateMachineUpdate {
  FlightSnapshot snapshot{};
  bool stateChanged = false;
  bool actuatorCommandValid = false;
  ActuatorCommand actuator{};
};

const char* flightStateName(FlightState state);
const char* reasonCodeName(ReasonCode reason);
