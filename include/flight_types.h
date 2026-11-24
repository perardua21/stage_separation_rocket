#pragma once

#include <stdint.h>

enum class VehicleRole : uint8_t { Booster = 0, Sustainer = 1 };

// 현재 구성에서 실제로 사용하는 흐름
//   1단: Init -> Safe -> Armed -> Launch -> Coast -> Apogee -> Deploy -> Ground
//   2단: Init -> Safe -> Armed -> Ride   -> Coast -> Apogee -> Deploy -> Ground
//
// Apogee 는 "정점 판정 + 화약 발사 시퀀스 실행" 구간이고, Deploy 는 "사출
// 이후 하강 감시" 구간이다. 이름이 직관과 어긋나지만 지상 로그 파서와
// 기존 문서가 이미 이 이름을 쓰고 있어 유지한다.
//
// Separating / Separated / Igniting / Boost / Recovery 는 2단 자체 점화가
// 있던 설계의 잔재다. 지금 구성에서는 진입하지 않는다. 나중에 2단 모터를
// 달면 다시 쓸 수 있도록 enum 값만 남겨 둔다.
enum class FlightState : uint8_t {
  Init,
  Safe,
  Armed,
  Launch,      // 1단: 추진 구간 (스택 전체가 올라감)
  Separating,  // 미사용
  Ride,        // 2단: 1단에 실려 올라가는 구간
  Separated,   // 미사용
  Igniting,    // 미사용
  Boost,       // 미사용
  Coast,       // 추진 종료 후 정점까지
  Recovery,    // 미사용
  Apogee,      // 정점 판정 + 화약 발사 시퀀스
  Deploy,      // 사출 후 하강 감시
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
  Arm,
  Disarm,
  BenchSeparationDetected,
};

enum class ActuatorChannel : uint8_t {
  None,
  Heater1,   // 1단: 분리 + 1단 낙하산 (한 발) / 2단: 미사용
  Heater2,   // 1단: 백업(장착 시) / 2단: 2단 낙하산
  CommonArm, // HEAT_ARM. 두 채널 공통 전원 스위치
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

enum class SensorSource : uint8_t { Imu, Barometer, Power, StageLink };

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
  bool separationDetected = false;
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
  bool ignitionPermanentlyInhibited = true;  // 2단 점화 미사용. 항상 true
  bool separationSeen = false;
  bool heaterArmed = false;
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
