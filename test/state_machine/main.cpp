#include <cassert>
#include <cstdio>
#include <vector>

#include "core/flight_state_machine.h"
#include "flight_config.h"

namespace {

struct Fired {
  uint32_t ms;
  ActuatorChannel channel;
  bool enabled;
};

// 테스트용 구동기. FSM 이 낸 명령을 순서대로 기록하고, 실제 하드웨어
// 인터락과 같은 규칙을 어기면 그 자리에서 터진다.
class Recorder {
 public:
  void apply(const ActuatorCommand& c) {
    if (c.channel == ActuatorChannel::CommonArm) {
      if (c.enabled) {
        assert(!armed_ && "ARM twice");
        armed_ = true;
      } else {
        assert(active_ == ActuatorChannel::None && "ARM off while ch active");
        armed_ = false;
      }
    } else if (c.enabled) {
      assert(armed_ && "channel fired without ARM");
      assert(active_ == ActuatorChannel::None && "two channels at once");
      active_ = c.channel;
    } else {
      active_ = ActuatorChannel::None;
    }
    log.push_back({c.timestampMs, c.channel, c.enabled});
  }

  bool armed() const { return armed_; }
  std::vector<Fired> log;

 private:
  bool armed_ = false;
  ActuatorChannel active_ = ActuatorChannel::None;
};

constexpr uint32_t kDtMs = 10;

struct Rig {
  FlightStateMachine machine;
  Recorder recorder;
  SensorSample sample{};
  uint32_t nowMs = 0;

  explicit Rig(VehicleRole role) : machine(role) {
    sample.imuValid = true;
    sample.barometerValid = true;
    sample.accelerationG = 1.0f;
  }

  FlightState tick(ControlCommand command = ControlCommand::None) {
    nowMs += kDtMs;
    sample.timestampMs = nowMs;
    StateMachineUpdate update = machine.step(sample, command);
    if (update.actuatorCommandValid) recorder.apply(update.actuator);
    return update.snapshot.state;
  }

  void run(uint32_t durationMs) {
    const uint32_t until = nowMs + durationMs;
    while (nowMs < until) tick();
  }

  FlightState state() { return machine.snapshot().state; }
};

void expectFired(const Recorder& r, ActuatorChannel primary) {
  // ARM on -> primary on -> primary off -> ARM off
  assert(r.log.size() == 4);
  assert(r.log[0].channel == ActuatorChannel::CommonArm && r.log[0].enabled);
  assert(r.log[1].channel == primary && r.log[1].enabled);
  assert(r.log[2].channel == primary && !r.log[2].enabled);
  assert(r.log[3].channel == ActuatorChannel::CommonArm && !r.log[3].enabled);
  // 펄스 폭이 설정값 이상 유지되었는지 확인한다.
  assert(r.log[2].ms - r.log[1].ms >= FlightConfig::kPyroPulseMs);
  assert(!r.armed());
}

// 이륙 -> 연소 종료 -> 정점까지 올려 보낸 뒤 하강시킨다.
void flyToApogee(Rig& rig) {
  // Init -> Safe -> Armed
  rig.tick();
  assert(rig.state() == FlightState::Safe);
  rig.tick(ControlCommand::Arm);
  assert(rig.state() == FlightState::Armed);

  // 이륙
  rig.sample.accelerationG = 8.0f;
  for (int i = 0; i < FlightConfig::kLaunchSamples + 1; ++i) rig.tick();
  const FlightState boosting = rig.state();
  assert(boosting == FlightState::Launch || boosting == FlightState::Ride);

  // 연소 종료
  rig.run(FlightConfig::kBurnoutEarliestMs + 50);
  rig.sample.accelerationG = 0.2f;
  for (int i = 0; i < FlightConfig::kBurnoutSamples + 1; ++i) rig.tick();
  assert(rig.state() == FlightState::Coast);

  // 상승. 최소 비행시간 가드를 지날 때까지 고도를 올린다.
  float altitude = 0.0f;
  while (rig.nowMs < FlightConfig::kApogeeEarliestFromLaunchMs + 500) {
    altitude += 0.5f;
    rig.sample.altitudeM = altitude;
    rig.sample.verticalVelocityMps = 20.0f;
    rig.tick();
    assert(rig.state() == FlightState::Coast && "상승 중 조기 사출");
  }

  // 하강 시작
  rig.sample.verticalVelocityMps = -5.0f;
  for (int i = 0; i < FlightConfig::kApogeeSamples + 2; ++i) {
    altitude -= 1.0f;
    rig.sample.altitudeM = altitude;
    rig.tick();
  }
  assert(rig.state() == FlightState::Apogee);
}

void testBoosterSingleCharge() {
  Rig rig(VehicleRole::Booster);
  flyToApogee(rig);

  // 화약 시퀀스가 끝나면 Deploy 로 넘어간다.
  rig.run(FlightConfig::kPyroPulseMs + 500);
  assert(rig.state() == FlightState::Deploy);
  // 1단은 CH1 한 발로 분리와 낙하산을 같이 한다.
  expectFired(rig.recorder, ActuatorChannel::Heater1);

  // 착지
  rig.sample.verticalVelocityMps = 0.0f;
  rig.run(FlightConfig::kLandingWindowMs + 200);
  assert(rig.state() == FlightState::Ground);
}

void testSustainerChute() {
  Rig rig(VehicleRole::Sustainer);
  flyToApogee(rig);

  // 2단은 분리가 끝날 시간을 준 뒤 CH2 로 자기 낙하산만 쏜다.
  const uint32_t apogeeMs = rig.nowMs;
  rig.run(FlightConfig::kSustainerDeployDelayMs + FlightConfig::kPyroPulseMs +
          500);
  assert(rig.state() == FlightState::Deploy);
  expectFired(rig.recorder, ActuatorChannel::Heater2);
  // 분리가 끝날 시간을 실제로 기다렸는지 확인한다. ARM 이 정점 직후가
  // 아니라 지연 뒤에 올라가야 한다.
  assert(rig.recorder.log[0].ms - apogeeMs >=
         FlightConfig::kSustainerDeployDelayMs);
}

// 기압계가 죽으면 고도 변화량이 0 으로 보인다. 그걸 "착지"로 오판해서는
// 안 된다.
void testNoFalseLandingWithoutBarometer() {
  Rig rig(VehicleRole::Booster);
  flyToApogee(rig);
  rig.run(FlightConfig::kPyroPulseMs + 500);
  assert(rig.state() == FlightState::Deploy);

  rig.sample.barometerValid = false;
  rig.run(FlightConfig::kLandingWindowMs * 3);
  assert(rig.state() == FlightState::Deploy && "기압 없이 착지 오판");

  // 절대 타임아웃으로만 끝난다.
  rig.run(FlightConfig::kLandingTimeoutMs);
  assert(rig.state() == FlightState::Ground);
  assert(rig.machine.snapshot().reason == ReasonCode::LandingTimeout);
}

// IMU 가 비행 중 멈춰도(imuValid=false) 기압만으로 정점을 잡고 사출한다.
void testApogeeWithDeadImuMidFlight() {
  Rig rig(VehicleRole::Booster);
  rig.tick();
  rig.tick(ControlCommand::Arm);
  rig.sample.accelerationG = 8.0f;
  for (int i = 0; i < FlightConfig::kLaunchSamples + 1; ++i) rig.tick();
  rig.run(FlightConfig::kBurnoutEarliestMs + 50);
  rig.sample.accelerationG = 0.2f;
  for (int i = 0; i < FlightConfig::kBurnoutSamples + 1; ++i) rig.tick();
  assert(rig.state() == FlightState::Coast);

  // 여기서 IMU 가 죽는다. 기압은 계속 살아 있다.
  rig.sample.imuValid = false;
  float altitude = 0.0f;
  while (rig.nowMs < FlightConfig::kApogeeEarliestFromLaunchMs + 500) {
    altitude += 0.5f;
    rig.sample.altitudeM = altitude;
    rig.sample.verticalVelocityMps = 20.0f;
    rig.tick();
    assert(rig.state() == FlightState::Coast);
  }
  rig.sample.verticalVelocityMps = -5.0f;
  for (int i = 0; i < FlightConfig::kApogeeSamples + 2; ++i) {
    altitude -= 1.0f;
    rig.sample.altitudeM = altitude;
    rig.tick();
  }
  assert(rig.state() == FlightState::Apogee);
  rig.run(FlightConfig::kPyroPulseMs + 500);
  expectFired(rig.recorder, ActuatorChannel::Heater1);
}

// 기압계가 죽어도 이륙 후 타임아웃으로 반드시 사출한다.
void testApogeeTimeoutFiresWithoutBarometer() {
  Rig rig(VehicleRole::Booster);
  rig.tick();
  rig.tick(ControlCommand::Arm);
  rig.sample.accelerationG = 8.0f;
  for (int i = 0; i < FlightConfig::kLaunchSamples + 1; ++i) rig.tick();
  rig.run(FlightConfig::kBurnoutEarliestMs + 50);
  rig.sample.accelerationG = 0.2f;
  for (int i = 0; i < FlightConfig::kBurnoutSamples + 1; ++i) rig.tick();
  assert(rig.state() == FlightState::Coast);

  rig.sample.barometerValid = false;
  rig.run(FlightConfig::kApogeeTimeoutFromLaunchMs + 100);
  assert(rig.state() == FlightState::Apogee ||
         rig.state() == FlightState::Deploy);
  rig.run(FlightConfig::kPyroPulseMs + 500);
  expectFired(rig.recorder, ActuatorChannel::Heater1);
}

// 기압을 못 믿는 2단은 단간 선이 끊긴 것을 정점 신호로 받아들인다.
void testSustainerSeparationFallback() {
  Rig rig(VehicleRole::Sustainer);
  rig.tick();
  rig.tick(ControlCommand::Arm);
  rig.sample.accelerationG = 8.0f;
  for (int i = 0; i < FlightConfig::kLaunchSamples + 1; ++i) rig.tick();
  rig.run(FlightConfig::kBurnoutEarliestMs + 50);
  rig.sample.accelerationG = 0.2f;
  for (int i = 0; i < FlightConfig::kBurnoutSamples + 1; ++i) rig.tick();
  assert(rig.state() == FlightState::Coast);

  rig.sample.barometerValid = false;
  rig.run(500);
  assert(rig.state() == FlightState::Coast);

  rig.sample.separationDetected = true;
  rig.tick();
  assert(rig.state() == FlightState::Apogee);
  assert(rig.machine.snapshot().reason == ReasonCode::SeparationDetected);
}

// 모터 불발로 기체가 거의 올라가지 못한 경우. 고도가 음수로 내려가도
// 기압 기반 정점 판정으로 지상에서 화약이 터져서는 안 된다.
void testNoGroundLevelFiring() {
  Rig rig(VehicleRole::Booster);
  rig.tick();
  rig.tick(ControlCommand::Arm);
  rig.sample.accelerationG = 8.0f;
  for (int i = 0; i < FlightConfig::kLaunchSamples + 1; ++i) rig.tick();
  rig.run(FlightConfig::kBurnoutEarliestMs + 50);
  rig.sample.accelerationG = 0.2f;
  for (int i = 0; i < FlightConfig::kBurnoutSamples + 1; ++i) rig.tick();
  assert(rig.state() == FlightState::Coast);

  // 5 m 까지만 올라갔다가 주저앉는다.
  float altitude = 0.0f;
  for (int i = 0; i < 100; ++i) {
    altitude += 0.05f;
    rig.sample.altitudeM = altitude;
    rig.sample.verticalVelocityMps = 5.0f;
    rig.tick();
  }
  rig.sample.verticalVelocityMps = -3.0f;
  for (int i = 0; i < 200; ++i) {
    altitude -= 0.1f;
    rig.sample.altitudeM = altitude;
    rig.tick();
  }
  // 최소 고도 가드에 걸려 기압 판정으로는 사출되지 않는다.
  assert(rig.state() == FlightState::Coast && "지상에서 화약 발사");
  assert(rig.recorder.log.empty());

  // 타임아웃 경로로는 사출된다. 지상에 떨어진 기체의 낙하산을 뒤늦게라도
  // 터뜨리는 쪽이, 하강 중인 기체를 방치하는 쪽보다 낫다.
  rig.run(FlightConfig::kApogeeTimeoutFromLaunchMs);
  rig.run(FlightConfig::kPyroPulseMs + 500);
  expectFired(rig.recorder, ActuatorChannel::Heater1);
}

// ARM 하지 않으면 어떤 가속도에도 이륙으로 넘어가지 않는다.
void testNoLaunchWhileSafe() {
  Rig rig(VehicleRole::Booster);
  rig.tick();
  assert(rig.state() == FlightState::Safe);
  rig.sample.accelerationG = 12.0f;
  rig.run(2000);
  assert(rig.state() == FlightState::Safe);
  assert(rig.recorder.log.empty());
}

// Disarm 으로 Safe 로 되돌아간다.
void testDisarm() {
  Rig rig(VehicleRole::Booster);
  rig.tick();
  rig.tick(ControlCommand::Arm);
  assert(rig.state() == FlightState::Armed);
  rig.tick(ControlCommand::Disarm);
  assert(rig.state() == FlightState::Safe);
}

}  // namespace

int main() {
  testNoLaunchWhileSafe();
  testDisarm();
  testBoosterSingleCharge();
  testSustainerChute();
  testApogeeTimeoutFiresWithoutBarometer();
  testSustainerSeparationFallback();
  testNoFalseLandingWithoutBarometer();
  testApogeeWithDeadImuMidFlight();
  testNoGroundLevelFiring();
  std::printf("state-machine tests passed\n");
  return 0;
}
