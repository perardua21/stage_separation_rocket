#include <assert.h>
#include <stdio.h>

#include "core/flight_state_machine.h"

namespace {
SensorSample sample(uint32_t ms, float accelG, float altitudeM,
                    float velocityMps = 0.0f) {
  SensorSample value;
  value.timestampMs = ms;
  value.accelerationG = accelG;
  value.altitudeM = altitudeM;
  value.verticalVelocityMps = velocityMps;
  value.integratedVelocityMps = velocityMps;
  value.imuValid = true;
  value.barometerValid = true;
  return value;
}

void initializeAndArm(FlightStateMachine& machine) {
  machine.step(sample(0, 1.0f, 0.0f));
  assert(machine.snapshot().state == FlightState::Safe);
  machine.step(sample(10, 1.0f, 0.0f), ControlCommand::BenchArm);
  assert(machine.snapshot().state == FlightState::Armed);
}

void launch(FlightStateMachine& machine, uint32_t startMs) {
  for (uint32_t i = 0; i < 5; ++i) {
    machine.step(sample(startMs + i * 10, 3.0f, i * 0.5f, 20.0f));
  }
}

void testBoosterNominal() {
  FlightStateMachine machine(VehicleRole::Booster);
  initializeAndArm(machine);
  launch(machine, 100);
  assert(machine.snapshot().state == FlightState::Launch);
  for (uint32_t i = 0; i < 5; ++i) {
    machine.step(sample(700 + i * 10, 1.0f, 40.0f + i, 50.0f));
  }
  assert(machine.snapshot().state == FlightState::Separating);
  machine.step(sample(1500, 0.5f, 65.0f, 60.0f),
               ControlCommand::BenchSeparationDetected);
  assert(machine.snapshot().state == FlightState::Coast);
  machine.step(sample(5100, 0.2f, 181.0f, 1.0f));
  for (uint32_t i = 0; i < 5; ++i) {
    machine.step(sample(5900 + i * 50, 0.2f, 177.0f - i, -2.0f));
    SensorSample imuOnly = sample(5910 + i * 50, 0.2f, 0.0f, 0.0f);
    imuOnly.barometerValid = false;
    machine.step(imuOnly);
  }
  assert(machine.snapshot().state == FlightState::Apogee);
}

void testSustainerNominal() {
  FlightStateMachine machine(VehicleRole::Sustainer);
  initializeAndArm(machine);
  launch(machine, 100);
  assert(machine.snapshot().state == FlightState::Ride);
  SensorSample separation = sample(1500, 0.5f, 65.0f, 64.0f);
  separation.integratedVelocityMps = 64.0f;
  machine.step(separation, ControlCommand::BenchSeparationDetected);
  assert(machine.snapshot().state == FlightState::Separated);
  SensorSample ignition = sample(2000, 0.5f, 95.0f, 57.0f);
  ignition.integratedVelocityMps = 57.0f;
  ignition.tiltDeg = 5.0f;
  StateMachineUpdate update = machine.step(ignition);
  assert(machine.snapshot().state == FlightState::Igniting);
  assert(update.actuatorCommandValid);
  for (uint32_t i = 0; i < 5; ++i) {
    machine.step(sample(2050 + i * 10, 5.0f, 100.0f + i, 60.0f));
  }
  assert(machine.snapshot().state == FlightState::Boost);
  update = machine.step(sample(3000, 5.0f, 200.0f, 90.0f));
  assert(update.actuatorCommandValid);
  assert(update.actuator.channel == ActuatorChannel::Heater1);
  assert(!update.actuator.enabled);
  for (uint32_t i = 0; i < 5; ++i) {
    machine.step(sample(3150 + i * 10, 1.5f, 250.0f + i, 100.0f));
  }
  assert(machine.snapshot().state == FlightState::Coast);
  machine.step(sample(12400, 0.2f, 717.0f, 0.2f));
  for (uint32_t i = 0; i < 5; ++i) {
    machine.step(sample(13000 + i * 50, 0.2f, 713.0f - i, -2.0f));
  }
  assert(machine.snapshot().state == FlightState::Apogee);
}

void testNoSeparationRecovery() {
  FlightStateMachine machine(VehicleRole::Sustainer);
  initializeAndArm(machine);
  launch(machine, 100);
  machine.step(sample(2700, 1.0f, 120.0f, 20.0f));
  assert(machine.snapshot().state == FlightState::Recovery);
  assert(machine.snapshot().reason == ReasonCode::NoSeparation);
  assert(machine.snapshot().ignitionPermanentlyInhibited);
}

void testTiltInhibitsIgnition() {
  FlightStateMachine machine(VehicleRole::Sustainer);
  initializeAndArm(machine);
  launch(machine, 100);
  SensorSample separation = sample(1500, 0.5f, 65.0f, 64.0f);
  separation.integratedVelocityMps = 64.0f;
  machine.step(separation, ControlCommand::BenchSeparationDetected);
  SensorSample tilted = sample(1600, 0.5f, 70.0f, 60.0f);
  tilted.integratedVelocityMps = 60.0f;
  tilted.tiltDeg = 20.0f;
  machine.step(tilted);
  assert(machine.snapshot().state == FlightState::Recovery);
  assert(machine.snapshot().reason == ReasonCode::Tilt);
  assert(machine.snapshot().ignitionPermanentlyInhibited);
}
}  // namespace

int main() {
  testBoosterNominal();
  testSustainerNominal();
  testNoSeparationRecovery();
  testTiltInhibitsIgnition();
  puts("state-machine tests passed");
  return 0;
}
