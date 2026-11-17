#include "hal/bench_actuator_output.h"

#include <Arduino.h>

#include "board_pins.h"

void BenchActuatorOutput::begin() {
  pinMode(BoardPins::kHeater1, OUTPUT);
  pinMode(BoardPins::kHeater2, OUTPUT);
  pinMode(BoardPins::kHeaterArm, OUTPUT);
  allOff();
}

void BenchActuatorOutput::allOff() {
  digitalWrite(BoardPins::kHeater1, LOW);
  digitalWrite(BoardPins::kHeater2, LOW);
  digitalWrite(BoardPins::kHeaterArm, LOW);
}

void BenchActuatorOutput::apply(const ActuatorCommand& command) {
  // The only supplied build is bench-safe. Requests are observable but never
  // reach the MOSFET outputs.
  allOff();
  Serial.printf("[BENCH BLOCKED] actuator=%u requested=%u at %lu ms\n",
                static_cast<unsigned>(command.channel), command.enabled,
                static_cast<unsigned long>(command.timestampMs));
}
