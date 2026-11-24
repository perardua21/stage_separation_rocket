#include "hal/flight_actuator_output.h"

#include <Arduino.h>

#include "board_pins.h"

// 회로상 active HIGH 로 확인되었다. 혹시 PCB 개정으로 반전되면 이 한 줄만
// 0 으로 바꾼다.
#ifndef NURA_HEATER_ACTIVE_HIGH
#define NURA_HEATER_ACTIVE_HIGH 1
#endif

namespace {
#if NURA_HEATER_ACTIVE_HIGH
constexpr int kOn = HIGH;
constexpr int kOff = LOW;
#else
constexpr int kOn = LOW;
constexpr int kOff = HIGH;
#endif

int pinFor(ActuatorChannel channel) {
  switch (channel) {
    case ActuatorChannel::Heater1: return BoardPins::kHeater1;
    case ActuatorChannel::Heater2: return BoardPins::kHeater2;
    default: return -1;
  }
}
}  // namespace

void FlightActuatorOutput::begin() {
  // 핀을 출력으로 바꾸기 전에 레벨부터 확정해 두어야 중간에 뜨는 구간이 없다.
  digitalWrite(BoardPins::kHeaterArm, kOff);
  digitalWrite(BoardPins::kHeater1, kOff);
  digitalWrite(BoardPins::kHeater2, kOff);
  pinMode(BoardPins::kHeaterArm, OUTPUT);
  pinMode(BoardPins::kHeater1, OUTPUT);
  pinMode(BoardPins::kHeater2, OUTPUT);
  allOff();
}

void FlightActuatorOutput::allOff() {
  // 채널을 먼저 내리고 공통 전원을 내린다. 반대 순서면 전원이 끊기는 순간
  // 채널이 켜진 채로 남는다.
  digitalWrite(BoardPins::kHeater1, kOff);
  digitalWrite(BoardPins::kHeater2, kOff);
  digitalWrite(BoardPins::kHeaterArm, kOff);
  armed_ = false;
  active_ = ActuatorChannel::None;
}

void FlightActuatorOutput::writeChannel(ActuatorChannel channel, bool on) {
  const int pin = pinFor(channel);
  if (pin < 0) return;
  digitalWrite(pin, on ? kOn : kOff);
}

void FlightActuatorOutput::apply(const ActuatorCommand& command) {
  if (command.channel == ActuatorChannel::CommonArm) {
    if (command.enabled) {
      armed_ = true;
      digitalWrite(BoardPins::kHeaterArm, kOn);
      Serial.printf("[PYRO] ARM on at %lu ms\n",
                    static_cast<unsigned long>(command.timestampMs));
    } else {
      // 해제할 때는 채널부터 확실히 내린다.
      writeChannel(ActuatorChannel::Heater1, false);
      writeChannel(ActuatorChannel::Heater2, false);
      active_ = ActuatorChannel::None;
      armed_ = false;
      digitalWrite(BoardPins::kHeaterArm, kOff);
      Serial.printf("[PYRO] ARM off at %lu ms\n",
                    static_cast<unsigned long>(command.timestampMs));
    }
    return;
  }

  if (pinFor(command.channel) < 0) return;

  if (!command.enabled) {
    writeChannel(command.channel, false);
    if (active_ == command.channel) active_ = ActuatorChannel::None;
    Serial.printf("[PYRO] ch%u off at %lu ms\n",
                  static_cast<unsigned>(command.channel),
                  static_cast<unsigned long>(command.timestampMs));
    return;
  }

  if (!armed_) {
    Serial.printf("[PYRO] ch%u REFUSED: not armed\n",
                  static_cast<unsigned>(command.channel));
    return;
  }
  if (active_ != ActuatorChannel::None && active_ != command.channel) {
    // 15 A 퓨즈. 두 채널 동시 구동은 어떤 경우에도 허용하지 않는다.
    Serial.printf("[PYRO] ch%u REFUSED: ch%u already active\n",
                  static_cast<unsigned>(command.channel),
                  static_cast<unsigned>(active_));
    return;
  }

  active_ = command.channel;
  writeChannel(command.channel, true);
  Serial.printf("[PYRO] ch%u ON at %lu ms\n",
                static_cast<unsigned>(command.channel),
                static_cast<unsigned long>(command.timestampMs));
}
