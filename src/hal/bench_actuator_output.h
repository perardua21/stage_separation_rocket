#pragma once

#include "flight_types.h"
#include "hal/actuator_output.h"

// 벤치 빌드 전용. 요청은 시리얼로 보이지만 MOSFET 까지 가지 않는다.
class BenchActuatorOutput : public IActuatorOutput {
 public:
  void begin() override;
  void allOff() override;
  void apply(const ActuatorCommand& command) override;
  bool armed() const override { return false; }
};
