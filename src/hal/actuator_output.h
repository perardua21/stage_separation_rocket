#pragma once

#include "flight_types.h"

// 액추에이터 출력 인터페이스. 벤치 빌드는 전부 차단하는 구현을, 비행 빌드는
// 실제 핀을 구동하는 구현을 꽂는다. ActuatorTask 는 어느 쪽인지 모른다.
class IActuatorOutput {
 public:
  virtual ~IActuatorOutput() = default;
  virtual void begin() = 0;
  virtual void allOff() = 0;
  virtual void apply(const ActuatorCommand& command) = 0;
  virtual bool armed() const = 0;
};
