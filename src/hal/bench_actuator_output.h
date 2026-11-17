#pragma once

#include "flight_types.h"

class BenchActuatorOutput {
 public:
  void begin();
  void allOff();
  void apply(const ActuatorCommand& command);
};
