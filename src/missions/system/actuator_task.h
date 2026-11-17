#pragma once

#include "hal/bench_actuator_output.h"
#include "runtime_channels.h"

class ActuatorTask {
 public:
  ActuatorTask(RuntimeChannels& channels, BenchActuatorOutput& output)
      : channels_(channels), output_(output) {}
  bool start(BaseType_t core);

 private:
  static void entry(void* context);
  void run();
  RuntimeChannels& channels_;
  BenchActuatorOutput& output_;
};
