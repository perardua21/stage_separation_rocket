#pragma once

#include "hal/actuator_output.h"
#include "runtime_channels.h"

// 열선 핀을 만지는 유일한 태스크. FSM 은 큐로 요청만 한다.
class ActuatorTask {
 public:
  ActuatorTask(RuntimeChannels& channels, IActuatorOutput& output)
      : channels_(channels), output_(output) {}
  bool start(BaseType_t core);

 private:
  static void entry(void* context);
  void run();
  RuntimeChannels& channels_;
  IActuatorOutput& output_;
};
