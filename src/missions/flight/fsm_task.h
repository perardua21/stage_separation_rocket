#pragma once

#include "core/flight_state_machine.h"
#include "runtime_channels.h"

class FsmTask {
 public:
  FsmTask(VehicleRole role, RuntimeChannels& channels, RuntimeStats& stats)
      : machine_(role), channels_(channels), stats_(stats) {}
  bool start(BaseType_t core);

 private:
  static void entry(void* context);
  void run();
  FlightStateMachine machine_;
  RuntimeChannels& channels_;
  RuntimeStats& stats_;
};
