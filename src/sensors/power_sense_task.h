#pragma once

#include "hal/battery_voltage_hal.h"
#include "runtime_channels.h"

class PowerSenseTask {
 public:
  PowerSenseTask(IBatteryVoltage& battery, RuntimeChannels& channels,
                 RuntimeStats& stats)
      : battery_(battery), channels_(channels), stats_(stats) {}
  bool start(BaseType_t core);

 private:
  static void entry(void* context);
  void run();
  IBatteryVoltage& battery_;
  RuntimeChannels& channels_;
  RuntimeStats& stats_;
};
