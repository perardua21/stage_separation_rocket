#pragma once

#include "hal/bmp388_hal.h"
#include "runtime_channels.h"

class BarometerTask {
 public:
  BarometerTask(IBarometer& barometer, RuntimeChannels& channels,
                RuntimeStats& stats)
      : barometer_(barometer), channels_(channels), stats_(stats) {}
  bool start(BaseType_t core);

 private:
  static void entry(void* context);
  void run();
  IBarometer& barometer_;
  RuntimeChannels& channels_;
  RuntimeStats& stats_;
};
