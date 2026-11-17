#pragma once

#include "hal/bno085_hal.h"
#include "runtime_channels.h"

class ImuTask {
 public:
  ImuTask(ILowGImu& imu, RuntimeChannels& channels, RuntimeStats& stats)
      : imu_(imu), channels_(channels), stats_(stats) {}
  bool start(BaseType_t core);

 private:
  static void entry(void* context);
  void run();
  ILowGImu& imu_;
  RuntimeChannels& channels_;
  RuntimeStats& stats_;
};
