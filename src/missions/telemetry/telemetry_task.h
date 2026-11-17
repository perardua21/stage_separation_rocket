#pragma once

#include <HardwareSerial.h>

#include "runtime_channels.h"

class TelemetryTask {
 public:
  explicit TelemetryTask(RuntimeChannels& channels)
      : channels_(channels), radio_(1) {}
  bool start(BaseType_t core);

 private:
  static void entry(void* context);
  void run();
  RuntimeChannels& channels_;
  HardwareSerial radio_;
};
