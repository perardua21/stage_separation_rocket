#pragma once

#include "runtime_channels.h"

class CommandTask {
 public:
  explicit CommandTask(RuntimeChannels& channels) : channels_(channels) {}
  bool start(BaseType_t core);

 private:
  static void entry(void* context);
  void run();
  RuntimeChannels& channels_;
};
