#pragma once

#include <SD.h>
#include <SPI.h>

#include "runtime_channels.h"

class FlightLogTask {
 public:
  explicit FlightLogTask(RuntimeChannels& channels)
      : channels_(channels), sdSpi_(HSPI) {}
  bool start(BaseType_t core);

 private:
  static void entry(void* context);
  void run();
  RuntimeChannels& channels_;
  SPIClass sdSpi_;
};

void writeCsvRow(Print& out, const FlightSnapshot& frame);
