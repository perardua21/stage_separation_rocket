#include "missions/system/command_task.h"

#include <Arduino.h>

bool CommandTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "console", 3072, this, 1, nullptr,
                                 core) == pdPASS;
}

void CommandTask::entry(void* context) {
  static_cast<CommandTask*>(context)->run();
}

void CommandTask::run() {
  String line;
  for (;;) {
    while (Serial.available()) {
      const char value = static_cast<char>(Serial.read());
      if (value == '\n' || value == '\r') {
        line.trim();
        ControlCommand command = ControlCommand::None;
        if (line.equalsIgnoreCase("arm")) command = ControlCommand::Arm;
        if (line.equalsIgnoreCase("disarm")) command = ControlCommand::Disarm;
        if (line.equalsIgnoreCase("sep")) {
          command = ControlCommand::BenchSeparationDetected;
        }
        if (command != ControlCommand::None) {
          xQueueSend(channels_.command, &command, 0);
          Serial.printf("bench command accepted: %s\n", line.c_str());
        } else if (line.length() > 0) {
          Serial.println("commands: arm | disarm | sep");
        }
        line = "";
      } else if (line.length() < 31) {
        line += value;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
