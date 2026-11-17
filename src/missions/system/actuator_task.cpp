#include "missions/system/actuator_task.h"

bool ActuatorTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "actuator", 4096, this, 6, nullptr,
                                 core) == pdPASS;
}

void ActuatorTask::entry(void* context) {
  static_cast<ActuatorTask*>(context)->run();
}

void ActuatorTask::run() {
  output_.allOff();
  ActuatorCommand command;
  for (;;) {
    if (xQueueReceive(channels_.actuator, &command, portMAX_DELAY) == pdTRUE) {
      output_.apply(command);
    }
  }
}
