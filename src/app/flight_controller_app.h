#pragma once

#include <optional>

#include "hal/bench_actuator_output.h"
#include "hal/battery_voltage_hal.h"
#include "hal/bmp388_hal.h"
#include "hal/bno085_hal.h"
#include "missions/flight/fsm_task.h"
#include "missions/logging/flight_log_task.h"
#include "missions/system/actuator_task.h"
#include "missions/system/command_task.h"
#include "missions/telemetry/telemetry_task.h"
#include "runtime_channels.h"
#include "sensors/barometer_task.h"
#include "sensors/imu_task.h"
#include "sensors/power_sense_task.h"

class FlightControllerApp {
 public:
  bool setup(uint32_t nowMs);
  void loop(uint32_t nowMs);

 private:
  [[noreturn]] void failSafe(const char* reason);

  RuntimeChannels channels_;
  RuntimeStats stats_;
  BenchActuatorOutput actuatorOutput_;
  Bno085Hal imuHal_;
  Bmp388Hal barometerHal_;
  BatteryVoltageHal batteryHal_;
  ImuTask imuTask_{imuHal_, channels_, stats_};
  BarometerTask barometerTask_{barometerHal_, channels_, stats_};
  PowerSenseTask powerSenseTask_{batteryHal_, channels_, stats_};
  std::optional<FsmTask> fsmTask_;
  ActuatorTask actuatorTask_{channels_, actuatorOutput_};
  FlightLogTask flightLogTask_{channels_};
  TelemetryTask telemetryTask_{channels_};
  CommandTask commandTask_{channels_};
};
