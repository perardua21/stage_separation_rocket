#pragma once

#include <optional>

#include "hal/battery_voltage_hal.h"
#include "hal/bmp388_hal.h"
#include "hal/bno085_hal.h"
#include "missions/flight/fsm_task.h"
#include "missions/logging/flight_log_task.h"
#include "missions/system/actuator_task.h"
#include "missions/system/buzzer_task.h"
#include "missions/system/command_task.h"
#include "missions/telemetry/telemetry_task.h"
#include "runtime_channels.h"
#include "sensors/barometer_task.h"
#include "sensors/imu_task.h"
#include "sensors/power_sense_task.h"
#include "sensors/stage_link_task.h"
#include "vehicle_role_config.h"

#if defined(NURA_BENCH_SAFE)
#include "hal/bench_actuator_output.h"
using SelectedActuatorOutput = BenchActuatorOutput;
#else
#include "hal/flight_actuator_output.h"
using SelectedActuatorOutput = FlightActuatorOutput;
#endif

class FlightControllerApp {
 public:
  bool setup(uint32_t nowMs);
  void loop(uint32_t nowMs);

 private:
  [[noreturn]] void failSafe(const char* reason);

  RuntimeChannels channels_;
  RuntimeStats stats_;
  SelectedActuatorOutput actuatorOutput_;
  Bno085Hal imuHal_;
  Bmp388Hal barometerHal_;
  BatteryVoltageHal batteryHal_;
  ImuTask imuTask_{imuHal_, channels_, stats_};
  BarometerTask barometerTask_{barometerHal_, channels_, stats_};
  PowerSenseTask powerSenseTask_{batteryHal_, channels_, stats_};
  FsmTask fsmTask_{kBuildRole, channels_, stats_};
  StageLinkTask stageLinkTask_{channels_, stats_};
  ActuatorTask actuatorTask_{channels_, actuatorOutput_};
  FlightLogTask flightLogTask_{channels_};
  TelemetryTask telemetryTask_{kBuildRole, channels_, stats_};
  CommandTask commandTask_{channels_};
  BuzzerTask buzzerTask_{stats_};
};
