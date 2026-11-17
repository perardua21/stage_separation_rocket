#include "missions/logging/flight_log_task.h"

#include "board_pins.h"

namespace {
void writeCsvHeader(File& file) {
  file.println("ms,role,state,reason,accel_g,alt_m,vel_mps,tilt_deg,"
               "integrated_vel_mps,battery_v,max_alt_m,log_drop,tlm_drop");
}
}  // namespace

void writeCsvRow(Print& out, const FlightSnapshot& frame) {
  out.printf("%lu,%s,%s,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lu,%lu\n",
             static_cast<unsigned long>(frame.timestampMs),
             frame.role == VehicleRole::Booster ? "BOOSTER" : "SUSTAINER",
             flightStateName(frame.state), reasonCodeName(frame.reason),
             frame.sensor.accelerationG, frame.sensor.altitudeM,
             frame.sensor.verticalVelocityMps, frame.sensor.tiltDeg,
             frame.sensor.integratedVelocityMps, frame.sensor.batteryV,
             frame.maxAltitudeM,
             static_cast<unsigned long>(frame.droppedLogFrames),
             static_cast<unsigned long>(frame.droppedTelemetryFrames));
}

bool FlightLogTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "flight_log", 6144, this, 2, nullptr,
                                 core) == pdPASS;
}

void FlightLogTask::entry(void* context) {
  static_cast<FlightLogTask*>(context)->run();
}

void FlightLogTask::run() {
  sdSpi_.begin(BoardPins::kSdSck, BoardPins::kSdMiso, BoardPins::kSdMosi,
               BoardPins::kSdCs);
  const bool sdReady = SD.begin(BoardPins::kSdCs, sdSpi_, 16000000);
  File file;
  if (sdReady) {
    file = SD.open("/flight.csv", FILE_APPEND);
    if (file && file.size() == 0) writeCsvHeader(file);
  }

  FlightSnapshot frame;
  uint32_t rowsSinceFlush = 0;
  for (;;) {
    if (xQueueReceive(channels_.log, &frame, portMAX_DELAY) != pdTRUE) continue;
    if (file) {
      writeCsvRow(file, frame);
      if (++rowsSinceFlush >= 20) {
        file.flush();
        rowsSinceFlush = 0;
      }
    }
  }
}
