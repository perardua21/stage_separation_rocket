#include "missions/logging/flight_log_task.h"

#include "board_pins.h"

namespace {
void writeCsvHeader(File& file) {
  file.println("ms,role,state,reason,accel_g,alt_m,vel_mps,tilt_deg,"
               "integrated_vel_mps,battery_v,max_alt_m,log_drop,tlm_drop,"
               "sep,heat_arm");
}
}  // namespace

void writeCsvRow(Print& out, const FlightSnapshot& frame) {
  out.printf("%lu,%s,%s,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lu,%lu,%u,%u\n",
             static_cast<unsigned long>(frame.timestampMs),
             frame.role == VehicleRole::Booster ? "BOOSTER" : "SUSTAINER",
             flightStateName(frame.state), reasonCodeName(frame.reason),
             frame.sensor.accelerationG, frame.sensor.altitudeM,
             frame.sensor.verticalVelocityMps, frame.sensor.tiltDeg,
             frame.sensor.integratedVelocityMps, frame.sensor.batteryV,
             frame.maxAltitudeM,
             static_cast<unsigned long>(frame.droppedLogFrames),
             static_cast<unsigned long>(frame.droppedTelemetryFrames),
             frame.separationSeen ? 1u : 0u, frame.heaterArmed ? 1u : 0u);
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
  Serial.printf("[SD] %s\n", sdReady ? "mounted" : "NOT MOUNTED");
  File file;
  if (sdReady) {
    file = SD.open("/flight.csv", FILE_APPEND);
    if (file && file.size() == 0) writeCsvHeader(file);
    if (!file) Serial.println("[SD] /flight.csv open FAILED");
  }

  FlightSnapshot frame;
  uint32_t rowsSinceFlush = 0;
  for (;;) {
    if (xQueueReceive(channels_.log, &frame, portMAX_DELAY) != pdTRUE) continue;
    if (file) {
      writeCsvRow(file, frame);
      // 화약이 터지는 구간에서는 전원이 끊길 수 있으므로 매 줄 flush 한다.
      if (frame.state == FlightState::Apogee || ++rowsSinceFlush >= 20) {
        file.flush();
        rowsSinceFlush = 0;
      }
    }
  }
}
