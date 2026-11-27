#include <Arduino.h>
#include <cmath>

#include "diagnostics/mount_tilt.h"
#include "diagnostics/tilt_log_math.h"
#include "hal/bno085_hal.h"

#if !defined(NURA_AXIS_TILT_DIAG)
#error "Build this diagnostic only with bno085_axis_tilt."
#endif

namespace {
constexpr uint32_t kPrintPeriodMs = 500;
constexpr uint32_t kStaleMs = 250;
constexpr float kRadToDeg = 57.2957795f;
Bno085Hal imu;
bool ready = false;
uint32_t lastGoodMs = 0, lastPrintMs = 0;
uint32_t lastLogMs = 0, orientationRxMs = 0, observedResetCount = 0;
bool haveOrientation = false, haveUp = false, referenceLost = false;
tilt_log::Quat orientation{};
tilt_log::Vec up{};

bool orientationFresh(uint32_t now) {
  return haveOrientation && now - orientationRxMs <= 150;
}

void printMotionLog(uint32_t now) {
  const auto ms = static_cast<unsigned long>(now);
  if (referenceLost) {
    Serial.printf("[TILT_LOG] %lu,NA,SENSOR_RESET\n", ms);
  } else if (!orientationFresh(now)) {
    Serial.printf("[TILT_LOG] %lu,NA,NO_FRESH_ORIENTATION\n", ms);
  } else if (!haveUp) {
    Serial.printf("[TILT_LOG] %lu,NA,WAIT_STILL\n", ms);
  } else {
    Serial.printf("[TILT_LOG] %lu,%.2f,ESTIMATE,%lu\n", ms,
                  tilt_log::mountAngle(orientation, up),
                  static_cast<unsigned long>(now - orientationRxMs));
  }
}

struct Window {
  float sum[3]{};
  float lo[3]{};
  float hi[3]{};
  unsigned count = 0;

  void add(float x, float y, float z) {
    const float v[]{x, y, z};
    for (int i = 0; i < 3; ++i) {
      if (count == 0 || v[i] < lo[i]) lo[i] = v[i];
      if (count == 0 || v[i] > hi[i]) hi[i] = v[i];
      sum[i] += v[i];
    }
    ++count;
  }
} window;

float inclination(float along, float across1, float across2) {
  return std::atan2(std::hypot(across1, across2), along) * kRadToDeg;
}

void printWindow(uint32_t now) {
  if (!ready) {
    Serial.println("[AXIS] BNO085 initialization failed; reset to retry");
  } else if (!window.count || now - lastGoodMs > kStaleMs) {
    Serial.println("[AXIS] no recent acceleration samples");
  } else {
    const float x = window.sum[0] / window.count;
    const float y = window.sum[1] / window.count;
    const float z = window.sum[2] / window.count;
    const float norm = std::hypot(std::hypot(x, y), z);
    if (norm > 0.01f) {
      // The accelerometer indicates up while supported and still. These are
      // angles of the positive sensor axes from up, not roll/pitch/yaw.
      const bool steady = window.count >= 5 && norm >= 8.5f && norm <= 11.0f &&
                          window.hi[0] - window.lo[0] < 0.7f &&
                          window.hi[1] - window.lo[1] < 0.7f &&
                          window.hi[2] - window.lo[2] < 0.7f;
      if (steady && !haveUp && !referenceLost && orientationFresh(now)) {
        up = tilt_log::rotate(orientation, {x/norm, y/norm, z/norm});
        haveUp = true;
        Serial.println("[TILT_LOG] vertical reference captured");
      }
      Serial.printf(
          "[AXIS] accel_xyz[m/s2]=(%+.2f,%+.2f,%+.2f) norm=%.2f "
          "from_up_xyz[deg]=(%.1f,%.1f,%.1f) mount_tilt_deg=%.1f n=%u %s\n",
          x, y, z, norm, inclination(x, y, z), inclination(y, x, z),
          inclination(z, x, y), mount_tilt::fromUpDeg(x, y, z), window.count,
          steady ? "STEADY" : "HOLD STILL");
    } else {
      Serial.println("[AXIS] acceleration magnitude too small for inclination");
    }
  }
  window = Window{};
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[AXIS] BNO085: +X/+Y/+Z angles from vertical up");
  Serial.println("[AXIS] 0=up, 90=horizontal, 180=down; hold still to measure");
  Serial.println("[TILT_LOG] ms,mount_tilt_deg,status,q_rx_age_ms (when available)");
  Serial.println("[TILT_LOG] Hold still at startup; after sensor reset restart while still.");
  ready = imu.begin();
  Serial.printf("[AXIS] initialization %s\n", ready ? "PASS" : "FAIL");
}

void loop() {
  Bno085Reading reading;
  const bool received = ready && imu.read(reading, millis());
  const uint32_t now = millis();
  if (imu.resetCountForLog() != observedResetCount) {
    observedResetCount = imu.resetCountForLog();
    referenceLost = referenceLost || haveUp;
    haveUp = false;
    haveOrientation = false;
    window = Window{};
  }
  if (received && reading.orientationValid) {
    tilt_log::Quat q{reading.quaternionW, reading.quaternionX,
                     reading.quaternionY, reading.quaternionZ};
    if (tilt_log::normalize(q)) {
      orientation = q;
      orientationRxMs = reading.orientationRxMs;
      haveOrientation = true;
    } else {
      haveOrientation = false;
    }
  }
  if (received &&
      std::isfinite(reading.accelXMps2) && std::isfinite(reading.accelYMps2) &&
      std::isfinite(reading.accelZMps2)) {
    lastGoodMs = now;
    window.add(reading.accelXMps2, reading.accelYMps2, reading.accelZMps2);
  }
  if (now - lastPrintMs >= kPrintPeriodMs) {
    lastPrintMs = now;
    printWindow(now);
  }
  if (now - lastLogMs >= 50) {
    lastLogMs = now;
    printMotionLog(now);
  }
  delay(5);
}
