#include "sensors/imu_task.h"

#include <Arduino.h>
#include <math.h>

#if defined(NURA_BENCH_SAFE)
#include "diagnostics/mount_tilt.h"
#include "diagnostics/tilt_log_math.h"
#endif

namespace {
constexpr uint32_t kPeriodMs = 10;
constexpr uint32_t kBaselineDurationMs = 10000;
constexpr float kGravityMps2 = 9.80665f;

struct Vec3 { float x; float y; float z; };
struct Quaternion { float w; float x; float y; float z; };

Vec3 rotate(const Quaternion& q, const Vec3& v) {
  const Vec3 t{2.0f * (q.y * v.z - q.z * v.y),
               2.0f * (q.z * v.x - q.x * v.z),
               2.0f * (q.x * v.y - q.y * v.x)};
  return {v.x + q.w * t.x + (q.y * t.z - q.z * t.y),
          v.y + q.w * t.y + (q.z * t.x - q.x * t.z),
          v.z + q.w * t.z + (q.x * t.y - q.y * t.x)};
}

float dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
float norm(const Vec3& v) { return sqrtf(dot(v, v)); }
Vec3 normalized(const Vec3& v) {
  const float length = norm(v);
  return length < 0.001f ? Vec3{1.0f, 0.0f, 0.0f}
                         : Vec3{v.x / length, v.y / length, v.z / length};
}
float clampUnit(float value) {
  return value < -1.0f ? -1.0f : (value > 1.0f ? 1.0f : value);
}
}  // namespace

bool ImuTask::start(BaseType_t core) {
  return xTaskCreatePinnedToCore(entry, "imu", 6144, this, 4, nullptr, core) ==
         pdPASS;
}

void ImuTask::entry(void* context) { static_cast<ImuTask*>(context)->run(); }

void ImuTask::run() {
  const bool ready = imu_.begin();
  Serial.printf("[IMU] BNO085 initialization %s\n", ready ? "PASS" : "FAIL");
  const uint32_t baselineStartMs = millis();
  uint32_t lastSampleMs = baselineStartMs;
  Quaternion orientation{1.0f, 0.0f, 0.0f, 0.0f};
  Vec3 launchVerticalWorld{1.0f, 0.0f, 0.0f};
  bool launchReferenceCaptured = false;
  float integratedVelocityMps = 0.0f;
#if defined(NURA_BENCH_SAFE)
  mount_tilt::Window tiltWindow;
  uint32_t lastTiltSampleMs = baselineStartMs;
  uint32_t lastTiltWindowMs = baselineStartMs;
  uint32_t tiltOrientationRxMs = 0;
  uint32_t tiltEpoch = 0;
  bool tiltReferenceReady = false;
  bool tiltReferenceLost = false;
  bool tiltOrientationValid = false;
  tilt_log::Quat tiltOrientation{};
  tilt_log::Vec tiltUp{};
  // Negative is outside [0, 180] and survives the existing CSV/E22 formats.
  constexpr float kBenchTiltUnavailable = -1.0f;
#endif

  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const uint32_t nowMs = millis();
    Bno085Reading reading;
    const bool readOk = ready && imu_.read(reading, nowMs);
    if (readOk) {
      orientation = {reading.quaternionW, reading.quaternionX,
                     reading.quaternionY, reading.quaternionZ};
      if (reading.orientationValid && !launchReferenceCaptured &&
          nowMs - baselineStartMs >= kBaselineDurationMs) {
        launchVerticalWorld =
            normalized(rotate(orientation, {1.0f, 0.0f, 0.0f}));
        launchReferenceCaptured = true;
      }

      const float dtS = (reading.sampleMs - lastSampleMs) / 1000.0f;
      lastSampleMs = reading.sampleMs;
      if (launchReferenceCaptured && dtS > 0.0f && dtS < 0.1f) {
        const Vec3 linear{reading.linearAccelXMps2,
                          reading.linearAccelYMps2,
                          reading.linearAccelZMps2};
        integratedVelocityMps +=
            dot(rotate(orientation, linear), launchVerticalWorld) * dtS;
      }

      const Vec3 acceleration{reading.accelXMps2, reading.accelYMps2,
                              reading.accelZMps2};
#if defined(NURA_BENCH_SAFE)
      if (reading.orientationValid && reading.orientationEpoch != tiltEpoch) {
        tiltEpoch = reading.orientationEpoch;
        tiltReferenceLost = tiltReferenceLost || tiltReferenceReady;
        tiltReferenceReady = false;
        tiltWindow = {};
      }
      tiltOrientation = {reading.quaternionW, reading.quaternionX,
                         reading.quaternionY, reading.quaternionZ};
      tiltOrientationRxMs = reading.orientationRxMs;
      tiltOrientationValid = reading.orientationValid &&
                             tilt_log::normalize(tiltOrientation);
      tiltWindow.add(acceleration.x, acceleration.y, acceleration.z);
      lastTiltSampleMs = nowMs;
#else
      const Vec3 currentAxis =
          normalized(rotate(orientation, {1.0f, 0.0f, 0.0f}));
#endif
      SensorMessage message;
      message.source = SensorSource::Imu;
      message.timestampMs = reading.sampleMs;
      message.accelerationG = norm(acceleration) / kGravityMps2;
#if defined(NURA_BENCH_SAFE)
      const bool tiltFresh = tiltOrientationValid &&
                             millis() - tiltOrientationRxMs <= 150;
      message.tiltDeg = tiltReferenceReady && tiltFresh
                            ? tilt_log::mountAngle(tiltOrientation, tiltUp)
                            : kBenchTiltUnavailable;
#else
      message.tiltDeg = launchReferenceCaptured
                            ? acosf(clampUnit(dot(currentAxis,
                                                  launchVerticalWorld))) *
                                  180.0f / PI
                            : 0.0f;
#endif
      message.integratedVelocityMps = integratedVelocityMps;
      message.valid = reading.orientationValid && launchReferenceCaptured;
      if (xQueueSend(channels_.sensor, &message, 0) != pdTRUE) {
        ++stats_.droppedSensorSamples;
      }
    }
#if defined(NURA_BENCH_SAFE)
    if (nowMs - lastTiltWindowMs >= 1000) {
      lastTiltWindowMs = nowMs;
      float angleDeg = 0;
      if (nowMs - lastTiltSampleMs <= 250 && tiltWindow.estimate(angleDeg)) {
        if (!tiltReferenceReady && !tiltReferenceLost && tiltOrientationValid &&
            millis() - tiltOrientationRxMs <= 150) {
          tilt_log::Vec up{tiltWindow.sum[0], tiltWindow.sum[1], tiltWindow.sum[2]};
          const float length = sqrtf(tilt_log::dot(up, up));
          up = {up.x / length, up.y / length, up.z / length};
          tiltUp = tilt_log::rotate(tiltOrientation, up);
          tiltReferenceReady = true;
          Serial.println("[IMU] bench vertical reference captured");
        }
      }
      tiltWindow = {};
    }
#endif
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kPeriodMs));
  }
}
