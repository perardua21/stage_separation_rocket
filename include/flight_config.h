#pragma once

#include <stdint.h>

namespace FlightConfig {

constexpr float kLaunchThresholdG = 2.0f;
constexpr uint8_t kLaunchSamples = 5;
constexpr float kBurnoutThresholdG = 2.0f;
constexpr uint8_t kBurnoutSamples = 5;
constexpr float kIgnitionConfirmG = 3.0f;
constexpr uint8_t kIgnitionSamples = 5;
constexpr float kApogeeDropM = 3.0f;
constexpr uint8_t kApogeeSamples = 5;
constexpr float kLandingRangeM = 2.0f;

constexpr uint32_t kBoosterBurnoutEarliestMs = 500;
constexpr uint32_t kBoosterBurnoutTimeoutMs = 1300;
constexpr uint32_t kSeparationTimeoutFromLaunchMs = 2500;
constexpr uint32_t kBarometerLockoutMs = 500;
constexpr uint32_t kIgnitionWindowOpenMs = 500;
constexpr uint32_t kIgnitionWindowCloseMs = 1500;
constexpr float kIgnitionMaxTiltDeg = 20.0f;
constexpr float kIgnitionMinVelocityMps = 30.0f;
constexpr float kIgnitionMinAltitudeM = 45.0f;
constexpr uint32_t kIgnitionConfirmTimeoutMs = 1000;
constexpr uint32_t kSustainerBurnoutEarliestMs = 1000;
constexpr uint32_t kSustainerBurnoutTimeoutMs = 2200;
constexpr uint32_t kBoosterApogeeEarliestFromLaunchMs = 3500;
constexpr uint32_t kBoosterApogeeTimeoutFromLaunchMs = 6200;
constexpr uint32_t kSustainerApogeeEarliestFromIgnitionMs = 6000;
constexpr uint32_t kSustainerApogeeTimeoutFromIgnitionMs = 12000;
constexpr uint32_t kPyroPulseMs = 1000;
constexpr uint32_t kLandingWindowMs = 5000;
constexpr uint32_t kBoosterLandingTimeoutMs = 40000;
constexpr uint32_t kSustainerLandingTimeoutMs = 140000;

}  // namespace FlightConfig

