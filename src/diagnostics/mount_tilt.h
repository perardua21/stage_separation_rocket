#pragma once

#include <cmath>

namespace mount_tilt {
// Stationary diagnostic only. The installed board's upward axis is sensor -Y.
inline float fromUpDeg(float x, float y, float z) {
  return std::atan2(std::hypot(x, z), -y) * 57.2957795f;
}

inline bool gravityLike(float x, float y, float z) {
  const float norm = std::hypot(std::hypot(x, y), z);
  return norm >= 8.5f && norm <= 11.0f;
}

struct Window {
  float sum[3]{}, low[3]{}, high[3]{};
  unsigned count = 0;

  void add(float x, float y, float z) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return;
    const float sample[]{x, y, z};
    for (int i = 0; i < 3; ++i) {
      if (!count || sample[i] < low[i]) low[i] = sample[i];
      if (!count || sample[i] > high[i]) high[i] = sample[i];
      sum[i] += sample[i];
    }
    ++count;
  }

  bool estimate(float& angleDeg) const {
    if (count < 5) return false;
    for (int i = 0; i < 3; ++i)
      if (high[i] - low[i] >= 0.7f) return false;
    const float x = sum[0] / count, y = sum[1] / count, z = sum[2] / count;
    if (!gravityLike(x, y, z)) return false;
    angleDeg = fromUpDeg(x, y, z);
    return true;
  }
};
}  // namespace mount_tilt
