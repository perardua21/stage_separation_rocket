#pragma once

#include <cmath>

namespace tilt_log {
struct Vec { float x, y, z; };
struct Quat { float w, x, y, z; };

inline float dot(Vec a, Vec b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec cross(Vec a, Vec b) {
  return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
inline bool normalize(Quat& q) {
  const float n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (!std::isfinite(n) || n < 0.5f || n > 1.5f) return false;
  q = {q.w/n, q.x/n, q.y/n, q.z/n};
  return true;
}
inline Vec rotate(Quat q, Vec v) {
  const Vec u{q.x, q.y, q.z};
  const Vec c = cross(u, v);
  const Vec d = cross(u, c);
  return {v.x + 2*(q.w*c.x+d.x), v.y + 2*(q.w*c.y+d.y),
          v.z + 2*(q.w*c.z+d.z)};
}
inline float mountAngle(Quat q, Vec up) {
  const Vec mount = rotate(q, {0, -1, 0});
  const Vec c = cross(mount, up);
  return std::atan2(std::sqrt(dot(c,c)), dot(mount,up)) * 57.2957795f;
}
}  // namespace tilt_log
