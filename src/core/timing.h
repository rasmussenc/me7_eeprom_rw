#pragma once

// Portable millisecond timing helpers.
//
// The read/write timeouts are computed in milliseconds against the configured
// baud rate; the per-byte poll loop sleeps in 1 ms increments. Uses
// steady_clock so wire timing is consistent across hosts, matching the
// original tool's per-byte/response deadline arithmetic.

#include <chrono>
#include <cmath>
#include <thread>

namespace me7 {

// Milliseconds elapsed since an arbitrary epoch.
inline double nowMs() {
  using namespace std::chrono;
  return duration_cast<duration<double, std::milli>>(
             steady_clock::now().time_since_epoch())
      .count();
}

inline void sleepMs(int ms) {
  if (ms <= 0) return;
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Seconds-to-milliseconds rounding helper for the (10.0/baud)*1000 deadline math.
template <typename T>
inline int roundMs(double v) {
  return static_cast<int>(std::round(v));
}

}  // namespace me7
