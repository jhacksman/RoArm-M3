#pragma once
#include <stdint.h>

namespace FeedbackSafety {
constexpr uint32_t kMaxAgeMs = 100;
constexpr uint32_t kMaxSampleMs = 100;

// Ordered comparisons reject NaN and infinities as well as out-of-range supply.
constexpr bool voltageValid(float volts, float minimum, float maximum) {
  return volts > minimum && volts < maximum;
}

constexpr bool positionValid(int position) {
  return position >= 0 && position <= 4095;
}

constexpr bool canStream(bool complete, bool supplyOK, uint32_t now,
                         uint32_t sampleTime, uint32_t sampleDuration) {
  return complete && supplyOK && sampleDuration <= kMaxSampleMs &&
         static_cast<uint32_t>(now - sampleTime) <= kMaxAgeMs;
}
}  // namespace FeedbackSafety
