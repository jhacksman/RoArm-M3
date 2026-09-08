#include "../src/RoArm-M3_safe/feedback_safety.h"
using namespace FeedbackSafety;
static_assert(!voltageValid(0.0f, 6.0f, 12.9f), "USB-only must block");
static_assert(!voltageValid(0.02f, 6.0f, 12.9f), "sensor noise must block");
static_assert(!voltageValid(6.0f, 6.0f, 12.9f), "minimum boundary must block");
static_assert(voltageValid(12.0f, 6.0f, 12.9f), "normal supply must pass");
static_assert(!voltageValid(12.9f, 6.0f, 12.9f), "maximum boundary must block");
static_assert(!voltageValid(__builtin_nanf(""), 6.0f, 12.9f), "NaN must block");
static_assert(!voltageValid(__builtin_inff(), 6.0f, 12.9f), "infinity must block");
static_assert(!positionValid(-1) && !positionValid(4096), "bad encoder range");
static_assert(positionValid(0) && positionValid(4095), "valid encoder endpoints");
static_assert(!canStream(false, true, 10, 10, 5), "failed joint must block");
static_assert(!canStream(true, false, 10, 10, 5), "bad supply must block");
static_assert(!canStream(false, false, 0, 0, 0), "initial state must block");
static_assert(canStream(true, true, 20, 10, 5), "fresh complete sample must pass");
static_assert(!canStream(true, true, 111, 10, 5), "stale sample must block");
static_assert(!canStream(true, true, 20, 20, 101), "slow partial snapshot must block");
static_assert(canStream(true, true, 10, UINT32_MAX - 10, 5), "millis rollover");
static_assert(!canStream(true, true, 1000, UINT32_MAX - 10, 5), "stale rollover");
