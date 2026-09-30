#ifndef LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_TIMESTAMP_H_
#define LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_TIMESTAMP_H_

#include <cmath>
#include <cstdint>
#include <limits>

namespace livo_atlas {

// Sensor time in nanoseconds from its sensor-defined epoch. The epoch is not
// assumed to be Unix time. Valid sensor timestamps are non-negative, and zero
// is a valid timestamp rather than a missing-value marker.
using Timestamp = std::int64_t;
constexpr Timestamp kNanosecondsPerSecond = 1'000'000'000;

// Converts non-negative seconds to the nearest nanosecond, with half-nanosecond
// ties rounded toward positive infinity. Rounding uses the actual double input
// value, which may already have lost precision. On failure, *timestamp is
// unchanged.
inline bool TimestampFromSeconds(double seconds, Timestamp* timestamp) {
  if (timestamp == nullptr || !std::isfinite(seconds) || seconds < 0.0) {
    return false;
  }

  const long double nanoseconds =
      static_cast<long double>(seconds) * kNanosecondsPerSecond;
  if (!std::isfinite(nanoseconds)) {
    return false;
  }

  // std::round avoids adding 0.5 at large magnitudes where that addition could
  // itself round on platforms whose long double has double precision.
  const long double rounded_nanoseconds = std::round(nanoseconds);
  // 2^63 is exactly representable even when long double has double precision.
  // Using it as an exclusive bound avoids converting INT64_MAX to a floating
  // type where it could round up to 2^63.
  const long double kTimestampUpperExclusive = std::ldexp(1.0L, 63);
  if (rounded_nanoseconds >= kTimestampUpperExclusive) {
    return false;
  }

  *timestamp = static_cast<Timestamp>(rounded_nanoseconds);
  return true;
}

// Converts integer seconds and nanoseconds exactly. Nanoseconds must be in
// [0, 1e9), and the result must fit Timestamp. On failure, *timestamp is
// unchanged.
inline bool TimestampFromSecondsAndNanoseconds(
    std::int64_t seconds, std::uint32_t nanoseconds, Timestamp* timestamp) {
  if (timestamp == nullptr || seconds < 0 ||
      nanoseconds >= kNanosecondsPerSecond) {
    return false;
  }

  const Timestamp max_timestamp = std::numeric_limits<Timestamp>::max();
  if (seconds > max_timestamp / kNanosecondsPerSecond) {
    return false;
  }

  const Timestamp seconds_in_nanoseconds = seconds * kNanosecondsPerSecond;
  if (nanoseconds > max_timestamp - seconds_in_nanoseconds) {
    return false;
  }

  *timestamp = seconds_in_nanoseconds + nanoseconds;
  return true;
}

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_INCLUDE_LIVO_ATLAS_CORE_TIMESTAMP_H_
