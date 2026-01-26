// core/provider_cartesian.hpp
#pragma once

/**
 * @file provider_cartesian.hpp
 * @brief Chief state provider backed by user-supplied Cartesian samples.
 *
 * Logging (sim-logger) policy implemented:
 * - Constructor: INFO (mode + frame_id)
 * - add_sample/set_current/clear_samples: DEBUG (configuration-time chatter)
 * - get(t0): no log on success
 *   - ERROR on invalid configuration (frame_id == nullptr), logged once
 *   - WARN on missing time (kTimeMissing), rate-limited by tick time
 *
 * FR-14: Exact-time only. No interpolation or nearest-time behavior.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/chief_state_provider.hpp"
#include "core/types.hpp"

namespace bullseye_pred {

class CartesianChiefProvider final : public IChiefStateProvider {
 public:
  enum class Mode : std::uint8_t { kCurrent = 0, kTimeSeries };

  /**
   * @param inertial_frame_id Must outlive this provider (string literal or config storage).
   * @param mode Current vs TimeSeries.
   * @param warn_period_sec Rate-limit period for repeated kTimeMissing warnings (based on t0).
   */
  explicit CartesianChiefProvider(const char* inertial_frame_id,
                                 Mode mode = Mode::kCurrent,
                                 double warn_period_sec = 1.0);

  /** Set the current sample (Mode::kCurrent). Caller must set t == t0 for FR-14 exact match. */
  void set_current(double t, const Vec3& r_i, const Vec3& v_i) noexcept;

  /**
   * Add a time-tagged sample (Mode::kTimeSeries).
   * Samples do not need to be added in sorted order; provider sorts deterministically.
   */
  void add_sample(double t, const Vec3& r_i, const Vec3& v_i);

  /** Clear stored samples (Mode::kTimeSeries). */
  void clear_samples() noexcept;

  [[nodiscard]] std::size_t sample_count() const noexcept { return samples_.size(); }

  /** 
    * Propagates ChiefState to time t0.
    */
  [[nodiscard]] ChiefState get(double t0) noexcept override;

 private:
  struct Sample final {
    double t{0.0};
    Vec3 r_i{};
    Vec3 v_i{};
  };

  void ensure_sorted_() noexcept;
  bool should_warn_time_missing_(double t0) noexcept;
  void log_invalid_input_once_() noexcept;

  const char* inertial_frame_id_{nullptr};
  Mode mode_{Mode::kCurrent};

  // Rate limiting for repeated WARN in get(t0).
  double warn_period_sec_{1.0};
  double last_warn_t0_{-1.0e300};

  // "Log once" latch for configuration errors.
  bool invalid_logged_{false};

  // Current mode storage
  Sample current_{};

  // Time series storage (sorted by t, deterministically)
  std::vector<Sample> samples_{};
  bool sorted_{true};
};

}  // namespace bullseye_pred
