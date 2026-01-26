// core/frame_provider_cartesian.hpp
#pragma once

/**
 * @file frame_provider_cartesian.hpp
 * @brief Adopted Bullseye-compatible RIC frame provider backed by user-supplied samples.
 *
 * Policy
 * - **FR-14a exact-time only**: no interpolation, no nearest-time.
 * - Provider is a pass-through for pose/ω declarations; correctness is enforced by the validator.
 *
 * Logging (sim-logger)
 * - INFO on init (frame_source_id, warn_period_sec)
 * - DEBUG on set_current/add_sample/clear_samples (configuration-time chatter)
 * - get(t0): no log on success
 *   - ERROR on invalid configuration (logged once)
 *   - WARN on missing time, rate-limited by tick time
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/bullseye_frame_provider.hpp"
#include "core/types.hpp"

namespace bullseye_pred {

class CartesianBullseyeFrameProvider final : public IBullseyeFrameProvider {
    public:
    enum class Mode : std::uint8_t { kCurrent = 0, kTimeSeries };

    /**
    * @param frame_source_id Optional provenance string (must outlive this provider).
    * @param mode Current vs TimeSeries.
    * @param warn_period_sec Rate-limit period for repeated kTimeMissing warnings (based on t0).
    */
    explicit CartesianBullseyeFrameProvider(const char* frame_source_id,
                                         Mode mode = Mode::kCurrent,
                                         double warn_period_sec = 1.0);

    /**
    * Set the current adopted frame sample (Mode::kCurrent).
    *
    * @param t  Time tag (caller must set t == t0 for FR-14a exact match).
    * @param origin_i  Frame origin expressed in inertial coordinates.
    * @param C_from_ric_to_inertial  DCM with required semantic:
    *        x_i = C_from_ric_to_inertial * x_ric
    */
    void set_current(double t, const Vec3& origin_i, const Mat3& C_from_ric_to_inertial) noexcept;

    /**
    * Optionally set ω_RIC for the current sample.
    * Provider enforces that ω coordinates are explicitly declared as RIC when present.
    */
    void set_current_omega_ric(const Vec3& omega_ric) noexcept;

    /** Clear ω for the current sample. */
    void clear_current_omega() noexcept;

    /**
    * Add a time-tagged adopted frame sample (Mode::kTimeSeries).
    * Samples do not need to be added in sorted order; provider sorts deterministically.
    */
    void add_sample(double t, const Vec3& origin_i, const Mat3& C_from_ric_to_inertial);

    /** Optionally set ω_RIC for the most recently added sample (Mode::kTimeSeries). */
    void set_last_sample_omega_ric(const Vec3& omega_ric);

    void clear_samples() noexcept;
    [[nodiscard]] std::size_t sample_count() const noexcept { return samples_.size(); }

    [[nodiscard]] AdoptedRicFrame get(double t0) noexcept override;

    private:
    struct Sample final {
        double t{0.0};
        Vec3 origin_i{};
        Mat3 C_from_ric_to_inertial{Mat3::identity()};
        bool has_omega{false};
        Vec3 omega_ric{};
    };
    void ensure_sorted_() noexcept;
    bool should_warn_time_missing_(double t0) noexcept;
    void log_invalid_input_once_(const char* why) noexcept;

    const char* frame_source_id_{nullptr};
    Mode mode_{Mode::kCurrent};

    double warn_period_sec_{1.0};
    double last_warn_t0_{-1.0e300};
    bool invalid_logged_{false};

    Sample current_{};
    std::vector<Sample> samples_{};
    bool sorted_{true};
};

}  // namespace bullseye_pred
