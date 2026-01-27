// core/provider_twobody.hpp
#pragma once

/**
 * @file provider_twobody.hpp
 * @brief Deterministic two-body chief state provider (universal-variable f-g propagation).
 *
 * Intent
 * - Provide a chief source that can always return a state for any requested `t0`
 *   (satisfies FR-14 exact-time by construction).
 *
 * Determinism policy
 * - Fixed iteration count for Kepler solve (no early exit).
 * - No allocations in `get()`.
 *
 * Logging policy (sim-logger)
 * - INFO on init (frame_id, mu, t_epoch)
 * - ERROR on invalid configuration (logged once)
 * - No logging on successful `get()`
 */

#include "core/chief_state_provider.hpp"
#include "core/types.hpp"

namespace bullseye_pred
{

class TwoBodyChiefProvider final : public IChiefStateProvider
{
  public:
    /**
     * @param inertial_frame_id Must outlive this provider (string literal or config storage).
     * @param mu Gravitational parameter (m^3/s^2). Must be > 0.
     * @param t_epoch Epoch time associated with (r_epoch_i, v_epoch_i).
     * @param r_epoch_i Chief position at epoch in inertial coordinates (m).
     * @param v_epoch_i Chief velocity at epoch in inertial coordinates (m/s).
     */
    TwoBodyChiefProvider(const char* inertial_frame_id, double mu, double t_epoch,
                         const Vec3& r_epoch_i, const Vec3& v_epoch_i);

    [[nodiscard]] ChiefState get(double t0) noexcept override;

  private:
    void log_invalid_config_once_(const char* why) noexcept;

    const char* inertial_frame_id_{nullptr};
    double mu_{0.0};
    double t_epoch_{0.0};
    Vec3 r0_{};
    Vec3 v0_{};

    bool invalid_logged_{false};

    // Fixed iteration count for universal-variable solve (deterministic control flow).
    static constexpr int kKeplerIters = 12;
};

} // namespace bullseye_pred
