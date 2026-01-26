// core/bullseye_frame_provider.hpp
#pragma once

/**
 * @file bullseye_frame_provider.hpp
 * @brief Interface for retrieving an adopted Bullseye-compatible RIC frame at tick time t0.
 *
 * ## DCM naming convention
 * We use `C_from_a_to_b` meaning:
 *   x_b = C_from_a_to_b * x_a
 *
 * Canonical adopted-frame orientation for v1 is:
 *   x_i = C_from_ric_to_inertial * x_ric
 *
 * ## Contract highlights
 * - **FR-14a (Fail-Fast Exact-Time):** Providers must return a frame explicitly tagged to `t0`
 *   or return a non-OK status.
 * - **FR-1b (Adopted RIC compatibility):** The validator will check time tag, centering,
 *   orthonormality/handedness, axis declaration, and ω declaration.
 *
 * This header is dependency-free (no JEOD/Trick).
 */

#include "core/types.hpp"

namespace bullseye_pred {

class IBullseyeFrameProvider {
 public:
  virtual ~IBullseyeFrameProvider() = default;

  /**
   * @brief Get adopted RIC frame for exactly the requested tick time t0.
   *
   * @param t0 Requested predictor tick time (seconds).
   * @return AdoptedRicFrame with `status.ok()==true` iff a valid frame for exactly t0 is available.
   *
   * Requirements:
   * - On success: `frame.time_tag == t0` (exact match), `frame.frame_kind == kBullseyeRIC`,
   *   `frame.axis_order == kRIC`, and `C_from_ric_to_inertial` is right-handed & orthonormal
   *   within tolerances (checked by validator).
   * - If `has_omega==true`, then `omega_coords` must be explicitly declared (v1 expects kOmegaRIC).
   * - On failure: `status.code != kOk` and payload fields may be left unspecified.
   */
  [[nodiscard]] virtual AdoptedRicFrame get(double t0) noexcept = 0;
};

}  // namespace bullseye_pred
