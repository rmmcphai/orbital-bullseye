// core/chief_state_provider.hpp
#pragma once

/**
 * @file chief_state_provider.hpp
 * @brief Interface for retrieving the chief Cartesian state in the configured inertial frame.
 *
 * ## Contract highlights
 * - **FR-14 (Fail-Fast Exact-Time):** Providers must return a state explicitly tagged to the
 *   requested time `t0` or return a non-OK status (no "nearest time" behavior).
 * - **GCR-3 (Inertial Frame Contract):** Returned state must be expressed in a single configured
 *   inertial frame and identify that frame via `frame_id`.
 *
 * This header is dependency-free (no JEOD/Trick).
 */

#include "core/types.hpp"

namespace bullseye_pred {

class IChiefStateProvider {
 public:
  virtual ~IChiefStateProvider() = default;

  /**
   * @brief Get chief state for exactly the requested tick time t0.
   *
   * @param t0 Requested predictor tick time (seconds).
   * @return ChiefState with `status.ok()==true` iff a valid state for exactly t0 is available.
   *
   * Requirements:
   * - On success: `state.time_tag == t0` (exact match) and `state.frame_id != nullptr`.
   * - On failure: `status.code != kOk` and payload fields may be left unspecified.
   */
  [[nodiscard]] virtual ChiefState get(double t0) noexcept = 0;
};

}  // namespace bullseye_pred
