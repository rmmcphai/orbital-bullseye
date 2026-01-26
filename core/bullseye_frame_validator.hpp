// core/bullseye_frame_validator.hpp
#pragma once

/**
 * @file bullseye_frame_validator.hpp
 * @brief Deterministic validator for an adopted Bullseye-compatible RIC frame (FR-1b).
 *
 * This module is pure logic:
 * - No logging (caller decides policy).
 * - No allocations.
 * - Deterministic comparisons using explicit tolerances.
 *
 * DCM convention:
 * - Matrices use `C_from_a_to_b` semantics.
 * - Adopted frames provide `C_from_ric_to_inertial`:
 *     x_i = C_from_ric_to_inertial * x_ric
 */

#include "core/types.hpp"

namespace bullseye_pred {

struct FrameValidationTolerances final {
  // Centering check: ||origin_i - chief.r_i|| <= abs + rel*||chief.r_i||
  double center_abs_m{1e-6};
  double center_rel{1e-12};

  // Orthonormality check uses max-abs element of (C*C^T - I).
  double ortho_max_abs{1e-12};

  // Handedness check: |det(C) - 1| <= det_one_abs
  double det_one_abs{1e-12};
};

enum class FrameValidationReason : std::uint8_t {
  kOk = 0,
  kChiefNotOk,
  kFrameNotOk,
  kTimeMismatch,
  kBadDeclaration,
  kCenteringMismatch,
  kNotOrthonormal,
  kNotRightHanded,
  kOmegaBadDeclaration,
  kNonFinite,
};

struct FrameValidationResult final {
  ProviderStatus status{};
  FrameValidationReason reason{FrameValidationReason::kOk};
};

/**
 * @brief Validate that an adopted frame is Bullseye RIC-compatible at tick time t0.
 *
 * Checks (FR-1b / FR-14a):
 * - chief.status OK
 * - frame.status OK
 * - time_tag exact match to t0
 * - declaration: frame_kind == kBullseyeRIC and axis_order == kRIC
 * - centering: origin_i equals chief.r_i within tolerance
 * - DCM orthonormality: max|C*C^T - I| <= ortho_max_abs
 * - right-handedness: |det(C) - 1| <= det_one_abs
 * - ω declaration: if has_omega, omega_coords must be kOmegaRIC and finite
 */
[[nodiscard]] FrameValidationResult validate_adopted_bullseye_ric_frame(
    double t0,
    const ChiefState& chief,
    const AdoptedRicFrame& frame,
    const FrameValidationTolerances& tol) noexcept;

}  // namespace bullseye_pred
