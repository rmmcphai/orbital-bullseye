// core/bullseye_frame_math.hpp
#pragma once

/**
 * @file bullseye_frame_math.hpp
 * @brief Deterministic construction of a Bullseye RIC frame from a chief inertial state.
 *
 * The constructed Bullseye frame is the standard RIC/RSW/LVLH triad:
 * - R: radial (along position)
 * - C: cross-track (along angular momentum)
 * - I: in-track (C x R)
 *
 * DCM convention:
 * - Matrices follow `C_from_a_to_b` semantics.
 * - For constructed frames we provide:
 *     x_i = C_from_ric_to_inertial * x_ric
 */

#include "core/contracts.hpp"
#include "core/types.hpp"

namespace bullseye_pred
{

struct ConstructedRicFrame final
{
    double time_tag{0.0};
    Vec3 origin_i{}; // == chief.r_i
    Mat3 C_from_ric_to_inertial{Mat3::identity()};
    Vec3 omega_ric{}; // ω of RIC wrt inertial, expressed in RIC components
    bool has_omega{true};
    OmegaCoords omega_coords{OmegaCoords::kOmegaRIC};
    FrameKind frame_kind{FrameKind::kBullseyeRIC};
    AxisOrder axis_order{AxisOrder::kRIC};
    ProviderStatus status{};
};

/**
 * @brief Construct a Bullseye RIC frame from chief inertial state.
 *
 * Deterministic behavior:
 * - No allocations.
 * - No logging.
 * - Uses contract thresholds in contracts::Tol.
 *
 * Failure cases:
 * - chief.status not OK
 * - non-finite inputs
 * - degeneracy: |r| < kRmin, |v| < kVmin, or h_hat < kHhatMin
 */
[[nodiscard]] ConstructedRicFrame construct_ric_from_chief(const ChiefState& chief) noexcept;

} // namespace bullseye_pred
