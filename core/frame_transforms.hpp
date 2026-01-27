// core/frame_transforms.hpp
#pragma once

/**
 * @file frame_transforms.hpp
 * @brief Deterministic inertial <-> RIC relative state transforms.
 *
 * v1 velocity convention (Option B):
 * - Inputs are instantaneous "snapshots" in the chief inertial frame.
 * - The RIC frame is rotating with angular velocity ω wrt inertial.
 * - ω is expressed in RIC components.
 */

#include "core/types.hpp"

namespace bullseye_pred
{

struct RelState final
{
    Vec3 r{};
    Vec3 v{};
};

[[nodiscard]] RelState inertial_to_ric_relative(const Vec3& veh_r_i, const Vec3& veh_v_i,
                                               const Vec3& chief_r_i, const Vec3& chief_v_i,
                                               const Mat3& C_from_inertial_to_ric,
                                               const Vec3& omega_ric) noexcept;

[[nodiscard]] RelState ric_to_inertial_relative(const Vec3& rel_r_ric, const Vec3& rel_v_ric,
                                               const Vec3& chief_r_i, const Vec3& chief_v_i,
                                               const Mat3& C_from_ric_to_inertial,
                                               const Vec3& omega_ric) noexcept;

} // namespace bullseye_pred
