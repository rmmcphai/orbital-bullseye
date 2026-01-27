// core/frame_transforms.cpp

#include "core/frame_transforms.hpp"

namespace bullseye_pred
{

RelState inertial_to_ric_relative(const Vec3& veh_r_i, const Vec3& veh_v_i,
                                 const Vec3& chief_r_i, const Vec3& chief_v_i,
                                 const Mat3& C_from_inertial_to_ric,
                                 const Vec3& omega_ric) noexcept
{
    const Vec3 dr_i = veh_r_i - chief_r_i;
    const Vec3 dv_i = veh_v_i - chief_v_i;

    const Vec3 r_ric = mul(C_from_inertial_to_ric, dr_i);
    const Vec3 v_ric = mul(C_from_inertial_to_ric, dv_i) - cross(omega_ric, r_ric);
    return RelState{r_ric, v_ric};
}

RelState ric_to_inertial_relative(const Vec3& rel_r_ric, const Vec3& rel_v_ric,
                                 const Vec3& chief_r_i, const Vec3& chief_v_i,
                                 const Mat3& C_from_ric_to_inertial,
                                 const Vec3& omega_ric) noexcept
{
    const Vec3 dr_i = mul(C_from_ric_to_inertial, rel_r_ric);
    const Vec3 dv_i = mul(C_from_ric_to_inertial, rel_v_ric + cross(omega_ric, rel_r_ric));

    const Vec3 r_i = chief_r_i + dr_i;
    const Vec3 v_i = chief_v_i + dv_i;
    return RelState{r_i, v_i};
}

} // namespace bullseye_pred
