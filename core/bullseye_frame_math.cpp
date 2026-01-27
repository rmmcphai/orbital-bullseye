// core/bullseye_frame_math.cpp

#include "core/bullseye_frame_math.hpp"

#include <cmath>

namespace bullseye_pred
{

static inline bool is_finite_vec(const Vec3& v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

static inline Vec3 unit_or_fail(const Vec3& v, double n, bool& ok) noexcept
{
    if (!(n > 0.0) || !std::isfinite(n))
    {
        ok = false;
        return Vec3{};
    }
    return (1.0 / n) * v;
}

ConstructedRicFrame construct_ric_from_chief(const ChiefState& chief) noexcept
{
    ConstructedRicFrame out;

    out.time_tag = chief.time_tag;
    out.origin_i = chief.r_i;

    if (!chief.status.ok())
    {
        out.status.code = ProviderCode::kNotAvailable;
        return out;
    }
    if (!is_finite_vec(chief.r_i) || !is_finite_vec(chief.v_i))
    {
        out.status.code = ProviderCode::kInvalidInput;
        return out;
    }

    const double r_norm = norm(chief.r_i);
    const double v_norm = norm(chief.v_i);

    if (!(r_norm >= contracts::Tol::kRmin_m) || !(v_norm >= contracts::Tol::kVmin_mps))
    {
        out.status.code = ProviderCode::kNotAvailable;
        return out;
    }

    const Vec3 h = cross(chief.r_i, chief.v_i);
    const double h_norm = norm(h);

    // Dimensionless degeneracy check: h_hat = |r×v| / (|r||v|) = sin(theta)
    const double h_hat = h_norm / (r_norm * v_norm);
    if (!(h_hat >= contracts::Tol::kHhatMin) || !std::isfinite(h_hat))
    {
        out.status.code = ProviderCode::kNotAvailable;
        return out;
    }

    bool ok = true;
    const Vec3 eR_i = unit_or_fail(chief.r_i, r_norm, ok);
    Vec3 eC_i = unit_or_fail(h, h_norm, ok);
    
    // In-track definition: transverse velocity component perpendicular to R.
    // Using t = h x r = |r|^2v - (r·v)r
    const Vec3 t_i = cross(h, chief.r_i);
    const double t_norm = norm(t_i);
    Vec3 eI_i = unit_or_fail(t_i, t_norm, ok);

    // Re-orthonormalize to guarante e a right-handed frame.
    // C = R x I
    const Vec3 c_i = cross(eR_i, eI_i);
    const double c_norm = norm(c_i);
    eC_i = unit_or_fail(c_i, c_norm, ok);

    // Recompute I = C x R to ensure orthogonality.
    const Vec3 i_i = cross(eC_i, eR_i);
    const double i_norm = norm(i_i);
    eI_i = unit_or_fail(i_i, i_norm, ok);

    if (!ok || !is_finite_vec(eR_i) || !is_finite_vec(eI_i) || !is_finite_vec(eC_i))
    {
        out.status.code = ProviderCode::kInternalError;
        return out;
    }

    // Build C_from_ric_to_inertial with columns = basis vectors in inertial components.
    Mat3 C;
    // Column 0: R
    C(0, 0) = eR_i.x;
    C(1, 0) = eR_i.y;
    C(2, 0) = eR_i.z;
    // Column 1: I
    C(0, 1) = eI_i.x;
    C(1, 1) = eI_i.y;
    C(2, 1) = eI_i.z;
    // Column 2: C
    C(0, 2) = eC_i.x;
    C(1, 2) = eC_i.y;
    C(2, 2) = eC_i.z;

    out.C_from_ric_to_inertial = C;

    // ω magnitude = |h| / |r|^2. Expressed in RIC coordinates: [0,0,ω].
    const double omega_mag = h_norm / (r_norm * r_norm);
    if (!std::isfinite(omega_mag))
    {
        out.status.code = ProviderCode::kInternalError;
        return out;
    }
    out.omega_ric = Vec3{0.0, 0.0, omega_mag};

    out.status.code = ProviderCode::kOk;
    return out;
}

} // namespace bullseye_pred
