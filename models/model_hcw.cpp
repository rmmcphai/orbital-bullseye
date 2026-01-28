// models/model_hcw.cpp

/**
 * @file model_hcw.cpp
 * @brief HCW closed-form solution implementation.
 */

#include "models/model_hcw.hpp"

#include <cmath>

namespace bullseye_pred
{

/**
 * @brief Check if all components of a vector are finite.
 *
 * @param v Vector to check.
 * @return true if all components are finite; false otherwise.
 */
static inline bool finite3(const Vec3& v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

IRelativeModel::Result ModelHCW::predict_hcw(const RelStateRic& x0_ric,
                                            const HcwParams& params,
                                            const TimeGrid& grid,
                                            Span<Vec3> out_r_ric,
                                            Span<Vec3> out_v_ric) const noexcept
{
    Result res{};

    const double n = params.n_radps;

    // Validate parameters.
    if (!(n > 0.0) || !std::isfinite(n))
    {
        res.code = ModelCode::kInvalidInput;
        return res;
    }

    // Validate state.
    if (!finite3(x0_ric.r_ric) || !finite3(x0_ric.v_ric))
    {
        res.code = ModelCode::kInvalidInput;
        return res;
    }

    // Validate output capacity.
    const std::size_t steps = grid.tau.size();
    if (steps == 0)
    {
        res.code = ModelCode::kOk;
        res.steps_written = 0;
        return res;
    }
    if (out_r_ric.data == nullptr || out_r_ric.size < steps)
    {
        res.code = ModelCode::kInsufficientOutputCapacity;
        return res;
    }

    // Velocities are optional; compute only if storage is present and large enough.
    const bool want_vel = (out_v_ric.data != nullptr && out_v_ric.size >= steps);

    // Unpack initial conditions in RIC axes: x=R, y=I, z=C.
    const double x0  = x0_ric.r_ric.x;
    const double y0  = x0_ric.r_ric.y;
    const double z0  = x0_ric.r_ric.z;

    const double xd0 = x0_ric.v_ric.x;
    const double yd0 = x0_ric.v_ric.y;
    const double zd0 = x0_ric.v_ric.z;

    const double inv_n = 1.0 / n;

    /**
     * HCW closed-form solution (one standard form):
     *
     * x(t) = (4 - 3 cos nt) x0 + (1/n) sin nt * xd0 + (2/n)(1 - cos nt) * yd0
     * y(t) = y0 + 6(sin nt - nt) x0 - (2/n)(1 - cos nt) xd0 + (1/n)(4 sin nt - 3 nt) yd0
     * z(t) = cos nt * z0 + (1/n) sin nt * zd0
     *
     * Velocities:
     * xd(t) = 3 n sin nt * x0 + cos nt * xd0 + 2 sin nt * yd0
     * yd(t) = 6 n (cos nt - 1) x0 - 2 sin nt * xd0 + (4 cos nt - 3) yd0
     * zd(t) = -n sin nt * z0 + cos nt * zd0
     */

    for (std::size_t k = 0; k < steps; ++k)
    {
        const double t = grid.tau[k];

        // HCW is defined for tau >= 0 in this v1 predictor context.
        if (!(t >= 0.0) || !std::isfinite(t))
        {
            res.code = ModelCode::kInvalidInput;
            return res;
        }

        const double nt = n * t;
        const double s  = std::sin(nt);
        const double c  = std::cos(nt);

        // Position
        const double x = (4.0 - 3.0 * c) * x0
                       + inv_n * s * xd0
                       + (2.0 * inv_n) * (1.0 - c) * yd0;

        const double y = 6.0 * (s - nt) * x0
                       + y0
                       - (2.0 * inv_n) * (1.0 - c) * xd0
                       + inv_n * (4.0 * s - 3.0 * nt) * yd0;

        const double z = c * z0 + inv_n * s * zd0;

        out_r_ric[k] = Vec3{x, y, z};

        if (want_vel)
        {
            // Velocity
            const double xd = 3.0 * n * s * x0 + c * xd0 + 2.0 * s * yd0;
            const double yd = 6.0 * n * (c - 1.0) * x0 - 2.0 * s * xd0 + (4.0 * c - 3.0) * yd0;
            const double zd = -n * s * z0 + c * zd0;

            out_v_ric[k] = Vec3{xd, yd, zd};
        }
    }

    res.code = ModelCode::kOk;
    res.steps_written = steps;
    return res;
}

} // namespace bullseye_pred
