// models/model_ya_stm.cpp

/**
 * @file model_ya_stm.cpp
 * @brief Deterministic eccentric-reference relative dynamics model (TH/YA-family).
 */

#include "models/model_ya_stm.hpp"
#include "core/math/stumpff.hpp"

#include <cmath>
#include <cstdint>

namespace bullseye_pred
{
using math::stumpff_C;
using math::stumpff_S;
namespace
{
// ------------------------------
// Helpers (deterministic, no heap)
// ------------------------------

static inline bool finite3(const Vec3& v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}


struct ChiefPv final
{
    Vec3 r_i{};
    Vec3 v_i{};
};

// Deterministic universal-variable propagation from (r0,v0) at t0 to t0+dt.
// Fixed iteration count keeps control flow deterministic.
inline bool propagate_two_body_universal(const Vec3& r0_i,
                                        const Vec3& v0_i,
                                        double mu,
                                        double dt,
                                        ChiefPv& out) noexcept
{
    if (!(mu > 0.0) || !std::isfinite(mu) || !std::isfinite(dt) || !finite3(r0_i) || !finite3(v0_i))
    {
        return false;
    }

    const double r0n = norm(r0_i);
    if (!(r0n > 0.0) || !std::isfinite(r0n))
    {
        return false;
    }

    const double sqrt_mu = std::sqrt(mu);
    const double v0n2 = dot(v0_i, v0_i);
    const double alpha = 2.0 / r0n - v0n2 / mu; // reciprocal semi-major axis

    // Deterministic initial guess.
    double x = 0.0;
    const double abs_alpha = std::fabs(alpha);
    if (abs_alpha > 1e-8)
    {
        x = sqrt_mu * abs_alpha * dt;
    }
    else
    {
        x = sqrt_mu * dt / r0n;
    }

    const double r0dotv0 = dot(r0_i, v0_i);
    const double r0dotv0_over_sqrtmu = r0dotv0 / sqrt_mu;

    constexpr int kKeplerIters = 8;
    for (int iter = 0; iter < kKeplerIters; ++iter)
    {
        const double x2 = x * x;
        const double z = alpha * x2;
        const double C = stumpff_C(z);
        const double S = stumpff_S(z);

        const double x3 = x2 * x;
        const double one_minus_alpha_r0 = 1.0 - alpha * r0n;

        const double F =
            r0dotv0_over_sqrtmu * x2 * C + one_minus_alpha_r0 * x3 * S + r0n * x - sqrt_mu * dt;

        const double dF =
            r0dotv0_over_sqrtmu * x * (1.0 - z * S) + one_minus_alpha_r0 * x2 * C + r0n;

        if (!(dF != 0.0) || !std::isfinite(dF) || !std::isfinite(F))
        {
            continue;
        }

        x = x - F / dF;
    }

    const double x2 = x * x;
    const double z = alpha * x2;
    const double C = stumpff_C(z);
    const double S = stumpff_S(z);

    const double f = 1.0 - (x2 / r0n) * C;
    const double g = dt - (x2 * x / sqrt_mu) * S;

    const Vec3 r = f * r0_i + g * v0_i;
    const double rn = norm(r);
    if (!(rn > 0.0) || !std::isfinite(rn))
    {
        return false;
    }

    const double fdot = (sqrt_mu / (r0n * rn)) * (z * S - 1.0) * x;
    const double gdot = 1.0 - (x2 / rn) * C;

    const Vec3 v = fdot * r0_i + gdot * v0_i;

    if (!finite3(r) || !finite3(v))
    {
        return false;
    }

    out.r_i = r;
    out.v_i = v;
    return true;
}

struct State6 final
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double xd{0.0};
    double yd{0.0};
    double zd{0.0};
};

inline State6 add(const State6& a, const State6& b, double scale_b) noexcept
{
    return State6{a.x + scale_b * b.x,
                  a.y + scale_b * b.y,
                  a.z + scale_b * b.z,
                  a.xd + scale_b * b.xd,
                  a.yd + scale_b * b.yd,
                  a.zd + scale_b * b.zd};
}

// LTV dynamics in chief RIC with omega aligned with +C.
inline bool deriv_th_ltv(double t,
                         const YaStmParams& p,
                         const State6& s,
                         State6& dsdt) noexcept
{
    ChiefPv chief{};
    if (!propagate_two_body_universal(p.chief_r0_i, p.chief_v0_i, p.mu, t, chief))
    {
        return false;
    }

    const double r = norm(chief.r_i);
    if (!(r > 0.0) || !std::isfinite(r))
    {
        return false;
    }

    const Vec3 h = cross(chief.r_i, chief.v_i);
    const double hmag = norm(h);
    if (!(hmag > 0.0) || !std::isfinite(hmag))
    {
        return false;
    }

    const double rdot = dot(chief.r_i, chief.v_i) / r;

    const double omega = hmag / (r * r);
    const double omegadot = -2.0 * omega * rdot / r;

    const double inv_r3 = 1.0 / (r * r * r);
    const double mu_over_r3 = p.mu * inv_r3;

    // Gravity gradient in RIC:
    // a_x = +2 mu/r^3 * x
    // a_y = -1 mu/r^3 * y
    // a_z = -1 mu/r^3 * z

    const double omega2 = omega * omega;

    const double xdd = (2.0 * mu_over_r3 + omega2) * s.x + 2.0 * omega * s.yd + omegadot * s.y;
    const double ydd = (omega2 - mu_over_r3) * s.y - 2.0 * omega * s.xd - omegadot * s.x;
    const double zdd = (-mu_over_r3) * s.z;

    dsdt.x = s.xd;
    dsdt.y = s.yd;
    dsdt.z = s.zd;
    dsdt.xd = xdd;
    dsdt.yd = ydd;
    dsdt.zd = zdd;

    return true;
}

inline bool rk4_step(double t,
                     double h,
                     const YaStmParams& p,
                     State6& s) noexcept
{
    State6 k1{}, k2{}, k3{}, k4{};

    if (!deriv_th_ltv(t, p, s, k1))
    {
        return false;
    }

    State6 s2 = add(s, k1, 0.5 * h);
    if (!deriv_th_ltv(t + 0.5 * h, p, s2, k2))
    {
        return false;
    }

    State6 s3 = add(s, k2, 0.5 * h);
    if (!deriv_th_ltv(t + 0.5 * h, p, s3, k3))
    {
        return false;
    }

    State6 s4 = add(s, k3, h);
    if (!deriv_th_ltv(t + h, p, s4, k4))
    {
        return false;
    }

    s.x += (h / 6.0) * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x);
    s.y += (h / 6.0) * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y);
    s.z += (h / 6.0) * (k1.z + 2.0 * k2.z + 2.0 * k3.z + k4.z);

    s.xd += (h / 6.0) * (k1.xd + 2.0 * k2.xd + 2.0 * k3.xd + k4.xd);
    s.yd += (h / 6.0) * (k1.yd + 2.0 * k2.yd + 2.0 * k3.yd + k4.yd);
    s.zd += (h / 6.0) * (k1.zd + 2.0 * k2.zd + 2.0 * k3.zd + k4.zd);

    return std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.z) && std::isfinite(s.xd) &&
           std::isfinite(s.yd) && std::isfinite(s.zd);
}

} // namespace

ModelYA_STM::Result ModelYA_STM::predict_ya_stm(const RelStateRic& x0_ric,
                                               const YaStmParams& params,
                                               const TimeGrid& grid,
                                               Span<Vec3> out_r_ric,
                                               Span<Vec3> out_v_ric) const noexcept
{
    Result res{};

    // Validate parameters.
    if (!(params.mu > 0.0) || !std::isfinite(params.mu))
    {
        res.code = ModelCode::kInvalidInput;
        return res;
    }
    if (!(params.max_dt_sec > 0.0) || !std::isfinite(params.max_dt_sec))
    {
        res.code = ModelCode::kInvalidInput;
        return res;
    }
    if (!finite3(params.chief_r0_i) || !finite3(params.chief_v0_i))
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

    const bool want_vel = (out_v_ric.data != nullptr && out_v_ric.size >= steps);

    // Initial state in RIC.
    State6 s{};
    s.x  = x0_ric.r_ric.x;
    s.y  = x0_ric.r_ric.y;
    s.z  = x0_ric.r_ric.z;
    s.xd = x0_ric.v_ric.x;
    s.yd = x0_ric.v_ric.y;
    s.zd = x0_ric.v_ric.z;

    // Integrate sequentially across tau.
    double t_prev = 0.0;

    for (std::size_t k = 0; k < steps; ++k)
    {
        const double t_target = grid.tau[k];
        if (!(t_target >= 0.0) || !std::isfinite(t_target))
        {
            res.code = ModelCode::kInvalidInput;
            return res;
        }
        if (t_target < t_prev)
        {
            // Not supported: grid must be nondecreasing.
            res.code = ModelCode::kInvalidInput;
            return res;
        }

        double dt = t_target - t_prev;
        if (dt > 0.0)
        {
            const double max_h = params.max_dt_sec;

            // Deterministic subdivision count: N = ceil(dt/max_h).
            const std::uint64_t n_steps = static_cast<std::uint64_t>(std::ceil(dt / max_h));
            const double h = dt / static_cast<double>(n_steps);

            double t = t_prev;
            for (std::uint64_t i = 0; i < n_steps; ++i)
            {
                if (!rk4_step(t, h, params, s))
                {
                    res.code = ModelCode::kInvalidInput;
                    return res;
                }
                t += h;
            }
        }

        out_r_ric[k] = Vec3{s.x, s.y, s.z};
        if (want_vel)
        {
            out_v_ric[k] = Vec3{s.xd, s.yd, s.zd};
        }

        t_prev = t_target;
        res.steps_written = k + 1;
    }

    res.code = ModelCode::kOk;
    return res;
}

} // namespace bullseye_pred
