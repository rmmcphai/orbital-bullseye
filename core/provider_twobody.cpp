// core/provider_twobody.cpp
#include "core/provider_twobody.hpp"

#include "core/log_names.hpp"
#include "core/logging.hpp"
#include "core/math/stumpff.hpp"
#include "logger/log_macros.hpp"

#include <cmath>
#include <limits>

namespace bullseye_pred
{
using math::stumpff_C;
using math::stumpff_S;

inline bool is_finite_vec(const Vec3& v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

TwoBodyChiefProvider::TwoBodyChiefProvider(const char* inertial_frame_id, double mu, double t_epoch,
                                           const Vec3& r_epoch_i, const Vec3& v_epoch_i)
    : inertial_frame_id_(inertial_frame_id), mu_(mu), t_epoch_(t_epoch), r0_(r_epoch_i),
      v0_(v_epoch_i)
{
    static auto log = bullseye_pred::logging::get(bullseye_pred::logname::kCoreProviderTwoBody);

    LOG_INFOF(log, "init: frame_id=%s mu=%.17g t_epoch=%.17g",
              inertial_frame_id_ ? inertial_frame_id_ : "(null)", mu_, t_epoch_);

    // Configuration validation is deferred to get() for status reporting, but we can
    // latch obvious issues now (still log once there).
}

void TwoBodyChiefProvider::log_invalid_config_once_(const char* why) noexcept
{
    if (invalid_logged_)
    {
        return;
    }
    invalid_logged_ = true;
    static auto log = bullseye_pred::logging::get(bullseye_pred::logname::kCoreProviderTwoBody);
    LOG_ERRORF(log, "invalid configuration: %s", why);
}

ChiefState TwoBodyChiefProvider::get(double t0) noexcept
{
    ChiefState out{};
    out.frame_id = inertial_frame_id_;

    // Validate configuration.
    if (inertial_frame_id_ == nullptr)
    {
        out.status.code = ProviderCode::kInvalidInput;
        log_invalid_config_once_("inertial_frame_id is null");
        return out;
    }
    if (!(mu_ > 0.0) || !std::isfinite(mu_))
    {
        out.status.code = ProviderCode::kInvalidInput;
        log_invalid_config_once_("mu must be finite and > 0");
        return out;
    }
    if (!std::isfinite(t0) || !std::isfinite(t_epoch_) || !is_finite_vec(r0_) ||
        !is_finite_vec(v0_))
    {
        out.status.code = ProviderCode::kInvalidInput;
        log_invalid_config_once_("non-finite input(s)");
        return out;
    }

    const double r0n = norm(r0_);
    if (!(r0n > 0.0) || !std::isfinite(r0n))
    {
        out.status.code = ProviderCode::kInvalidInput;
        log_invalid_config_once_("||r_epoch|| must be finite and > 0");
        return out;
    }

    const double dt = t0 - t_epoch_;
    const double sqrt_mu = std::sqrt(mu_);
    const double v0n2 = dot(v0_, v0_);
    const double alpha = 2.0 / r0n - v0n2 / mu_; // reciprocal semi-major axis

    // Initial guess for universal anomaly x (deterministic heuristic).
    double x = 0.0;
    const double abs_alpha = std::fabs(alpha);
    if (abs_alpha > 1e-8)
    {
        x = sqrt_mu * abs_alpha * dt;
    }
    else
    {
        // Near-parabolic fallback.
        x = sqrt_mu * dt / r0n;
    }

    // Fixed-iteration Newton solve of Kepler's universal equation.
    const double r0dotv0 = dot(r0_, v0_);
    const double r0dotv0_over_sqrtmu = r0dotv0 / sqrt_mu;

    for (int iter = 0; iter < kKeplerIters; ++iter)
    {
        const double x2 = x * x;
        const double z = alpha * x2;
        const double C = stumpff_C(z);
        const double S = stumpff_S(z);

        // F(x) = r0·v0/sqrt(mu) * x^2*C + (1 - alpha*r0) * x^3*S + r0*x - sqrt(mu)*dt
        const double x3 = x2 * x;
        const double one_minus_alpha_r0 = 1.0 - alpha * r0n;
        const double F =
            r0dotv0_over_sqrtmu * x2 * C + one_minus_alpha_r0 * x3 * S + r0n * x - sqrt_mu * dt;

        // F'(x) = r0·v0/sqrt(mu) * x*(1 - z*S) + (1 - alpha*r0) * x^2*C + r0
        const double dF =
            r0dotv0_over_sqrtmu * x * (1.0 - z * S) + one_minus_alpha_r0 * x2 * C + r0n;

        // Guard against pathological dF ~ 0 (deterministic fallback: no update).
        if (!(dF != 0.0) || !std::isfinite(dF) || !std::isfinite(F))
        {
            // Leave x unchanged; loop continues fixed-count for determinism.
            continue;
        }

        x = x - F / dF;
    }

    // Compute f, g and propagate.
    const double x2 = x * x;
    const double z = alpha * x2;
    const double C = stumpff_C(z);
    const double S = stumpff_S(z);

    const double f = 1.0 - (x2 / r0n) * C;
    const double g = dt - (x2 * x / sqrt_mu) * S;

    const Vec3 r = f * r0_ + g * v0_;
    const double rn = norm(r);

    // Compute fdot, gdot
    // fdot = sqrt(mu)/(r0*rn) * (z*S - 1) * x
    // gdot = 1 - x^2/rn * C
    double fdot = 0.0;
    double gdot = 0.0;
    if (rn > 0.0 && std::isfinite(rn))
    {
        fdot = (sqrt_mu / (r0n * rn)) * (z * S - 1.0) * x;
        gdot = 1.0 - (x2 / rn) * C;
    }
    else
    {
        out.status.code = ProviderCode::kInternalError;
        return out;
    }

    const Vec3 v = fdot * r0_ + gdot * v0_;

    // Final sanity: finite outputs.
    if (!is_finite_vec(r) || !is_finite_vec(v) || !std::isfinite(rn))
    {
        out.status.code = ProviderCode::kInternalError;
        return out;
    }

    out.time_tag = t0; // exact by definition
    out.r_i = r;
    out.v_i = v;
    out.status.code = ProviderCode::kOk;
    return out;
}

} // namespace bullseye_pred
