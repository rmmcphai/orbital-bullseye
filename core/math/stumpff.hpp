// core/math/stumpff.hpp
#pragma once

#include <cmath>

namespace bullseye_pred::math {

// Keep the threshold centralized so all users behave identically near z=0.
inline constexpr double kStumpffSeriesThreshold = 1e-8;

/**
 * @brief Stumpff C function.
 *
 * C(z) = (1 - cos(sqrt(z))) / z              for z > 0
 * C(0) = 1/2
 * C(z) = (cosh(sqrt(-z)) - 1) / (-z)         for z < 0
 *
 * Numerically stabilized via a series expansion for small |z|.
 */
[[nodiscard]] inline double stumpff_C(double z) noexcept
{
    const double az = std::fabs(z);

    // Series: C(z) = 1/2 - z/24 + z^2/720 - z^3/40320 + ...
    if (az < kStumpffSeriesThreshold) {
        const double z2 = z * z;
        return 0.5 - z / 24.0 + z2 / 720.0 - (z2 * z) / 40320.0;
    }

    if (z > 0.0) {
        const double s  = std::sqrt(z);
        const double cs = std::cos(s);
        return (1.0 - cs) / z;
    }

    // z < 0
    const double s   = std::sqrt(-z);
    const double chs = std::cosh(s);
    return (chs - 1.0) / (-z);
}

/**
 * @brief Stumpff S function.
 *
 * S(z) = (sqrt(z) - sin(sqrt(z))) / (sqrt(z))^3          for z > 0
 * S(0) = 1/6
 * S(z) = (sinh(sqrt(-z)) - sqrt(-z)) / (sqrt(-z))^3      for z < 0
 *
 * Numerically stabilized via a series expansion for small |z|.
 */
[[nodiscard]] inline double stumpff_S(double z) noexcept
{
    const double az = std::fabs(z);

    // Series: S(z) = 1/6 - z/120 + z^2/5040 - z^3/362880 + ...
    if (az < kStumpffSeriesThreshold) {
        const double z2 = z * z;
        return (1.0 / 6.0) - z / 120.0 + z2 / 5040.0 - (z2 * z) / 362880.0;
    }

    if (z > 0.0) {
        const double s  = std::sqrt(z);
        const double ss = std::sin(s);
        return (s - ss) / (s * s * s);
    }

    // z < 0
    const double s   = std::sqrt(-z);
    const double shs = std::sinh(s);
    return (shs - s) / (s * s * s);
}

} // namespace ob::math
