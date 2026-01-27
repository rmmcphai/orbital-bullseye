#pragma once
/**
 * @file time_grid.hpp
 * @brief Utilities for generating a discrete set of prediction times relative to a reference epoch
 * t0.
 *
 * @details
 * This module provides a simple, deterministic time grid generator used by predictors and buffers.
 * The grid is expressed as offsets \f$\tau_k\f$ (seconds) from a caller-defined epoch \f$t_0\f$.
 *
 * The intent is to centralize "how many future samples and at what spacing" in one place,
 * so buffers and predictors agree on the sampling schedule.
 *
 * @par Design goals
 * - Deterministic: same inputs produce same grid (bitwise-identical offsets where possible).
 * - Lightweight: minimal dependencies and small API surface.
 * - Explicit semantics: includes \f$\tau = 0\f$ (sample at t0) and never exceeds the horizon.
 *
 * @note
 * This is not a propagator. It does not compute states, only the sampling times.
 */

#include <vector>

namespace bullseye_pred
{

/**
 * @brief Discrete sampling schedule expressed as offsets from t0.
 *
 * @details
 * The grid is stored as a monotonically non-decreasing list of offsets in seconds:
 * \f[
 *   \tau_0 = 0,\quad \tau_k = k \cdot \Delta t
 * \f]
 * with \f$\tau_k \le \text{horizon}\f$.
 *
 * @note
 * Using offsets (rather than absolute times) avoids accidental dependence on time systems
 * and makes tests simpler and more deterministic.
 */
struct TimeGrid
{
    /// Offsets (seconds) from epoch t0. The first element is always 0.0 for valid inputs.
    std::vector<double> tau;
};

/**
 * @brief Generate a uniform time grid from 0 to horizon inclusive, spaced by cadence.
 *
 * @param horizon_sec Prediction horizon (seconds). Must be >= 0.
 * @param cadence_sec Sampling cadence (seconds). Must be > 0.
 *
 * @return A TimeGrid containing offsets in seconds from t0.
 *
 * @details
 * Semantics:
 * - If inputs are valid, the returned grid includes \f$\tau_0 = 0\f$.
 * - Subsequent samples are spaced by cadence: \f$\tau_k = k \cdot \text{cadence}\f$.
 * - The last sample satisfies \f$\tau_{\text{last}} \le \text{horizon}\f$.
 * - If cadence divides horizon exactly, the last sample equals horizon.
 *
 * Invalid inputs:
 * - If horizon_sec < 0 or cadence_sec <= 0, returns an empty grid.
 *
 * @warning
 * This function uses repeated addition (t += cadence). For many steps, floating-point
 * accumulation error can occur. For typical short horizons (seconds to minutes) this
 * is usually acceptable, but if you require strict numerical properties, consider
 * computing tau_k as (k * cadence) using integer k and a computed step count.
 */
TimeGrid make_time_grid(double horizon_sec, double cadence_sec);

} // namespace bullseye_pred
