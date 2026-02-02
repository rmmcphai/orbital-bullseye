// models/model_ya_stm.hpp
#pragma once

/**
 * @file model_ya_stm.hpp
 * @brief Deterministic eccentric-reference relative dynamics model (TH/YA-family).
 *
 * Notes:
 * - This implementation propagates the Tschauner-Hempel (TH) linear time-varying (LTV)
 *   relative dynamics using a fixed-step RK4 integrator.
 * - The classical Yamanaka-Ankersen (YA) solution provides a closed-form STM for TH.
 *   This module is structured so the RK4 core can be replaced by a closed-form STM
 *   later without changing the caller-facing API.
 *
 * Design constraints:
 * - Deterministic control flow (fixed Kepler iterations; fixed RK4 stepping policy).
 * - No logging.
 * - No heap allocations in steady-state.
 */

#include <cstddef>

#include "core/time_grid.hpp"
#include "core/types.hpp"
#include "models/relative_model.hpp"

namespace bullseye_pred
{

/**
 * @brief Parameter block for eccentric-reference (TH/YA-family) model.
 */
struct YaStmParams final
{
    /** @brief Gravitational parameter [m^3/s^2]. Must be finite and > 0. */
    double mu{0.0};

    /** @brief Chief inertial position at t0 [m]. */
    Vec3 chief_r0_i{};

    /** @brief Chief inertial velocity at t0 [m/s]. */
    Vec3 chief_v0_i{};

    /**
     * @brief Maximum RK4 substep size [s].
     *
     * The integrator will subdivide each requested dt into N = ceil(dt / max_dt_sec)
     * substeps. Must be finite and > 0.
     */
    double max_dt_sec{0.25};
};

/**
 * @brief Deterministic TH/YA-family relative motion propagator.
 */
class ModelYA_STM final
{
  public:
    /**
     * @brief Result of a YA/TH prediction call.
     */
    struct Result final
    {
        ModelCode code{ModelCode::kOk};
        std::size_t steps_written{0};
    };

    /**
     * @brief Predict relative trajectory under eccentric-reference LTV dynamics.
     *
     * Frame:
     * - Inputs and outputs are in the Bullseye RIC frame (R, I, C).
     *
     * Time semantics:
     * - grid.tau[k] are offsets from the chief epoch t0.
     * - chief_r0_i/chief_v0_i are the chief inertial state at t0.
     *
     * Outputs:
     * - out_r_ric is required and must hold grid.tau.size() elements.
     * - out_v_ric is optional; if provided, it must also hold grid.tau.size().
     *
     * Determinism:
     * - Chief propagation uses fixed-iteration universal-variable solve.
     * - RK4 uses a deterministic ceil-based subdivision per interval.
     */
    [[nodiscard]] Result predict_ya_stm(const RelStateRic& x0_ric,
                                       const YaStmParams& params,
                                       const TimeGrid& grid,
                                       Span<Vec3> out_r_ric,
                                       Span<Vec3> out_v_ric) const noexcept;
};

} // namespace bullseye_pred
