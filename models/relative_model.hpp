// models/relative_model.hpp
#pragma once

/**
 * @file relative_model.hpp
 * @brief Interfaces and POD types for relative dynamics models (pure math layer).
 *
 * This module defines:
 * - Minimal span-like views (non-owning) for deterministic, allocation-free output.
 * - Relative state representation in the Bullseye RIC frame.
 * - A model interface suitable for HCW/YA-style predictors.
 *
 * Design constraints:
 * - No logging.
 * - No heap allocations in steady-state.
 * - Deterministic iteration order.
 */

#include <cstddef>
#include <cstdint>

#include "core/time_grid.hpp"
#include "core/types.hpp"

namespace bullseye_pred
{

/**
 * @brief Minimal C++17 span-like view (non-owning).
 *
 * @tparam T Element type.
 *
 * This is intentionally tiny to avoid requiring C++20 std::span.
 * The caller retains ownership of the memory. No bounds checking is performed.
 */
template <typename T>
struct Span final
{
    /** @brief Pointer to the first element (may be nullptr if size==0). */
    T* data{nullptr};

    /** @brief Number of elements in the view. */
    std::size_t size{0};

    /** @brief Default constructor (empty span). */
    constexpr Span() = default;

    /**
     * @brief Construct a span from pointer + length.
     *
     * @param p Pointer to first element.
     * @param n Number of elements.
     */
    constexpr Span(T* p, std::size_t n) : data(p), size(n) {}

    /**
     * @brief Mutable element access (unchecked).
     *
     * @param i Index.
     * @return Reference to element i.
     */
    [[nodiscard]] constexpr T& operator[](std::size_t i) noexcept { return data[i]; }

    /**
     * @brief Const element access (unchecked).
     *
     * @param i Index.
     * @return Const reference to element i.
     */
    [[nodiscard]] constexpr const T& operator[](std::size_t i) const noexcept { return data[i]; }
};

/**
 * @brief Relative state expressed in the Bullseye RIC frame.
 *
 * Units:
 * - r_ric: meters
 * - v_ric: meters/second
 *
 * Coordinate order:
 * - x: radial (R)
 * - y: in-track (I)
 * - z: cross-track (C)
 */
struct RelStateRic final
{
    /** @brief Relative position in RIC [m]. */
    Vec3 r_ric{};

    /** @brief Relative velocity in RIC [m/s]. */
    Vec3 v_ric{};
};

/**
 * @brief Status codes for model evaluation.
 *
 * Kept small and deterministic (no strings, no allocations).
 */
enum class ModelCode : std::uint8_t
{
    /** @brief Success. */
    kOk = 0,

    /** @brief Invalid input (non-finite, negative time, invalid parameters, etc.). */
    kInvalidInput,

    /** @brief Output spans are too small for the requested time grid. */
    kInsufficientOutputCapacity,
};

/**
 * @brief Parameter block for HCW (circular reference orbit) models.
 */
struct HcwParams final
{
    /**
     * @brief Chief mean motion [rad/s].
     *
     * Must be finite and strictly > 0.
     */
    double n_radps{0.0};
};

/**
 * @brief Common interface for relative dynamics models.
 *
 * This is the "pure math" boundary. Implementations:
 * - Must not allocate heap memory (steady-state).
 * - Must not perform logging.
 * - Must treat outputs as caller-owned storage via Span.
 *
 * Time semantics:
 * - The TimeGrid provides offsets tau[k] from t0 (i.e., evaluation at t0 + tau[k]).
 */
class IRelativeModel
{
  public:
    /** @brief Virtual destructor for interface. */
    virtual ~IRelativeModel() = default;

    /**
     * @brief Result of a model prediction call.
     */
    struct Result final
    {
        /** @brief Status code. */
        ModelCode code{ModelCode::kOk};

        /** @brief Number of time steps successfully written into outputs. */
        std::size_t steps_written{0};
    };

    /**
     * @brief Predict relative trajectory using Hill-Clohessy-Wiltshire (HCW) equations.
     *
     * Frame:
     * - Inputs and outputs are in the Bullseye RIC frame (R, I, C).
     *
     * Outputs:
     * - out_r_ric is required and must hold grid.tau.size() elements.
     * - out_v_ric is optional; if provided, it must also hold grid.tau.size().
     *
     * Determinism:
     * - Iteration proceeds in ascending k over grid.tau.
     *
     * @param x0_ric Initial relative state at t0 in RIC.
     * @param params HCW parameters (mean motion, etc.).
     * @param grid Time grid of offsets tau from t0.
     * @param out_r_ric Output positions per time step (required).
     * @param out_v_ric Output velocities per time step (optional; may be {nullptr,0}).
     * @return Result code and number of steps written.
     */
    [[nodiscard]] virtual Result predict_hcw(const RelStateRic& x0_ric,
                                            const HcwParams& params,
                                            const TimeGrid& grid,
                                            Span<Vec3> out_r_ric,
                                            Span<Vec3> out_v_ric) const noexcept = 0;
};

} // namespace bullseye_pred
