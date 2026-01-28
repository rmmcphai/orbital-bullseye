// core/relative_predictor.hpp
#pragma once
/**
 * @file relative_predictor.hpp
 * @brief End-to-end relative predictor orchestrator (providers + bullseye + transforms + model).
 *
 * This is the "glue" layer:
 * - retrieves chief inertial state at exactly t0 (FR-14 via provider contract)
 * - updates BullseyeFrame snapshot at t0
 * - retrieves deputy inertial state(s) at exactly t0
 * - computes initial relative state in RIC using Option-B semantics
 * - runs HCW model and writes results into PredictionBuffer
 * - publishes via Publisher (double-buffer atomic publish)
 *
 * Design constraints:
 * - deterministic iteration order
 * - no heap allocation in steady-state
 * - fail-fast: do not publish on invalid inputs
 */

#include <cstddef>

#include "core/bullseye_frame.hpp"
#include "core/chief_state_provider.hpp"
#include "core/constants.hpp"
#include "core/publisher.hpp"
#include "core/time_grid.hpp"
#include "core/types.hpp"
#include "core/vehicle_index_map.hpp"

namespace bullseye_pred
{

/**
 * @brief Minimal Cartesian state for a deputy vehicle in the inertial frame.
 *
 * Contract highlights (mirrors chief provider contracts):
 * - On success: time_tag == requested t0 and frame_id != nullptr.
 * - On failure: status.code != kOk and other fields unspecified.
 */
struct VehicleState final
{
    double time_tag{0.0};
    Vec3 r_i{};
    Vec3 v_i{};
    const char* frame_id{nullptr};
    ProviderStatus status{};
};

/**
 * @brief Interface for retrieving deputy vehicle inertial states at the predictor tick.
 *
 * This is intentionally small and dependency-free (no JEOD/Trick).
 */
class IVehicleStateProvider
{
  public:
    virtual ~IVehicleStateProvider() = default;

    /**
     * @brief Get vehicle state for exactly the requested tick time t0.
     *
     * @param vehicle_id Vehicle identifier (from VehicleIndexMap).
     * @param t0 Requested predictor tick time (seconds).
     * @return VehicleState for exactly t0 on success; non-OK status otherwise.
     */
    [[nodiscard]] virtual VehicleState get(VehicleIndexMap::VehicleId id, double t0) noexcept = 0;
};

/**
 * @brief Relative predictor that produces HCW trajectories in the Bullseye RIC frame.
 *
 * Output:
 * - Writes predicted relative positions into PredictionBuffer::positions:
 *   positions[i][k] = predicted RIC position for vehicle index i at grid.tau[k].
 *
 * Notes:
 * - Current PredictionBuffer has no explicit step count. Consumers must know the configured
 *   horizon/cadence or use a separately-shared TimeGrid.
 */
class RelativePredictor final
{
  public:
    /**
     * @brief Construct a predictor.
     *
     * @param publisher Output publisher (double-buffered).
     * @param vehicle_map Stable mapping from vehicle_id -> index [0..MAX_VEHICLES).
     * @param chief_provider Chief state provider.
     * @param vehicle_provider Deputy state provider.
     * @param bullseye Bullseye frame product (constructed/adopted policy).
     */
    RelativePredictor(Publisher& publisher,
                      VehicleIndexMap& vehicle_map,
                      IChiefStateProvider& chief_provider,
                      IVehicleStateProvider& vehicle_provider,
                      BullseyeFrame& bullseye) noexcept
        : pub_(publisher),
          map_(vehicle_map),
          chief_(chief_provider),
          veh_(vehicle_provider),
          bullseye_(bullseye)
    {
    }

    /**
     * @brief Compute and publish HCW predictions for registered vehicles.
     *
     * @param t0 Epoch time for this prediction snapshot.
     * @param horizon_sec Prediction horizon (seconds).
     * @param cadence_sec Sampling cadence (seconds).
     */
    void step(double t0, double horizon_sec, double cadence_sec) noexcept;

  private:
    Publisher& pub_;
    VehicleIndexMap& map_;
    IChiefStateProvider& chief_;
    IVehicleStateProvider& veh_;
    BullseyeFrame& bullseye_;
};

} // namespace bullseye_pred
