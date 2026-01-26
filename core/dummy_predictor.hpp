#pragma once

#include <cstddef>

#include "core/publisher.hpp"
#include "core/time_grid.hpp"
#include "core/vehicle_index_map.hpp"

namespace bullseye {

/**
 * @brief Deterministic "dummy" predictor used to validate the publication pipeline.
 *
 * @details
 * Fills positions with a simple deterministic function of:
 *   - vehicle index i
 *   - step index k
 *   - time offset tau[k]
 *
 * This is not physically meaningful. It exists to validate buffer layout, indexing,
 * determinism, and publish/read semantics.
 */
class DummyPredictor final {
 public:
  DummyPredictor(Publisher& publisher,
                 VehicleIndexMap& vehicle_map) noexcept
      : pub_(publisher), map_(vehicle_map) {}

  /**
   * @brief Compute and publish a snapshot.
   *
   * @param t0 Epoch time for the snapshot.
   * @param horizon_sec Prediction horizon (seconds).
   * @param cadence_sec Sampling cadence (seconds).
   */
  void step(double t0, double horizon_sec, double cadence_sec) noexcept;

 private:
  Publisher& pub_;
  VehicleIndexMap& map_;
};

} // namespace bullseye
