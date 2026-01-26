#pragma once
/**
 * @file prediction_buffer.hpp
 * @brief Preallocated storage for predicted trajectories and associated per-tick metadata.
 *
 * @details
 * A prediction buffer holds a set of future samples for multiple vehicles.
 *
 * In Sprint 1, the goal is to establish:
 * - Fixed capacity and predictable layout (no per-tick heap allocation).
 * - Deterministic addressing (vehicle index, time-step index).
 * - Minimal per-tick metadata (sequence number, epoch time).
 *
 * Later sprints can expand this structure to include:
 * - Relative state (position/velocity), covariances, additional frames
 * - Model provenance and quality flags
 * - Error/status fields and per-vehicle validity windows
 *
 * @par Layout rationale
 * The storage is "vehicle-major": positions[vehicle_index][step_index].
 * This makes each vehicle’s trajectory contiguous in memory, which is cache-friendly for
 * per-vehicle consumers (e.g., UI drawing or guidance logic).
 *
 * @note
 * This is a data container. Publication/visibility rules (front/back buffers, atomics)
 * belong in publisher.*.
 */

#include <array>
#include <cstddef>
#include <cstdint>

namespace bullseye {

/**
 * @brief Maximum number of simultaneously tracked vehicles in the prediction product.
 *
 * @details
 * This is a fixed capacity bound used to preallocate storage. Increasing it increases
 * memory footprint deterministically.
 */
constexpr std::size_t MAX_VEHICLES = 32;

/**
 * @brief Maximum number of time samples per vehicle trajectory.
 *
 * @details
 * Sprint requirements commonly need at least 60 seconds of look-ahead at 1 Hz plus t0,
 * implying a minimum of 61 samples. We enforce >= 61 in unit tests.
 *
 * @note
 * This constant is intentionally compile-time to avoid runtime allocation.
 */
constexpr std::size_t MAX_STEPS = 64; // Must satisfy MAX_STEPS >= 61

/**
 * @brief Simple 3D vector POD type.
 *
 * @details
 * Kept intentionally minimal for Sprint 1. Later you may replace with a richer
 * math type, but keep ABI/layout considerations in mind if this is published externally.
 */
struct Vec3 {
  double x, y, z;
};

/**
 * @brief Prediction product for a single tick/update.
 *
 * @details
 * The buffer is filled by a producer (predictor) for a given epoch t0 and then
 * published as an immutable snapshot for consumers.
 *
 * Fields:
 * - seqno: Monotonic publication sequence number (increments each publish).
 * - t0: Epoch time associated with the prediction (seconds, caller-defined convention).
 * - positions: Predicted positions for each vehicle at each step offset.
 *
 * @warning
 * This Sprint-1 skeleton does not include validity bits. Consumers must assume all
 * entries are valid unless higher-level logic gates usage. Adding per-vehicle validity
 * is recommended before integrating into a wider sim.
 */
struct PredictionBuffer {
  /// Publication sequence number (monotonic). Used to detect new snapshots.
  std::uint64_t seqno = 0;

  /// Prediction epoch time associated with this buffer (seconds).
  double t0 = 0.0;

  /**
   * @brief Vehicle-major predicted positions.
   *
   * @details
   * positions[i][k] is the predicted position for vehicle index i at time-step k.
   *
   * Time-step k is interpreted in conjunction with a TimeGrid:
   * - k=0 corresponds to tau=0 (t0 sample).
   * - k>0 corresponds to tau[k] offsets as produced by make_time_grid(...).
   */
  std::array<std::array<Vec3, MAX_STEPS>, MAX_VEHICLES> positions{};
};

} // namespace bullseye
