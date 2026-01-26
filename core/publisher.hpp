#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "core/prediction_buffer.hpp"

namespace bullseye {

/**
 * @brief Double-buffer publisher for PredictionBuffer snapshots.
 *
 * @details
 * Producer workflow:
 *   - PredictionBuffer& back = publisher.begin_write();
 *   - Fill back (positions, metadata, etc.)
 *   - publisher.publish(t0);
 *
 * Consumer workflow:
 *   - const PredictionBuffer& front = publisher.read();
 *   - Use front as immutable snapshot until next read()
 *
 * Threading:
 * - This supports a single producer with any number of readers.
 * - Publication uses release/acquire semantics so readers see a fully-written snapshot.
 */
class Publisher final {
 public:
  Publisher() = default;

  /// @return reference to the writable back buffer.
  PredictionBuffer& begin_write() noexcept;

  /**
   * @brief Publish the back buffer as the new front snapshot.
   *
   * @param t0 Epoch time for this prediction snapshot.
   * @return The new published seqno.
   */
  std::uint64_t publish(double t0) noexcept;

  /// @return current immutable front snapshot (acquire).
  const PredictionBuffer& read() const noexcept;

  /// @return most recently published seqno (acquire).
  std::uint64_t published_seqno() const noexcept;

 private:
  static constexpr std::size_t kNumBuffers = 2;

  // Two buffers: one is front (visible), one is back (writable).
  PredictionBuffer buffers_[kNumBuffers]{};

  // Index of the front buffer (0 or 1).
  std::atomic<std::size_t> front_index_{0};

  // Monotonic publish sequence number.
  std::atomic<std::uint64_t> seqno_{0};
};

} // namespace bullseye
