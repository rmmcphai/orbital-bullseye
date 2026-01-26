#include "core/dummy_predictor.hpp"

#include "core/log_names.hpp"
#include "core/logging.hpp"
#include "logger/log_macros.hpp"

#include <algorithm> // std::min

namespace bullseye {

void DummyPredictor::step(double t0, double horizon_sec, double cadence_sec) noexcept {
  static auto log = bullseye::logging::get(bullseye::logname::kCoreDummyPredictor);

  const auto grid = make_time_grid(horizon_sec, cadence_sec);
  if (grid.tau.empty()) {
    LOG_WARNF(log, "step: empty grid (t0=%.17g horizon=%.17g cadence=%.17g)",
              t0, horizon_sec, cadence_sec);
    return;
  }

  auto& buf = pub_.begin_write();

  // Determine how many steps we can write without exceeding MAX_STEPS.
  const std::size_t steps = std::min<std::size_t>(grid.tau.size(), MAX_STEPS);

  // For Sprint 1: fill only registered vehicles [0..map_.size()).
  const std::size_t nveh = std::min<std::size_t>(map_.size(), MAX_VEHICLES);

  // Deterministic fill function:
  //   pos.x = i + 0.001*k
  //   pos.y = k + 0.01*i
  //   pos.z = tau[k]
  //
  // Note: tau[k] itself is deterministic from make_time_grid().
  for (std::size_t i = 0; i < nveh; ++i) {
    for (std::size_t k = 0; k < steps; ++k) {
      const double tau = grid.tau[k];
      buf.positions[i][k] = {
          static_cast<double>(i) + 0.001 * static_cast<double>(k),
          static_cast<double>(k) + 0.01  * static_cast<double>(i),
          tau
      };
    }
  }

  // Optional: if you want deterministic "unused" region behavior, you can zero it here.
  // Not required yet; leaving stale values is fine as long as consumers gate on step count.

  const auto seq = pub_.publish(t0);

  LOG_DEBUGF(log, "step: published seqno=%llu t0=%.17g nveh=%zu steps=%zu",
             static_cast<unsigned long long>(seq), t0, nveh, steps);
}

} // namespace bullseye
