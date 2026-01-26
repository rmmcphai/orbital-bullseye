#include "core/publisher.hpp"

#include "core/log_names.hpp"
#include "core/logging.hpp"
#include "logger/log_macros.hpp"

namespace bullseye {

PredictionBuffer& Publisher::begin_write() noexcept {
  const std::size_t front = front_index_.load(std::memory_order_acquire);
  const std::size_t back  = 1u - front;
  return buffers_[back];
}

std::uint64_t Publisher::publish(double t0) noexcept {
  static auto log = bullseye::logging::get(bullseye::logname::kCorePublisher);

  // Determine back buffer (the one not currently visible).
  const std::size_t front = front_index_.load(std::memory_order_acquire);
  const std::size_t back  = 1u - front;

  // Increment seqno and stamp into the buffer before publishing.
  const std::uint64_t new_seq = seqno_.fetch_add(1u, std::memory_order_relaxed) + 1u;
  buffers_[back].seqno = new_seq;
  buffers_[back].t0    = t0;

  // Publish: release so all writes to buffers_[back] become visible to readers.
  front_index_.store(back, std::memory_order_release);

  LOG_INFOF(log, "publish seqno=%llu t0=%.17g front=%zu",
            static_cast<unsigned long long>(new_seq), t0, back);

  return new_seq;
}

const PredictionBuffer& Publisher::read() const noexcept {
  const std::size_t front = front_index_.load(std::memory_order_acquire);
  return buffers_[front];
}

std::uint64_t Publisher::published_seqno() const noexcept {
  // The authoritative seqno is the seqno stamped into the currently-front buffer.
  // Using front buffer avoids any ambiguity about ordering of seqno_ vs front_index_.
  const auto& front = read();
  return front.seqno;
}

} // namespace bullseye
