// core/provider_cartesian.cpp
#include "core/provider_cartesian.hpp"

#include "core/logging.hpp"
#include "core/log_names.hpp"
#include "logger/log_macros.hpp"

#include <algorithm>
#include <limits>

namespace bullseye_pred {


CartesianChiefProvider::CartesianChiefProvider(const char* inertial_frame_id,
                                               Mode mode,
                                               double warn_period_sec)
    : inertial_frame_id_(inertial_frame_id),
      mode_(mode),
      warn_period_sec_(warn_period_sec) {
  static auto log = bullseye::logging::get(bullseye::logname::kCoreProviderCartesian);

  // Initialize current_ to a sentinel time that should never match a real t0.
  current_.t = std::numeric_limits<double>::quiet_NaN();

  LOG_INFOF(log, "init: mode=%s frame_id=%s warn_period_sec=%.17g",
            (mode_ == Mode::kCurrent) ? "current" : "timeseries",
            inertial_frame_id_ ? inertial_frame_id_ : "(null)",
            warn_period_sec_);
}

void CartesianChiefProvider::set_current(double t, const Vec3& r_i, const Vec3& v_i) noexcept {
  static auto log = bullseye::logging::get(bullseye::logname::kCoreProviderCartesian);

  current_.t = t;
  current_.r_i = r_i;
  current_.v_i = v_i;

  LOG_DEBUGF(log, "set_current: t=%.17g", t);
}

void CartesianChiefProvider::add_sample(double t, const Vec3& r_i, const Vec3& v_i) {
  static auto log = bullseye::logging::get(bullseye::logname::kCoreProviderCartesian);

  samples_.push_back(Sample{t, r_i, v_i});
  sorted_ = false;

  LOG_DEBUGF(log, "add_sample: t=%.17g count=%zu", t, samples_.size());
}

void CartesianChiefProvider::clear_samples() noexcept {
  static auto log = bullseye::logging::get(bullseye::logname::kCoreProviderCartesian);
  samples_.clear();
  // Configuration-time operation; acceptable. Avoid calling during steady-state ticks.
  samples_.shrink_to_fit();
  sorted_ = true;

  LOG_DEBUGF(log, "clear_samples");
}

void CartesianChiefProvider::ensure_sorted_() noexcept {
  if (sorted_) {
    return;
  }
  std::sort(samples_.begin(), samples_.end(),
            [](const Sample& a, const Sample& b) { return a.t < b.t; });
  sorted_ = true;
}

bool CartesianChiefProvider::should_warn_time_missing_(double t0) noexcept {
  if (warn_period_sec_ <= 0.0) {
    return true;  // no rate limit if configured <= 0
  }
  if ((t0 - last_warn_t0_) >= warn_period_sec_) {
    last_warn_t0_ = t0;
    return true;
  }
  return false;
}

void CartesianChiefProvider::log_invalid_input_once_() noexcept {
  if (invalid_logged_) {
    return;
  }
  invalid_logged_ = true;
  static auto log = bullseye::logging::get(bullseye::logname::kCoreProviderCartesian);
  LOG_ERRORF(log, "invalid configuration: inertial_frame_id is null");
}

ChiefState CartesianChiefProvider::get(double t0) noexcept {
  ChiefState out{};
  out.frame_id = inertial_frame_id_;

  if (inertial_frame_id_ == nullptr) {
    out.status.code = ProviderCode::kInvalidInput;
    log_invalid_input_once_();
    return out;
  }

  if (mode_ == Mode::kCurrent) {
    // FR-14 exact-time: require the caller to have set current_.t == t0.
    if (!(current_.t == t0)) {  // exact compare by contract
      out.status.code = ProviderCode::kTimeMissing;

      if (should_warn_time_missing_(t0)) {
        static auto log = bullseye::logging::get(bullseye::logname::kCoreProviderCartesian);
        LOG_WARNF(log, "get: time missing (mode=current) t0=%.17g current_t=%.17g",
                  t0, current_.t);
      }
      return out;
    }

    out.time_tag = current_.t;
    out.r_i = current_.r_i;
    out.v_i = current_.v_i;
    out.status.code = ProviderCode::kOk;
    return out;
  }

  // TimeSeries mode: exact match lookup in sorted samples.
  ensure_sorted_();

  auto it = std::lower_bound(samples_.begin(), samples_.end(), t0,
                             [](const Sample& s, double t) { return s.t < t; });

  if (it == samples_.end() || !(it->t == t0)) {  // exact compare by contract
    out.status.code = ProviderCode::kTimeMissing;

    if (should_warn_time_missing_(t0)) {
      static auto log = bullseye::logging::get(bullseye::logname::kCoreProviderCartesian);
      const double found_t = (it == samples_.end()) ? std::numeric_limits<double>::quiet_NaN()
                                                    : it->t;
      LOG_WARNF(log, "get: time missing (mode=timeseries) t0=%.17g next_sample_t=%.17g count=%zu",
                t0, found_t, samples_.size());
    }
    return out;
  }

  out.time_tag = it->t;
  out.r_i = it->r_i;
  out.v_i = it->v_i;
  out.status.code = ProviderCode::kOk;
  return out;
}

}  // namespace bullseye_pred
