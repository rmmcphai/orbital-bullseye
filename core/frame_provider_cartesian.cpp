// core/frame_provider_cartesian.cpp
#include "core/frame_provider_cartesian.hpp"

#include "core/log_names.hpp"
#include "core/logging.hpp"
#include "logger/log_macros.hpp"

#include <algorithm>
#include <limits>

namespace bullseye_pred {

CartesianBullseyeFrameProvider::CartesianBullseyeFrameProvider(const char* frame_source_id,
                                                               Mode mode,
                                                               double warn_period_sec):
    frame_source_id_(frame_source_id), 
    mode_(mode), 
    warn_period_sec_(warn_period_sec) {
    static auto log = bullseye::logging::get(bullseye::logname::kCoreFrameProviderCartesian);

    current_.t = std::numeric_limits<double>::quiet_NaN();

    LOG_INFOF(log, "init: mode=%s frame_source_id=%s warn_period_sec=%.17g",
            (mode_ == Mode::kCurrent) ? "current" : "timeseries",
            frame_source_id_ ? frame_source_id_ : "(null)",
            warn_period_sec_);
}

void CartesianBullseyeFrameProvider::set_current(double t,
                                                 const Vec3& origin_i,
                                                 const Mat3& C_from_ric_to_inertial) noexcept {
    static auto log = bullseye::logging::get(bullseye::logname::kCoreFrameProviderCartesian);

    current_.t = t;
    current_.origin_i = origin_i;
    current_.C_from_ric_to_inertial = C_from_ric_to_inertial;
    // Do not implicitly set ω; keep prior setting as-is.
    LOG_DEBUGF(log, "set_current: t=%.17g", t);
}

void CartesianBullseyeFrameProvider::set_current_omega_ric(const Vec3& omega_ric) noexcept {
    static auto log = bullseye::logging::get(bullseye::logname::kCoreFrameProviderCartesian);

    current_.has_omega = true;
    current_.omega_ric = omega_ric;
    LOG_DEBUGF(log, "set_current_omega_ric");
}

void CartesianBullseyeFrameProvider::clear_current_omega() noexcept {
    static auto log = bullseye::logging::get(bullseye::logname::kCoreFrameProviderCartesian);

    current_.has_omega = false;
    current_.omega_ric = Vec3{};
    LOG_DEBUGF(log, "clear_current_omega");
}

void CartesianBullseyeFrameProvider::add_sample(double t,
                                                const Vec3& origin_i,
                                                const Mat3& C_from_ric_to_inertial) {
    static auto log = bullseye::logging::get(bullseye::logname::kCoreFrameProviderCartesian);

    samples_.push_back(Sample{t, origin_i, C_from_ric_to_inertial, false, Vec3{}});
    sorted_ = false;

    LOG_DEBUGF(log, "add_sample: t=%.17g count=%zu", t, samples_.size());
}

void CartesianBullseyeFrameProvider::set_last_sample_omega_ric(const Vec3& omega_ric) {
    static auto log = bullseye::logging::get(bullseye::logname::kCoreFrameProviderCartesian);

    if (samples_.empty()) {
        // Configuration mistake; treat as invalid input at use time, but log here to aid debugging.
        LOG_WARNF(log, "set_last_sample_omega_ric: no samples present");
        return;
    }
    samples_.back().has_omega = true;
    samples_.back().omega_ric = omega_ric;
    LOG_DEBUGF(log, "set_last_sample_omega_ric");
}

void CartesianBullseyeFrameProvider::clear_samples() noexcept {
    static auto log = bullseye::logging::get(bullseye::logname::kCoreFrameProviderCartesian);

    samples_.clear();
    samples_.shrink_to_fit();  // configuration-time
    sorted_ = true;

    LOG_DEBUGF(log, "clear_samples");
}

void CartesianBullseyeFrameProvider::ensure_sorted_() noexcept {
    if (sorted_) {
        return;
    }
    std::sort(samples_.begin(), samples_.end(),
            [](const Sample& a, const Sample& b) { return a.t < b.t; });
    sorted_ = true;
}

bool CartesianBullseyeFrameProvider::should_warn_time_missing_(double t0) noexcept {
    if (warn_period_sec_ <= 0.0) {
        return true;
    }
    if ((t0 - last_warn_t0_) >= warn_period_sec_) {
        last_warn_t0_ = t0;
        return true;
    }
    return false;
}

void CartesianBullseyeFrameProvider::log_invalid_input_once_(const char* why) noexcept {
    if (invalid_logged_) {
        return;
    }
    invalid_logged_ = true;
    static auto log = bullseye::logging::get(bullseye::logname::kCoreFrameProviderCartesian);
    LOG_ERRORF(log, "invalid configuration: %s", why);
}

AdoptedRicFrame CartesianBullseyeFrameProvider::get(double t0) noexcept {
    AdoptedRicFrame out{};
    out.frame_source_id = frame_source_id_;

    if (frame_source_id_ == nullptr) {
        out.status.code = ProviderCode::kInvalidInput;
        log_invalid_input_once_("frame_source_id is null");
        return out;
    }
    if (!std::isfinite(t0)) {
        out.status.code = ProviderCode::kInvalidInput;
        log_invalid_input_once_("t0 is not finite");
        return out;
    }

    // v1 declarations (provider-side): always declare the frame as Bullseye RIC with RIC axis order.
    out.frame_kind = FrameKind::kBullseyeRIC;
    out.axis_order = AxisOrder::kRIC;

    // ω declaration: if provided, we declare it as ω_RIC (Option B compatible).
    out.omega_coords = OmegaCoords::kUnspecified;  // set if has_omega

    if (mode_ == Mode::kCurrent) {
        if (!(current_.t == t0)) {
        out.status.code = ProviderCode::kTimeMissing;
        if (should_warn_time_missing_(t0)) {
            static auto log = bullseye::logging::get(bullseye::logname::kCoreFrameProviderCartesian);
            LOG_WARNF(log, "get: time missing (mode=current) t0=%.17g current_t=%.17g",
                  t0, current_.t);
        }
        return out;
        }

        out.time_tag = current_.t;
        out.origin_i = current_.origin_i;
        out.C_from_ric_to_inertial = current_.C_from_ric_to_inertial;

        out.has_omega = current_.has_omega;
        if (out.has_omega) {
        out.omega_ric = current_.omega_ric;
        out.omega_coords = OmegaCoords::kOmegaRIC;
        }

        out.status.code = ProviderCode::kOk;
        return out;
    }

    // TimeSeries mode.
    ensure_sorted_();

    auto it = std::lower_bound(samples_.begin(), samples_.end(), t0,
                             [](const Sample& s, double t) { return s.t < t; });

    if (it == samples_.end() || !(it->t == t0)) {
        out.status.code = ProviderCode::kTimeMissing;
        if (should_warn_time_missing_(t0)) {
        static auto log = bullseye::logging::get(bullseye::logname::kCoreFrameProviderCartesian);
        const double found_t = (it == samples_.end()) ? std::numeric_limits<double>::quiet_NaN()
                                                    : it->t;
        LOG_WARNF(log, "get: time missing (mode=timeseries) t0=%.17g next_sample_t=%.17g count=%zu",
                t0, found_t, samples_.size());
        }
        return out;
    }

    out.time_tag = it->t;
    out.origin_i = it->origin_i;
    out.C_from_ric_to_inertial = it->C_from_ric_to_inertial;

    out.has_omega = it->has_omega;
    if (out.has_omega) {
        out.omega_ric = it->omega_ric;
        out.omega_coords = OmegaCoords::kOmegaRIC;
    }

    out.status.code = ProviderCode::kOk;
    return out;
}

}  // namespace bullseye_pred
