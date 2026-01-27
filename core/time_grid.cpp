#include "core/time_grid.hpp"

#include "core/logging.hpp"
#include "core/log_names.hpp"
#include "logger/log_macros.hpp"

#include <cmath>   // std::floor
#include <cstddef> // std::size_t

namespace bullseye_pred
{

TimeGrid make_time_grid(double horizon_sec, double cadence_sec)
{
    static auto log = bullseye_pred::logging::get(bullseye_pred::logname::kCoreTimeGrid);

    TimeGrid grid{};

    // Invalid inputs => empty schedule.
    if (horizon_sec < 0.0 || cadence_sec <= 0.0)
    {
        LOG_WARNF(log, "invalid_inputs horizon=%.17g cadence=%.17g", horizon_sec, cadence_sec);
        return grid;
    }

    const double k_max_d = std::floor(horizon_sec / cadence_sec);
    const std::size_t k_max = (k_max_d > 0.0) ? static_cast<std::size_t>(k_max_d) : 0u;

    grid.tau.reserve(k_max + 1u);
    for (std::size_t k = 0; k <= k_max; ++k)
    {
        grid.tau.push_back(static_cast<double>(k) * cadence_sec);
    }

    // DEBUG-only: useful while bringing up predictors; typically disable at runtime.
    LOG_DEBUGF(log, "grid horizon=%.17g cadence=%.17g steps=%zu last=%.17g", horizon_sec,
               cadence_sec, grid.tau.size(), grid.tau.empty() ? 0.0 : grid.tau.back());

    return grid;
}

} // namespace bullseye_pred
