#include "core/vehicle_index_map.hpp"
#include "core/logging.hpp"
#include "logger/log_macros.hpp"
#include "core/log_names.hpp"

namespace bullseye_pred
{

std::optional<std::size_t> VehicleIndexMap::index_of(VehicleId id) const noexcept
{
    for (std::size_t i = 0; i < size_; ++i)
    {
        if (ids_[i] == id)
        {
            return i;
        }
    }
    return std::nullopt;
}

std::optional<std::size_t> VehicleIndexMap::register_vehicle(VehicleId id) noexcept
{
    static auto log = bullseye_pred::logging::get(bullseye_pred::logname::kCoreVehicleIndexMap);

    if (auto idx = index_of(id))
    {
        LOG_DEBUGF(log, "duplicate_register id=%llu idx=%zu", static_cast<unsigned long long>(id),
                   *idx);
        return idx;
    }

    if (size_ >= MAX_VEHICLES)
    {
        LOG_WARNF(log, "capacity_reject id=%llu size=%zu cap=%zu",
                  static_cast<unsigned long long>(id), size_, MAX_VEHICLES);
        return std::nullopt;
    }

    ids_[size_] = id;
    const std::size_t assigned = size_;
    ++size_;

    LOG_INFOF(log, "register id=%llu idx=%zu size=%zu", static_cast<unsigned long long>(id),
              assigned, size_);

    return assigned;
}

std::optional<VehicleIndexMap::VehicleId> VehicleIndexMap::id_at(std::size_t index) const noexcept
{
    if (index >= size_)
    {
        return std::nullopt;
    }
    return ids_[index];
}

} // namespace bullseye_pred
