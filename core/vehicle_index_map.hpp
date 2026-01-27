#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "core/constants.hpp"

namespace bullseye_pred
{

/**
 * @brief Fixed-capacity, deterministic mapping from VehicleId -> stable index [0..MAX_VEHICLES).
 *
 * @details
 * - Indices are assigned in insertion order and never change (until clear()).
 * - Lookup is O(N) linear search (N <= MAX_VEHICLES). This is acceptable for Sprint 1.
 *
 * @note
 * This container intentionally avoids hashing for determinism and simplicity.
 */
class VehicleIndexMap final
{
  public:
    using VehicleId = std::uint64_t;

    /// @param capacity Maximum number of vehicles supported by this map.
    static constexpr std::size_t capacity() noexcept
    {
        return MAX_VEHICLES;
    }

    VehicleIndexMap() = default;

    /// Remove all entries and reset size to zero.
    void clear() noexcept
    {
        size_ = 0;
    }

    /// @return number of registered vehicles.
    std::size_t size() const noexcept
    {
        return size_;
    }

    /// @return true if no vehicles are registered.
    bool empty() const noexcept
    {
        return size_ == 0;
    }

    /**
     * @brief Register a vehicle id if not present.
     * @return index if already present or newly registered; std::nullopt if capacity full.
     */
    std::optional<std::size_t> register_vehicle(VehicleId id) noexcept;

    /**
     * @brief Lookup the stable index for an id.
     * @return index if present, std::nullopt otherwise.
     */
    std::optional<std::size_t> index_of(VehicleId id) const noexcept;

    /// @return true if id is present.
    bool contains(VehicleId id) const noexcept
    {
        return index_of(id).has_value();
    }

    /**
     * @brief Get the id at a given stable index.
     * @return id if index < size(); std::nullopt otherwise.
     */
    std::optional<VehicleId> id_at(std::size_t index) const noexcept;

  private:
    std::array<VehicleId, MAX_VEHICLES> ids_{};
    std::size_t size_{0};
};

} // namespace bullseye_pred
