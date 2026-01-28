#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "core/types.hpp"
#include "core/constants.hpp"

namespace bullseye_pred
{

struct PredictionBuffer final
{
    std::uint64_t seqno{0};
    double t0{0.0};
    std::array<std::array<Vec3, MAX_STEPS>, MAX_VEHICLES> positions{};
    bool write_positions(std::size_t count,
                         const Vec3* r_ric_in) noexcept;
    void reset(double t0_in) noexcept;
};

} // namespace bullseye_pred
