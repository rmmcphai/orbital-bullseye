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
};

} // namespace bullseye_pred
