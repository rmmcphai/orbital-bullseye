#pragma once
/**
 * @file constants.hpp
 * @brief Single-source constants and types not enforcing v1 contracts.
 */

#include <cstddef>

namespace bullseye_pred {

// Capacity / sizing constants (v1 fixed-size design).
inline constexpr std::size_t MAX_VEHICLES = 32;
inline constexpr std::size_t MAX_STEPS = 600;

}  // namespace bullseye_pred
