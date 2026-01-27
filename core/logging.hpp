#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "logger/level.hpp"

namespace sim_logger
{
class Logger;
} // namespace sim_logger

namespace bullseye_pred::logging
{

/**
 * @brief Logging configuration for Bullseye.
 *
 * @details
 * This is intentionally small and stable. You can extend later (env config, JSON, etc.)
 * without touching call sites.
 */
struct Config
{
    /// Default log level applied to the root logger.
    sim_logger::Level level;

    /// Whether to flush immediately after each record (useful during development).
    bool immediate_flush{false};

    /// If non-empty, also log to this file (in addition to console).
    std::string file_path{};

    /// Log line format (sim-logger PatternFormatter syntax).
    std::string pattern{"{met} {level} {logger} {msg}"};
};

/**
 * @brief Initialize the Bullseye logging tree.
 *
 * @details
 * Creates/configures the root logger and sinks (console, optional file).
 * Safe to call multiple times; last call wins.
 */
void init(const Config& cfg);

/**
 * @brief Get (or create) a component logger under the Bullseye root.
 *
 * @param component Dotted suffix (e.g. "core.vehicle_index_map").
 * @return Shared logger instance.
 */
std::shared_ptr<sim_logger::Logger> get(std::string_view component);

} // namespace bullseye_pred::logging
