#include <catch2/catch_test_macros.hpp>

#include "core/logging.hpp"
#include "logger/level.hpp"
#include "logger/log_macros.hpp"

TEST_CASE("logging init and get logger")
{
    bullseye_pred::logging::Config cfg;
    cfg.level = sim_logger::Level::Debug;
    cfg.immediate_flush = true;
    cfg.file_path.clear();

    bullseye_pred::logging::init(cfg);

    auto log = bullseye_pred::logging::get("tests.logging");
    REQUIRE(log != nullptr);

    LOG_INFO(log, std::string("logging_smoke"));
}
