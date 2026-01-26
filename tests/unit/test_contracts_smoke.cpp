// tests/unit/test_contracts_smoke.cpp (raw)

#include <type_traits>

#include <catch2/catch_test_macros.hpp>

// sim-logger (required)
#include <logger/level.hpp>
#include <logger/logger.hpp>
#include <logger/log_record.hpp>

#include "core/contracts.hpp"

namespace c = bullseye_pred::contracts;

TEST_CASE("smoke: contracts header compiles and key symbols exist", "[smoke][contracts]") {
  STATIC_REQUIRE(std::is_same_v<decltype(c::Frames::kInertialFrameId), const char* const>);
  STATIC_REQUIRE(std::is_enum_v<c::TimePolicy::OnTimeMismatch>);

  REQUIRE(c::Frames::kInertialFrameId != nullptr);
  REQUIRE(c::Det::kKeplerIters > 0);
}

TEST_CASE("smoke: sim-logger headers compile and core types are visible", "[smoke][logging]") {
  // No construction required; just verify the API surface is present.
  STATIC_REQUIRE(std::is_enum_v<sim_logger::Level>);
  (void)sim_logger::Level::Info;  // adjust if your enum uses different spelling

  // Type visibility (no instantiation)
  STATIC_REQUIRE(std::is_class_v<sim_logger::Logger>);
  STATIC_REQUIRE(std::is_class_v<sim_logger::LogRecord>);

  // Link-time proof happens via target_link_libraries(... sim_logger::sim_logger)
  SUCCEED();
}
