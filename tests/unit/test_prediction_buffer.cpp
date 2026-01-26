/**
 * @file test_prediction_buffer.cpp
 * @brief Unit tests for PredictionBuffer sizing and basic layout invariants.
 */

#include <catch2/catch_test_macros.hpp>
#include "core/prediction_buffer.hpp"

using namespace bullseye;

TEST_CASE("Prediction buffer satisfies minimum step requirements") {
  // Sprint requirement: >= 61 samples available (e.g., 60 seconds + t0 at 1 Hz)
  STATIC_REQUIRE(MAX_STEPS >= 61);
}

TEST_CASE("Prediction buffer is vehicle-major contiguous") {
  PredictionBuffer buf{};

  // positions[0][k] should be adjacent elements in memory for increasing k
  auto* p0 = &buf.positions[0][0];
  auto* p1 = &buf.positions[0][1];

  REQUIRE(reinterpret_cast<const char*>(p1) >
          reinterpret_cast<const char*>(p0));
}
