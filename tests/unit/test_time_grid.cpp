/**
 * @file test_time_grid.cpp
 * @brief Unit tests for time grid semantics (FR-15).
 */

#include <catch2/catch_test_macros.hpp>
#include "core/time_grid.hpp"

using namespace bullseye_pred;

TEST_CASE("Time grid includes t0 and respects horizon")
{
    // horizon=10, cadence=2 -> expected offsets: 0,2,4,6,8,10
    auto grid = make_time_grid(10.0, 2.0);

    REQUIRE(grid.tau.size() == 6);
    REQUIRE(grid.tau.front() == 0.0);
    REQUIRE(grid.tau.back() == 10.0);
}

TEST_CASE("Invalid cadence produces empty grid")
{
    // cadence must be > 0
    auto grid = make_time_grid(10.0, 0.0);
    REQUIRE(grid.tau.empty());
}

TEST_CASE("Time grid never exceeds horizon (tricky FP)")
{
    const double horizon = 1.0;
    const double cadence = 0.1;

    auto grid = make_time_grid(horizon, cadence);

    REQUIRE_FALSE(grid.tau.empty());
    REQUIRE(grid.tau.front() == 0.0);
    REQUIRE(grid.tau.back() <= horizon);
}
