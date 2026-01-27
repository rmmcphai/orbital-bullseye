// tests/unit/test_providers.cpp

#include <cstdint>
#include <cstring>

#include <catch2/catch_test_macros.hpp>

#include "core/provider_cartesian.hpp"
#include "core/provider_twobody.hpp"

namespace bullseye_pred
{
namespace
{

static bool bitwise_equal(const ChiefState& a, const ChiefState& b)
{
    // Compare only the fields that are meaningful for determinism here.
    // (Pointers like frame_id are compared by pointer value, which is stable in this test.)
    return (a.time_tag == b.time_tag) &&
           (a.r_i.x == b.r_i.x && a.r_i.y == b.r_i.y && a.r_i.z == b.r_i.z) &&
           (a.v_i.x == b.v_i.x && a.v_i.y == b.v_i.y && a.v_i.z == b.v_i.z) &&
           (a.frame_id == b.frame_id) && (a.status.code == b.status.code);
}

} // namespace
} // namespace bullseye_pred

using bullseye_pred::CartesianChiefProvider;
using bullseye_pred::ChiefState;
using bullseye_pred::ProviderCode;
using bullseye_pred::TwoBodyChiefProvider;
using bullseye_pred::Vec3;

TEST_CASE("CartesianChiefProvider (current) returns exact-time sample (FR-14)", "[providers]")
{
    constexpr const char* kFrameId = "INERTIAL";
    CartesianChiefProvider p{kFrameId, CartesianChiefProvider::Mode::kCurrent,
                             /*warn_period_sec=*/0.0};

    const double t0 = 100.0;
    p.set_current(t0, Vec3{1.0, 2.0, 3.0}, Vec3{4.0, 5.0, 6.0});

    const ChiefState s = p.get(t0);
    REQUIRE(s.status.code == ProviderCode::kOk);
    REQUIRE(s.time_tag == t0);
    REQUIRE(s.frame_id == kFrameId);
    REQUIRE(s.r_i.x == 1.0);
    REQUIRE(s.r_i.y == 2.0);
    REQUIRE(s.r_i.z == 3.0);
    REQUIRE(s.v_i.x == 4.0);
    REQUIRE(s.v_i.y == 5.0);
    REQUIRE(s.v_i.z == 6.0);
}

TEST_CASE("CartesianChiefProvider (current) fails fast if time missing (FR-14)", "[providers]")
{
    constexpr const char* kFrameId = "INERTIAL";
    CartesianChiefProvider p{kFrameId, CartesianChiefProvider::Mode::kCurrent,
                             /*warn_period_sec=*/0.0};

    const double t_set = 10.0;
    const double t_req = 11.0;
    p.set_current(t_set, Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0});

    const ChiefState s = p.get(t_req);
    REQUIRE(s.status.code == ProviderCode::kTimeMissing);
    REQUIRE(s.frame_id == kFrameId);
}

TEST_CASE("CartesianChiefProvider (timeseries) returns exact-time sample (FR-14)", "[providers]")
{
    constexpr const char* kFrameId = "INERTIAL";
    CartesianChiefProvider p{kFrameId, CartesianChiefProvider::Mode::kTimeSeries,
                             /*warn_period_sec=*/0.0};

    // Add out of order to prove deterministic sort + lookup.
    p.add_sample(2.0, Vec3{2.0, 0.0, 0.0}, Vec3{0.0, 2.0, 0.0});
    p.add_sample(1.0, Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0});
    p.add_sample(3.0, Vec3{3.0, 0.0, 0.0}, Vec3{0.0, 3.0, 0.0});

    const ChiefState s = p.get(2.0);
    REQUIRE(s.status.code == ProviderCode::kOk);
    REQUIRE(s.time_tag == 2.0);
    REQUIRE(s.frame_id == kFrameId);
    REQUIRE(s.r_i.x == 2.0);
    REQUIRE(s.v_i.y == 2.0);
}

TEST_CASE("CartesianChiefProvider (timeseries) fails fast if time missing (FR-14)", "[providers]")
{
    constexpr const char* kFrameId = "INERTIAL";
    CartesianChiefProvider p{kFrameId, CartesianChiefProvider::Mode::kTimeSeries,
                             /*warn_period_sec=*/0.0};

    p.add_sample(1.0, Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0});
    p.add_sample(2.0, Vec3{2.0, 0.0, 0.0}, Vec3{0.0, 2.0, 0.0});

    const ChiefState s = p.get(1.5);
    REQUIRE(s.status.code == ProviderCode::kTimeMissing);
    REQUIRE(s.frame_id == kFrameId);
}

TEST_CASE("CartesianChiefProvider is deterministic for identical inputs",
          "[providers][determinism]")
{
    constexpr const char* kFrameId = "INERTIAL";
    CartesianChiefProvider p{kFrameId, CartesianChiefProvider::Mode::kTimeSeries,
                             /*warn_period_sec=*/0.0};

    p.add_sample(5.0, Vec3{5.0, 6.0, 7.0}, Vec3{8.0, 9.0, 10.0});
    p.add_sample(6.0, Vec3{15.0, 16.0, 17.0}, Vec3{18.0, 19.0, 110.0});

    const ChiefState a = p.get(5.0);
    const ChiefState b = p.get(5.0);

    REQUIRE(a.status.code == ProviderCode::kOk);
    REQUIRE(b.status.code == ProviderCode::kOk);
    REQUIRE(bullseye_pred::bitwise_equal(a, b));
}

TEST_CASE("CartesianChiefProvider returns kInvalidInput when inertial frame id is null",
          "[providers]")
{
    CartesianChiefProvider p{nullptr, CartesianChiefProvider::Mode::kCurrent,
                             /*warn_period_sec=*/0.0};
    const ChiefState s = p.get(0.0);
    REQUIRE(s.status.code == ProviderCode::kInvalidInput);
    REQUIRE(s.frame_id == nullptr);
}

static bool finite3(const Vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

TEST_CASE("TwoBodyChiefProvider returns deterministic state for same t0",
          "[providers][twobody][determinism]")
{
    // Earth mu (SI).
    const double mu = 3.986004418e14;

    // Simple LEO-ish state.
    const Vec3 r0{7000e3, 0.0, 0.0};
    const Vec3 v0{0.0, 7546.05329, 0.0};

    constexpr const char* kFrameId = "INERTIAL";
    TwoBodyChiefProvider p{kFrameId, mu, /*t_epoch=*/0.0, r0, v0};

    const double t0_req = 1234.5;

    const ChiefState a = p.get(t0_req);
    const ChiefState b = p.get(t0_req);

    REQUIRE(a.status.code == ProviderCode::kOk);
    REQUIRE(b.status.code == ProviderCode::kOk);
    REQUIRE(a.time_tag == t0_req);
    REQUIRE(b.time_tag == t0_req);
    REQUIRE(a.frame_id == kFrameId);
    REQUIRE(b.frame_id == kFrameId);

    // Bitwise-ish checks (exact equality) are acceptable here because we run the same code path
    // twice.
    REQUIRE(a.r_i.x == b.r_i.x);
    REQUIRE(a.r_i.y == b.r_i.y);
    REQUIRE(a.r_i.z == b.r_i.z);
    REQUIRE(a.v_i.x == b.v_i.x);
    REQUIRE(a.v_i.y == b.v_i.y);
    REQUIRE(a.v_i.z == b.v_i.z);
}

TEST_CASE("TwoBodyChiefProvider produces finite outputs and varies with time",
          "[providers][twobody]")
{
    const double mu = 3.986004418e14;
    const Vec3 r0{7000e3, 0.0, 0.0};
    const Vec3 v0{0.0, 7546.05329, 0.0};

    constexpr const char* kFrameId = "INERTIAL";
    TwoBodyChiefProvider p{kFrameId, mu, /*t_epoch=*/0.0, r0, v0};

    const ChiefState s0 = p.get(0.0);
    const ChiefState s1 = p.get(10.0);

    REQUIRE(s0.status.code == ProviderCode::kOk);
    REQUIRE(s1.status.code == ProviderCode::kOk);
    REQUIRE(finite3(s0.r_i));
    REQUIRE(finite3(s0.v_i));
    REQUIRE(finite3(s1.r_i));
    REQUIRE(finite3(s1.v_i));

    // Not equal for dt != 0 (sanity only; no tolerance-based orbital check here).
    REQUIRE((s0.r_i.x != s1.r_i.x || s0.r_i.y != s1.r_i.y || s0.r_i.z != s1.r_i.z));
}

TEST_CASE("TwoBodyChiefProvider rejects invalid mu", "[providers][twobody]")
{
    const Vec3 r0{7000e3, 0.0, 0.0};
    const Vec3 v0{0.0, 7546.05329, 0.0};

    constexpr const char* kFrameId = "INERTIAL";
    TwoBodyChiefProvider p{kFrameId, /*mu=*/0.0, /*t_epoch=*/0.0, r0, v0};

    const ChiefState s = p.get(1.0);
    REQUIRE(s.status.code == ProviderCode::kInvalidInput);
}