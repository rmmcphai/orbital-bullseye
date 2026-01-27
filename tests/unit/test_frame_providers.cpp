// tests/unit/test_frame_providers.cpp  (new file)

#include <catch2/catch_test_macros.hpp>

#include "core/frame_provider_cartesian.hpp"

using bullseye_pred::AdoptedRicFrame;
using bullseye_pred::AxisOrder;
using bullseye_pred::CartesianBullseyeFrameProvider;
using bullseye_pred::FrameKind;
using bullseye_pred::Mat3;
using bullseye_pred::OmegaCoords;
using bullseye_pred::ProviderCode;
using bullseye_pred::Vec3;

TEST_CASE("CartesianBullseyeFrameProvider (current) returns exact-time frame (FR-14a)",
          "[frame_providers]")
{
    constexpr const char* kSrc = "USER_CARTESIAN_FRAME";
    CartesianBullseyeFrameProvider p{kSrc, CartesianBullseyeFrameProvider::Mode::kCurrent, 0.0};

    const double t0 = 100.0;
    const Mat3 C = Mat3::identity();
    p.set_current(t0, Vec3{1.0, 2.0, 3.0}, C);
    p.set_current_omega_ric(Vec3{0.1, 0.2, 0.3});

    const AdoptedRicFrame f = p.get(t0);
    REQUIRE(f.status.code == ProviderCode::kOk);
    REQUIRE(f.time_tag == t0);
    REQUIRE(f.frame_source_id == kSrc);

    REQUIRE(f.frame_kind == FrameKind::kBullseyeRIC);
    REQUIRE(f.axis_order == AxisOrder::kRIC);

    REQUIRE(f.origin_i.x == 1.0);
    REQUIRE(f.origin_i.y == 2.0);
    REQUIRE(f.origin_i.z == 3.0);

    REQUIRE(f.C_from_ric_to_inertial(0, 0) == 1.0);
    REQUIRE(f.C_from_ric_to_inertial(1, 1) == 1.0);
    REQUIRE(f.C_from_ric_to_inertial(2, 2) == 1.0);

    REQUIRE(f.has_omega == true);
    REQUIRE(f.omega_coords == OmegaCoords::kOmegaRIC);
    REQUIRE(f.omega_ric.x == 0.1);
    REQUIRE(f.omega_ric.y == 0.2);
    REQUIRE(f.omega_ric.z == 0.3);
}

TEST_CASE("CartesianBullseyeFrameProvider (current) fails fast if time missing (FR-14a)",
          "[frame_providers]")
{
    constexpr const char* kSrc = "USER_CARTESIAN_FRAME";
    CartesianBullseyeFrameProvider p{kSrc, CartesianBullseyeFrameProvider::Mode::kCurrent, 0.0};

    p.set_current(10.0, Vec3{0.0, 0.0, 0.0}, Mat3::identity());

    const AdoptedRicFrame f = p.get(11.0);
    REQUIRE(f.status.code == ProviderCode::kTimeMissing);
    REQUIRE(f.frame_source_id == kSrc);
    REQUIRE(f.frame_kind == FrameKind::kBullseyeRIC);
    REQUIRE(f.axis_order == AxisOrder::kRIC);
}

TEST_CASE("CartesianBullseyeFrameProvider (timeseries) returns exact-time frame (FR-14a)",
          "[frame_providers]")
{
    constexpr const char* kSrc = "USER_CARTESIAN_FRAME";
    CartesianBullseyeFrameProvider p{kSrc, CartesianBullseyeFrameProvider::Mode::kTimeSeries, 0.0};

    // Add out of order to prove deterministic sort + lookup.
    p.add_sample(2.0, Vec3{2.0, 0.0, 0.0}, Mat3::identity());
    p.add_sample(1.0, Vec3{1.0, 0.0, 0.0}, Mat3::identity());
    p.add_sample(3.0, Vec3{3.0, 0.0, 0.0}, Mat3::identity());
    p.set_last_sample_omega_ric(Vec3{0.0, 0.0, 0.01}); // applies to t=3.0 sample

    const AdoptedRicFrame f = p.get(3.0);
    REQUIRE(f.status.code == ProviderCode::kOk);
    REQUIRE(f.time_tag == 3.0);
    REQUIRE(f.origin_i.x == 3.0);
    REQUIRE(f.has_omega == true);
    REQUIRE(f.omega_coords == OmegaCoords::kOmegaRIC);
}

TEST_CASE("CartesianBullseyeFrameProvider (timeseries) fails fast if time missing (FR-14a)",
          "[frame_providers]")
{
    constexpr const char* kSrc = "USER_CARTESIAN_FRAME";
    CartesianBullseyeFrameProvider p{kSrc, CartesianBullseyeFrameProvider::Mode::kTimeSeries, 0.0};

    p.add_sample(1.0, Vec3{1.0, 0.0, 0.0}, Mat3::identity());
    p.add_sample(2.0, Vec3{2.0, 0.0, 0.0}, Mat3::identity());

    const AdoptedRicFrame f = p.get(1.5);
    REQUIRE(f.status.code == ProviderCode::kTimeMissing);
}

TEST_CASE("CartesianBullseyeFrameProvider returns kInvalidInput when frame_source_id is null",
          "[frame_providers]")
{
    CartesianBullseyeFrameProvider p{nullptr, CartesianBullseyeFrameProvider::Mode::kCurrent, 0.0};
    const AdoptedRicFrame f = p.get(0.0);
    REQUIRE(f.status.code == ProviderCode::kInvalidInput);
    REQUIRE(f.frame_source_id == nullptr);
}
