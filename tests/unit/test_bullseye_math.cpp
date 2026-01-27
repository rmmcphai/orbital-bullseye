// tests/unit/test_bullseye_math.cpp

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "core/bullseye_frame_math.hpp"
#include "core/types.hpp"

using bullseye_pred::ChiefState;
using bullseye_pred::Mat3;
using bullseye_pred::ProviderCode;
using bullseye_pred::Vec3;
using bullseye_pred::construct_ric_from_chief;
using bullseye_pred::cross;
using bullseye_pred::dot;
using bullseye_pred::norm;

static void require_orthonormal_right_handed(const Mat3& C_r2i)
{
    const Vec3 eR{C_r2i(0, 0), C_r2i(1, 0), C_r2i(2, 0)};
    const Vec3 eI{C_r2i(0, 1), C_r2i(1, 1), C_r2i(2, 1)};
    const Vec3 eC{C_r2i(0, 2), C_r2i(1, 2), C_r2i(2, 2)};

    REQUIRE(norm(eR) == Catch::Approx(1.0).epsilon(1e-12));
    REQUIRE(norm(eI) == Catch::Approx(1.0).epsilon(1e-12));
    REQUIRE(norm(eC) == Catch::Approx(1.0).epsilon(1e-12));

    REQUIRE(dot(eR, eI) == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(dot(eR, eC) == Catch::Approx(0.0).margin(1e-12));
    REQUIRE(dot(eI, eC) == Catch::Approx(0.0).margin(1e-12));

    // Right-handedness: R x I points along +C.
    const Vec3 RxI = cross(eR, eI);
    REQUIRE(dot(RxI, eC) == Catch::Approx(1.0).epsilon(1e-12));
}

TEST_CASE("Constructed RIC: expected triad for simple circular case", "[bullseye_math]")
{
    const double t0 = 42.0;

    const Vec3 r_i{7000e3, 0.0, 0.0};
    const Vec3 v_i{0.0, 7500.0, 0.0};

    ChiefState chief{};
    chief.time_tag = t0;
    chief.r_i = r_i;
    chief.v_i = v_i;
    chief.frame_id = "INERTIAL";
    chief.status.code = ProviderCode::kOk;

    const auto f = construct_ric_from_chief(chief);
    REQUIRE(f.status.code == ProviderCode::kOk);

    const Mat3& C = f.C_from_ric_to_inertial;
    require_orthonormal_right_handed(C);

    // In this geometry, R=[+x], I=[+y], C=[+z] -> identity.
    REQUIRE(C(0, 0) == Catch::Approx(1.0));
    REQUIRE(C(1, 0) == Catch::Approx(0.0));
    REQUIRE(C(2, 0) == Catch::Approx(0.0));

    REQUIRE(C(0, 1) == Catch::Approx(0.0));
    REQUIRE(C(1, 1) == Catch::Approx(1.0));
    REQUIRE(C(2, 1) == Catch::Approx(0.0));

    REQUIRE(C(0, 2) == Catch::Approx(0.0));
    REQUIRE(C(1, 2) == Catch::Approx(0.0));
    REQUIRE(C(2, 2) == Catch::Approx(1.0));

    const double omega_expected = 7500.0 / 7000e3;
    REQUIRE(f.has_omega);
    REQUIRE(f.omega_ric.x == Catch::Approx(0.0));
    REQUIRE(f.omega_ric.y == Catch::Approx(0.0));
    REQUIRE(f.omega_ric.z == Catch::Approx(omega_expected).epsilon(1e-12));
}

TEST_CASE("Constructed RIC: in-track aligns with transverse direction (h x r)", "[bullseye_math]")
{
    const double t0 = 1.0;

    // Deliberately include a substantial radial component to mimic high-eccentricity geometry.
    // r along +x, v has radial (+x) and transverse (+y) components.
    const Vec3 r_i{8000e3, 0.0, 0.0};
    const Vec3 v_i{1200.0, 6500.0, 0.0};

    ChiefState chief{};
    chief.time_tag = t0;
    chief.r_i = r_i;
    chief.v_i = v_i;
    chief.frame_id = "INERTIAL";
    chief.status.code = ProviderCode::kOk;

    const auto f = construct_ric_from_chief(chief);
    REQUIRE(f.status.code == ProviderCode::kOk);

    const Mat3& C = f.C_from_ric_to_inertial;
    require_orthonormal_right_handed(C);

    const Vec3 eI{C(0, 1), C(1, 1), C(2, 1)};

    // Expected transverse direction: t = h x r (proportional to v_perp).
    const Vec3 h = cross(r_i, v_i);
    const Vec3 t = cross(h, r_i);
    const double t_norm = norm(t);
    REQUIRE(t_norm > 0.0);

    const Vec3 eI_expected = (1.0 / t_norm) * t;

    // Align up to sign? In this construction we enforce right-handedness, so it should be +1.
    REQUIRE(dot(eI, eI_expected) == Catch::Approx(1.0).epsilon(1e-12));
}
