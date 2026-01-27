// tests/unit/test_transforms.cpp

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "core/bullseye_frame_math.hpp"
#include "core/frame_transforms.hpp"

using bullseye_pred::ChiefState;
using bullseye_pred::Mat3;
using bullseye_pred::ProviderCode;
using bullseye_pred::RelState;
using bullseye_pred::Vec3;
using bullseye_pred::construct_ric_from_chief;
using bullseye_pred::inertial_to_ric_relative;
using bullseye_pred::ric_to_inertial_relative;
using bullseye_pred::transpose;

TEST_CASE("Inertial<->RIC round-trip (Option B velocity)", "[transforms]")
{
    const double t0 = 0.0;
    ChiefState chief{};
    chief.time_tag = t0;
    chief.r_i = Vec3{7000e3, 0.0, 0.0};
    chief.v_i = Vec3{0.0, 7500.0, 0.0};
    chief.frame_id = "INERTIAL";
    chief.status.code = ProviderCode::kOk;

    const auto f = construct_ric_from_chief(chief);
    REQUIRE(f.status.code == ProviderCode::kOk);

    const Mat3 C_r2i = f.C_from_ric_to_inertial;
    const Mat3 C_i2r = transpose(C_r2i);

    const Vec3 dep_r_i = chief.r_i + Vec3{10.0, -20.0, 5.0};
    const Vec3 dep_v_i = chief.v_i + Vec3{0.01, -0.02, 0.005};

    const RelState rel =
        inertial_to_ric_relative(dep_r_i, dep_v_i, chief.r_i, chief.v_i, C_i2r, f.omega_ric);
    const RelState back =
        ric_to_inertial_relative(rel.r, rel.v, chief.r_i, chief.v_i, C_r2i, f.omega_ric);

    REQUIRE(back.r.x == Catch::Approx(dep_r_i.x).epsilon(1e-12));
    REQUIRE(back.r.y == Catch::Approx(dep_r_i.y).epsilon(1e-12));
    REQUIRE(back.r.z == Catch::Approx(dep_r_i.z).epsilon(1e-12));

    REQUIRE(back.v.x == Catch::Approx(dep_v_i.x).epsilon(1e-12));
    REQUIRE(back.v.y == Catch::Approx(dep_v_i.y).epsilon(1e-12));
    REQUIRE(back.v.z == Catch::Approx(dep_v_i.z).epsilon(1e-12));
}
