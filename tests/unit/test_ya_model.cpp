// tests/unit/test_ya_model.cpp

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "models/model_hcw.hpp"
#include "models/model_ya_stm.hpp"

using bullseye_pred::HcwParams;
using bullseye_pred::ModelCode;
using bullseye_pred::ModelHCW;
using bullseye_pred::ModelYA_STM;
using bullseye_pred::RelStateRic;
using bullseye_pred::Span;
using bullseye_pred::TimeGrid;
using bullseye_pred::Vec3;
using bullseye_pred::YaStmParams;

TEST_CASE("YA/TH: zero initial state remains zero", "[ya]")
{
    ModelYA_STM m;

    RelStateRic x0{};
    x0.r_ric = Vec3{0.0, 0.0, 0.0};
    x0.v_ric = Vec3{0.0, 0.0, 0.0};

    // Circular chief (values irrelevant for the zero-state invariance, but must be valid).
    const double mu = 3.986004418e14;
    const double r0 = 7000e3;
    const double v0 = std::sqrt(mu / r0);

    YaStmParams p{};
    p.mu = mu;
    p.chief_r0_i = Vec3{r0, 0.0, 0.0};
    p.chief_v0_i = Vec3{0.0, v0, 0.0};
    p.max_dt_sec = 0.1;

    TimeGrid g;
    g.tau = {0.0, 1.0, 10.0, 60.0};

    Vec3 r_out[4]{};
    Vec3 v_out[4]{};

    const auto res = m.predict_ya_stm(x0, p, g, Span<Vec3>{r_out, 4}, Span<Vec3>{v_out, 4});
    REQUIRE(res.code == ModelCode::kOk);
    REQUIRE(res.steps_written == 4);

    for (int i = 0; i < 4; ++i)
    {
        REQUIRE(r_out[i].x == Catch::Approx(0.0));
        REQUIRE(r_out[i].y == Catch::Approx(0.0));
        REQUIRE(r_out[i].z == Catch::Approx(0.0));
        REQUIRE(v_out[i].x == Catch::Approx(0.0));
        REQUIRE(v_out[i].y == Catch::Approx(0.0));
        REQUIRE(v_out[i].z == Catch::Approx(0.0));
    }
}

TEST_CASE("YA/TH: reduces to HCW for circular chief (numerical RK4)", "[ya][hcw]")
{
    // Chief: circular orbit in inertial XY plane.
    const double mu = 3.986004418e14;
    const double r0 = 7000e3;
    const double v0 = std::sqrt(mu / r0);

    const double n = std::sqrt(mu / (r0 * r0 * r0));

    // Relative initial condition in RIC.
    RelStateRic x0{};
    x0.r_ric = Vec3{100.0, -50.0, 25.0};
    x0.v_ric = Vec3{0.10, -0.20, 0.05};

    TimeGrid g;
    g.tau = {0.0, 5.0, 10.0, 30.0, 60.0};

    Vec3 r_hcw[5]{};
    Vec3 v_hcw[5]{};
    Vec3 r_ya[5]{};
    Vec3 v_ya[5]{};

    // HCW closed-form.
    ModelHCW hcw;
    HcwParams hp{};
    hp.n_radps = n;
    const auto rh = hcw.predict_hcw(x0, hp, g, Span<Vec3>{r_hcw, 5}, Span<Vec3>{v_hcw, 5});
    REQUIRE(rh.code == ModelCode::kOk);
    REQUIRE(rh.steps_written == 5);

    // YA/TH (RK4). Use small step to make this comparison tight but robust.
    ModelYA_STM ya;
    YaStmParams yp{};
    yp.mu = mu;
    yp.chief_r0_i = Vec3{r0, 0.0, 0.0};
    yp.chief_v0_i = Vec3{0.0, v0, 0.0};
    yp.max_dt_sec = 0.02;

    const auto ry = ya.predict_ya_stm(x0, yp, g, Span<Vec3>{r_ya, 5}, Span<Vec3>{v_ya, 5});
    REQUIRE(ry.code == ModelCode::kOk);
    REQUIRE(ry.steps_written == 5);

    // RK4 error should be small for this small horizon and step size.
    // Use absolute tolerances appropriate for meter-scale relative motion.
    const double pos_tol_m = 5e-3;  // 5 mm
    const double vel_tol_mps = 5e-6; // 5 um/s

    for (int i = 0; i < 5; ++i)
    {
        REQUIRE(r_ya[i].x == Catch::Approx(r_hcw[i].x).margin(pos_tol_m));
        REQUIRE(r_ya[i].y == Catch::Approx(r_hcw[i].y).margin(pos_tol_m));
        REQUIRE(r_ya[i].z == Catch::Approx(r_hcw[i].z).margin(pos_tol_m));

        REQUIRE(v_ya[i].x == Catch::Approx(v_hcw[i].x).margin(vel_tol_mps));
        REQUIRE(v_ya[i].y == Catch::Approx(v_hcw[i].y).margin(vel_tol_mps));
        REQUIRE(v_ya[i].z == Catch::Approx(v_hcw[i].z).margin(vel_tol_mps));
    }
}
