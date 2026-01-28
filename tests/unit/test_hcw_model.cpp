// tests/unit/test_hcw_model.cpp

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "models/model_hcw.hpp"

using bullseye_pred::HcwParams;
using bullseye_pred::ModelCode;
using bullseye_pred::ModelHCW;
using bullseye_pred::RelStateRic;
using bullseye_pred::Span;
using bullseye_pred::TimeGrid;
using bullseye_pred::Vec3;

TEST_CASE("HCW: zero initial state remains zero", "[hcw]")
{
    ModelHCW m;

    RelStateRic x0{};
    x0.r_ric = Vec3{0.0, 0.0, 0.0};
    x0.v_ric = Vec3{0.0, 0.0, 0.0};

    HcwParams p{};
    p.n_radps = 0.001;

    TimeGrid g;
    g.tau = {0.0, 1.0, 10.0, 60.0};

    Vec3 r_out[4]{};
    Vec3 v_out[4]{};

    const auto res = m.predict_hcw(x0, p, g, Span<Vec3>{r_out, 4}, Span<Vec3>{v_out, 4});
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

TEST_CASE("HCW: simple x0-only case matches closed-form at selected times", "[hcw]")
{
    ModelHCW m;

    const double n = 0.001; // rad/s

    RelStateRic x0{};
    x0.r_ric = Vec3{100.0, 0.0, 0.0};
    x0.v_ric = Vec3{0.0, 0.0, 0.0};

    HcwParams p{};
    p.n_radps = n;

    TimeGrid g;
    g.tau = {0.0, 10.0, 60.0};

    Vec3 r_out[3]{};
    Vec3 v_out[3]{};

    const auto res = m.predict_hcw(x0, p, g, Span<Vec3>{r_out, 3}, Span<Vec3>{v_out, 3});
    REQUIRE(res.code == ModelCode::kOk);
    REQUIRE(res.steps_written == 3);

    auto expect = [&](int k)
    {
        const double t = g.tau[static_cast<std::size_t>(k)];
        const double nt = n * t;
        const double s = std::sin(nt);
        const double c = std::cos(nt);

        const double x = (4.0 - 3.0 * c) * 100.0;
        const double y = 6.0 * (s - nt) * 100.0;
        const double z = 0.0;

        return Vec3{x, y, z};
    };

    for (int k = 0; k < 3; ++k)
    {
        const Vec3 e = expect(k);
        REQUIRE(r_out[k].x == Catch::Approx(e.x).epsilon(1e-12));
        REQUIRE(r_out[k].y == Catch::Approx(e.y).epsilon(1e-12));
        REQUIRE(r_out[k].z == Catch::Approx(e.z).epsilon(1e-12));
    }
}
