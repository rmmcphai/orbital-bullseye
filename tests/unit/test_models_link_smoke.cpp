// tests/unit/test_models_link_smoke.cpp

#include <catch2/catch_test_macros.hpp>

#include "models/model_hcw.hpp"
#include "models/relative_model.hpp"

using bullseye_pred::HcwParams;
using bullseye_pred::ModelCode;
using bullseye_pred::ModelHCW;
using bullseye_pred::RelStateRic;
using bullseye_pred::Span;
using bullseye_pred::TimeGrid;
using bullseye_pred::Vec3;

TEST_CASE("smoke: orbital_bullseye_models links into unit tests (ModelHCW vtable)", "[link][models]")
{
    ModelHCW model;

    RelStateRic x0{};
    x0.r_ric = Vec3{0.0, 0.0, 0.0};
    x0.v_ric = Vec3{0.0, 0.0, 0.0};

    HcwParams p{};
    p.n_radps = 0.001;

    TimeGrid g;
    g.tau = {}; // empty grid should be kOk with 0 steps

    Vec3 out_r[1]{};
    const auto res = model.predict_hcw(x0, p, g, Span<Vec3>{out_r, 1}, Span<Vec3>{nullptr, 0});

    REQUIRE(res.code == ModelCode::kOk);
    REQUIRE(res.steps_written == 0);
}
