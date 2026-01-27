// tests/unit/test_bullseye_frame.cpp

#include <catch2/catch_test_macros.hpp>

#include "core/bullseye_frame.hpp"

using bullseye_pred::AdoptedRicFrame;
using bullseye_pred::AxisOrder;
using bullseye_pred::BullseyeFrame;
using bullseye_pred::BullseyeFrameMode;
using bullseye_pred::ChiefState;
using bullseye_pred::FrameKind;
using bullseye_pred::IBullseyeFrameProvider;
using bullseye_pred::IChiefStateProvider;
using bullseye_pred::Mat3;
using bullseye_pred::OmegaCoords;
using bullseye_pred::ProviderCode;
using bullseye_pred::Vec3;

namespace
{

class FixedChief final : public IChiefStateProvider
{
  public:
    ChiefState state;
    [[nodiscard]] ChiefState get(double t0) noexcept override
    {
        state.time_tag = t0;
        return state;
    }
};

class FixedAdopted final : public IBullseyeFrameProvider
{
  public:
    AdoptedRicFrame frame;
    [[nodiscard]] AdoptedRicFrame get(double t0) noexcept override
    {
        frame.time_tag = t0;
        return frame;
    }
};

} // namespace

TEST_CASE("BullseyeFrame uses adopted when valid", "[bullseye_frame]")
{
    FixedChief chief;
    chief.state.r_i = Vec3{7000e3, 0.0, 0.0};
    chief.state.v_i = Vec3{0.0, 7500.0, 0.0};
    chief.state.frame_id = "INERTIAL";
    chief.state.status.code = ProviderCode::kOk;

    FixedAdopted adopted;
    adopted.frame.origin_i = chief.state.r_i;
    adopted.frame.C_from_ric_to_inertial = Mat3::identity();
    adopted.frame.has_omega = false;
    adopted.frame.omega_coords = OmegaCoords::kUnspecified;
    adopted.frame.frame_kind = FrameKind::kBullseyeRIC;
    adopted.frame.axis_order = AxisOrder::kRIC;
    adopted.frame.frame_source_id = "ADOPTED";
    adopted.frame.status.code = ProviderCode::kOk;

    BullseyeFrame bf(chief, &adopted, BullseyeFrameMode::kAdoptedPrefer);
    const auto snap = bf.update(10.0);

    REQUIRE(snap.status.code == ProviderCode::kOk);
    REQUIRE(snap.used_adopted);
    REQUIRE(snap.adopted_frame_source_id != nullptr);
}

TEST_CASE("BullseyeFrame falls back to constructed when adopted invalid", "[bullseye_frame]")
{
    FixedChief chief;
    chief.state.r_i = Vec3{7000e3, 0.0, 0.0};
    chief.state.v_i = Vec3{0.0, 7500.0, 0.0};
    chief.state.frame_id = "INERTIAL";
    chief.state.status.code = ProviderCode::kOk;

    FixedAdopted adopted;
    adopted.frame.origin_i = chief.state.r_i;
    adopted.frame.C_from_ric_to_inertial = Mat3::identity();
    adopted.frame.has_omega = false;
    adopted.frame.omega_coords = OmegaCoords::kUnspecified;
    adopted.frame.frame_kind = FrameKind::kBullseyeRIC;
    adopted.frame.axis_order = AxisOrder::kUnspecified; // invalid declaration
    adopted.frame.frame_source_id = "ADOPTED";
    adopted.frame.status.code = ProviderCode::kOk;

    BullseyeFrame bf(chief, &adopted, BullseyeFrameMode::kAdoptedPrefer);
    const auto snap = bf.update(10.0);

    REQUIRE(snap.status.code == ProviderCode::kOk);
    REQUIRE_FALSE(snap.used_adopted);
    REQUIRE(static_cast<std::uint32_t>(snap.degraded) != 0u);
}
