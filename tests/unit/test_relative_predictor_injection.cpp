// tests/unit/test_relative_predictor_injection.cpp

#include <catch2/catch_test_macros.hpp>
#include <cstdint>

#include "core/relative_predictor.hpp"

#include "core/bullseye_frame.hpp"
#include "core/chief_state_provider.hpp"
#include "core/frame_transforms.hpp"
#include "core/publisher.hpp"
#include "core/time_grid.hpp"
#include "core/types.hpp"
#include "core/vehicle_index_map.hpp"
#include "models/relative_model.hpp"

using namespace bullseye_pred;

namespace
{

class FixedChief final : public IChiefStateProvider
{
  public:
    ChiefState s{};
    [[nodiscard]] ChiefState get(double t0) noexcept override
    {
        s.time_tag = t0;
        return s;
    }
};

class FixedVehicle final : public IVehicleStateProvider
{
  public:
    VehicleState s{};
    [[nodiscard]] VehicleState get(VehicleIndexMap::VehicleId /*id*/, double t0) noexcept override
    {
        s.time_tag = t0;
        return s;
    }
};

class FakeModel final : public IRelativeModel
{
  public:
    Result predict_hcw(const RelStateRic& /*x0*/,
                       const HcwParams& /*params*/,
                       const TimeGrid& grid,
                       Span<Vec3> out_r,
                       Span<Vec3> /*out_v*/) const noexcept override
    {
        if (out_r.data == nullptr || out_r.size < grid.tau.size())
        {
            return Result{ModelCode::kInsufficientOutputCapacity, 0};
        }
        for (std::size_t k = 0; k < grid.tau.size(); ++k)
        {
            // Deterministic pattern: (k, 2k, 3k)
            out_r[k] = Vec3{static_cast<double>(k), 2.0 * static_cast<double>(k), 3.0 * static_cast<double>(k)};
        }
        return Result{ModelCode::kOk, grid.tau.size()};
    }
};

} // namespace

TEST_CASE("RelativePredictor (injection): writes model outputs into prediction buffer", "[predictor]")
{
    // Vehicle registry
    VehicleIndexMap map;
    const VehicleIndexMap::VehicleId veh0 = static_cast<std::uint64_t>(0xBEEFu)
    REQUIRE(map.register_vehicle("veh0").has_value());
    // Chief and adopted frame providers: simplest path is constructed-only bullseye.
    FixedChief chief;
    chief.s.r_i = Vec3{7000e3, 0.0, 0.0};
    chief.s.v_i = Vec3{0.0, 7500.0, 0.0};
    chief.s.frame_id = "INERTIAL";
    chief.s.status.code = ProviderCode::kOk;

    // No adopted provider; constructed-only
    BullseyeFrame bullseye(chief, nullptr, BullseyeFrameMode::kConstructedOnly);

    // Vehicle state identical to chief => rel state 0, but FakeModel ignores it anyway.
    FixedVehicle veh;
    veh.s.r_i = chief.s.r_i;
    veh.s.v_i = chief.s.v_i;
    veh.s.frame_id = chief.s.frame_id;
    veh.s.status.code = ProviderCode::kOk;

    // Publisher + buffer
    Publisher pub;

    // Model injection
    FakeModel model;

    // Predictor under test (Option A interface)
    RelativePredictor pred(pub, map, chief, veh, bullseye, model);

    // Run one tick
    const double t0 = 10.0;
    pred.step(t0, /*horizon*/ 2.0, /*cadence*/ 1.0);

    // Validate published buffer contents
    const auto snap = pub.read();
    REQUIRE(snap != nullptr);
    REQUIRE(snap->t0 == t0);

    // TimeGrid: {0,1,2} => 3 steps (assuming your make_time_grid includes endpoint).
    // Your time_grid semantics are already tested; use that same expectation:
    REQUIRE(snap->positions[0][0].x == 0.0);
    REQUIRE(snap->positions[0][1].x == 1.0);
    REQUIRE(snap->positions[0][2].x == 2.0);
}
