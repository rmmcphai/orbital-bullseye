#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "core/dummy_predictor.hpp"
#include "core/publisher.hpp"
#include "core/vehicle_index_map.hpp"

using bullseye::DummyPredictor;
using bullseye::Publisher;
using bullseye::VehicleIndexMap;

TEST_CASE("DummyPredictor publishes deterministic content") {
  Publisher pub;
  VehicleIndexMap map;

  // Register two vehicles (stable insertion order -> indices 0,1).
  REQUIRE(map.register_vehicle(100).has_value());
  REQUIRE(map.register_vehicle(200).has_value());

  DummyPredictor pred(pub, map);

  pred.step(/*t0=*/10.0, /*horizon=*/1.0, /*cadence=*/0.5);

  const auto& snap = pub.read();
  REQUIRE(snap.seqno == 1);
  REQUIRE(snap.t0 == 10.0);

  // Grid: tau = {0.0, 0.5, 1.0} => steps=3
  // Vehicle 0, step 0:
  REQUIRE(snap.positions[0][0].x == 0.0);
  REQUIRE(snap.positions[0][0].y == 0.0);
  REQUIRE(snap.positions[0][0].z == 0.0);

  // Vehicle 1, step 2:
  // x = i + 0.001*k = 1 + 0.002
  // y = k + 0.01*i = 2 + 0.01
  REQUIRE(snap.positions[1][2].x == Catch::Approx(1.002));
  REQUIRE(snap.positions[1][2].y == Catch::Approx(2.01));
  REQUIRE(snap.positions[1][2].z == Catch::Approx(1.0));
}

TEST_CASE("DummyPredictor does not publish on empty grid") {
  Publisher pub;
  VehicleIndexMap map;
  DummyPredictor pred(pub, map);

  // Invalid cadence -> empty grid -> no publish.
  pred.step(/*t0=*/10.0, /*horizon=*/1.0, /*cadence=*/0.0);

  REQUIRE(pub.published_seqno() == 0);
}
