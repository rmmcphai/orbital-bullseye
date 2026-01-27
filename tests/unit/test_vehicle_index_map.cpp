#include <catch2/catch_test_macros.hpp>
#include "core/vehicle_index_map.hpp"

using bullseye_pred::VehicleIndexMap;

TEST_CASE("register_vehicle assigns stable indices in insertion order")
{
    VehicleIndexMap map;

    auto a = map.register_vehicle(100);
    auto b = map.register_vehicle(200);
    auto c = map.register_vehicle(300);

    REQUIRE(a.has_value());
    REQUIRE(b.has_value());
    REQUIRE(c.has_value());

    REQUIRE(*a == 0);
    REQUIRE(*b == 1);
    REQUIRE(*c == 2);

    // Re-register returns same index, does not change size.
    auto b2 = map.register_vehicle(200);
    REQUIRE(b2.has_value());
    REQUIRE(*b2 == 1);
    REQUIRE(map.size() == 3);
}

TEST_CASE("index_of returns nullopt when not present")
{
    VehicleIndexMap map;
    REQUIRE_FALSE(map.index_of(42).has_value());
}

TEST_CASE("id_at reflects stable insertion order")
{
    VehicleIndexMap map;
    map.register_vehicle(7);
    map.register_vehicle(9);

    REQUIRE(map.id_at(0).has_value());
    REQUIRE(map.id_at(1).has_value());
    REQUIRE(*map.id_at(0) == 7);
    REQUIRE(*map.id_at(1) == 9);

    REQUIRE_FALSE(map.id_at(2).has_value());
}

TEST_CASE("capacity enforcement: register_vehicle returns nullopt when full")
{
    VehicleIndexMap map;

    // Fill to capacity.
    for (std::size_t i = 0; i < VehicleIndexMap::capacity(); ++i)
    {
        auto idx = map.register_vehicle(1000 + i);
        REQUIRE(idx.has_value());
        REQUIRE(*idx == i);
    }

    REQUIRE(map.size() == VehicleIndexMap::capacity());

    // Next insertion should fail.
    auto fail = map.register_vehicle(999999);
    REQUIRE_FALSE(fail.has_value());
}
