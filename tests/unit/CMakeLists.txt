# tests/unit/CMakeLists.txt

set(UNIT_TEST_SOURCES
    test_contracts_smoke.cpp
    test_time_grid.cpp
    test_prediction_buffer.cpp
    test_vehicle_index_map.cpp
    test_publisher.cpp
    test_dummy_predictor.cpp
    test_providers.cpp
    test_frame_providers.cpp
    test_frame_validator.cpp
    test_bullseye_frame.cpp
    test_transforms.cpp
    test_bullseye_math.cpp
)

add_executable(orbital_bullseye_unit_tests
    ${UNIT_TEST_SOURCES}
)

target_link_libraries(orbital_bullseye_unit_tests
    PRIVATE
        Catch2::Catch2WithMain
        orbital_bullseye_core
)

include(CTest)
include(Catch)

catch_discover_tests(orbital_bullseye_unit_tests)
