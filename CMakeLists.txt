# CMakeLists.txt (raw)
cmake_minimum_required(VERSION 3.20)

project(orbital_bullseye
  VERSION 0.1.0
  LANGUAGES CXX
)

# ------------------------------
# Global build settings
# ------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ------------------------------
# Dependencies (sim-logger required, Catch2 optional)
# ------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(cmake/Dependencies.cmake)

# ------------------------------
# Core library (scaffold)
# ------------------------------
add_library(orbital_bullseye_core
  core/bullseye_frame.cpp
  core/bullseye_frame_math.cpp
  core/bullseye_frame_validator.cpp
  core/frame_provider_cartesian.cpp
  core/frame_transforms.cpp
  core/prediction_buffer.cpp
  core/provider_cartesian.cpp
  core/provider_twobody.cpp
  core/publisher.cpp
  core/relative_predictor.cpp
  core/time_grid.cpp
  core/vehicle_index_map.cpp
)

target_include_directories(orbital_bullseye_core
  PUBLIC
    ${CMAKE_SOURCE_DIR}
)

# sim-logger is required; canonical target guaranteed by Dependencies.cmake
target_link_libraries(orbital_bullseye_core
  PUBLIC
    sim_logger::sim_logger
)

# ------------------------------
# Optional integration sources (scaffold)
# ------------------------------
option(BULLSEYE_BUILD_INTEGRATION "Build JEOD/Trick integration stubs" ON)

if(BULLSEYE_BUILD_INTEGRATION)
  add_library(orbital_bullseye_integration
    integration/adapters/jeod_relstate_wrapper.cpp
    integration/bullseye_jeod_adapter.cpp
    integration/frame_provider_jeod.cpp
    integration/provider_jeod.cpp
    integration/trick_jobs.cpp
  )

  target_include_directories(orbital_bullseye_integration
    PUBLIC
      ${CMAKE_SOURCE_DIR}
  )

  target_link_libraries(orbital_bullseye_integration
    PUBLIC
      orbital_bullseye_core
      sim_logger::sim_logger
  )
endif()

# ------------------------------
# Models library (scaffold)
# ------------------------------
add_library(orbital_bullseye_models
  models/model_hcw.cpp
  models/model_selector.cpp
  models/model_ya_stm.cpp
)

target_include_directories(orbital_bullseye_models
  PUBLIC
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(orbital_bullseye_models
  PUBLIC
    orbital_bullseye_core
    sim_logger::sim_logger
)

# ------------------------------
# Tests (optional: only if Catch2 is available)
# ------------------------------
include(CTest)
enable_testing()

if(BULLSEYE_ENABLE_TESTS)
  add_subdirectory(tests)
endif()
