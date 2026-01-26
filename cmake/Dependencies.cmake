# cmake/Dependencies.cmake (raw, updated to fix Catch2 target collision)
#
# Root cause:
# - sim-logger vendors Catch2 under external/sim-logger/third_party/Catch2 and is currently
#   configuring it (likely via sim-logger tests).
# - You also add external/catch2, so CMake tries to create targets Catch2/Catch2WithMain twice.
#
# Fix strategy:
# 1) Prevent sim-logger from configuring its own tests (preferred: keeps Catch2 truly optional).
# 2) If Catch2 targets already exist (from anywhere), do NOT add external/catch2 again.

include(FetchContent)

option(BULLSEYE_ENABLE_TESTS "Build tests (requires Catch2)" ON)
option(BULLSEYE_USE_CATCH2_SUBMODULE "Use Catch2 from external/catch2 submodule" ON)
option(BULLSEYE_FETCH_CATCH2_IF_MISSING "Fetch Catch2 if submodule missing" OFF)

option(BULLSEYE_USE_SIM_LOGGER_SUBDIR "Use sim-logger from a subdirectory" ON)

# ------------------------------
# sim-logger (REQUIRED)
# ------------------------------
# Ensure sim-logger does NOT pull in its own Catch2/tests.
# We set multiple common option names defensively; harmless if unused.
set(BUILD_TESTING OFF CACHE BOOL "Disable tests for dependencies" FORCE)
set(SIM_LOGGER_BUILD_TESTS OFF CACHE BOOL "Disable sim-logger tests" FORCE)
set(SIM_LOGGER_ENABLE_TESTS OFF CACHE BOOL "Disable sim-logger tests" FORCE)
set(SIM_LOGGER_TESTS OFF CACHE BOOL "Disable sim-logger tests" FORCE)
set(SIM_LOGGER_BUILD_EXAMPLES OFF CACHE BOOL "Disable sim-logger examples" FORCE)
set(SIM_LOGGER_ENABLE_EXAMPLES OFF CACHE BOOL "Disable sim-logger examples" FORCE)

if(BULLSEYE_USE_SIM_LOGGER_SUBDIR)
  if(EXISTS "${CMAKE_SOURCE_DIR}/external/sim-logger/CMakeLists.txt")
    add_subdirectory("${CMAKE_SOURCE_DIR}/external/sim-logger" "${CMAKE_BINARY_DIR}/_deps/sim-logger")
  elseif(EXISTS "${CMAKE_SOURCE_DIR}/sim-logger/CMakeLists.txt")
    add_subdirectory("${CMAKE_SOURCE_DIR}/sim-logger" "${CMAKE_BINARY_DIR}/_deps/sim-logger")
  endif()
endif()

# Canonicalize sim-logger target name
if(TARGET sim_logger::sim_logger)
  # already canonical
elseif(TARGET logger_core)
  add_library(sim_logger::sim_logger ALIAS logger_core)
elseif(TARGET sim_logger)
  add_library(sim_logger::sim_logger ALIAS sim_logger)
elseif(TARGET logger::logger_core)
  add_library(sim_logger::sim_logger ALIAS logger::logger_core)
elseif(TARGET logger::core)
  add_library(sim_logger::sim_logger ALIAS logger::core)
else()
  message(FATAL_ERROR
    "sim-logger is required but no recognized target was found.\n"
    "Looked for: sim_logger::sim_logger, logger_core, sim_logger, logger::logger_core, logger::core.\n"
    "Fix: export/alias a canonical target from sim-logger or update this mapping."
  )
endif()

# Re-enable testing for THIS project if requested.
# (This does not retroactively enable sim-logger tests, since it already configured.)
if(BULLSEYE_ENABLE_TESTS)
  set(BUILD_TESTING ON CACHE BOOL "Enable tests for this project" FORCE)
endif()

# ------------------------------
# Catch2 (OPTIONAL)
# ------------------------------
if(BULLSEYE_ENABLE_TESTS)
  set(_have_catch2 OFF)

  # If Catch2 targets already exist (from any dependency), reuse them.
  if(TARGET Catch2::Catch2WithMain OR TARGET Catch2WithMain OR TARGET Catch2::Catch2 OR TARGET Catch2)
    set(_have_catch2 ON)
  endif()

  if(NOT _have_catch2)
    if(BULLSEYE_USE_CATCH2_SUBMODULE AND EXISTS "${CMAKE_SOURCE_DIR}/external/catch2/CMakeLists.txt")
      add_subdirectory("${CMAKE_SOURCE_DIR}/external/catch2" "${CMAKE_BINARY_DIR}/_deps/catch2")
      set(_have_catch2 ON)
    elseif(BULLSEYE_FETCH_CATCH2_IF_MISSING)
      FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.4
      )
      FetchContent_MakeAvailable(Catch2)
      set(_have_catch2 ON)
    endif()
  endif()

  if(NOT _have_catch2)
    message(STATUS "Catch2 not found and fetch disabled; disabling tests.")
    set(BULLSEYE_ENABLE_TESTS OFF CACHE BOOL "Build tests (requires Catch2)" FORCE)
  endif()
endif()
