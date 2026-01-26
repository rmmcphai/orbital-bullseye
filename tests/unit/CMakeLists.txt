# tests/unit/CMakeLists.txt (raw snippet)
add_executable(test_contracts_smoke
  test_contracts_smoke.cpp
)

target_include_directories(test_contracts_smoke PRIVATE
  ${CMAKE_SOURCE_DIR}
)

target_link_libraries(test_contracts_smoke PRIVATE
  Catch2::Catch2WithMain
  sim_logger::sim_logger
)

# Catch2 v3 provides this module when you add_subdirectory(Catch2)
include(Catch)
catch_discover_tests(test_contracts_smoke)


