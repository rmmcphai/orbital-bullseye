if(NOT BULLSEYE_ENABLE_TESTS)
  return()
endif()

add_subdirectory(unit)
#add_subdirectory(integration)
#add_subdirectory(harness)

