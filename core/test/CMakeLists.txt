# ##############################################################################
# System Dependencies
# ##############################################################################
# find googletest library Prevent overriding the parent project's
# compiler/linker settings on Windows
set(gtest_force_shared_crt
    ON
    CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines the gtest and gtest_main
# targets.
add_subdirectory(
    ${PROJECT_SOURCE_DIR}/dependencies/googletest
    ${PROJECT_SOURCE_DIR}/dependencies/build/googletest EXCLUDE_FROM_ALL)

set(GTEST_LIBRARIES gtest gmock gtest_main)

# ##############################################################################
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/AddTestWithLibraries.cmake)

# ##############################################################################
add_subdirectory(core)
add_subdirectory(dependencies)
