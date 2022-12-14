# ##############################################################################
# System Dependencies
# ##############################################################################

# ##############################################################################
# Library
# ##############################################################################
# define library name
if (BUILD_HARDWARE_COMMON)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/common)
endif ()

if (BUILD_NAVIO)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/navio)
endif ()
message(STATUS "NAVIO project is set ${BUILD_NAVIO}")

if (BUILD_BBB)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/bbb)
endif ()
message(STATUS "BBB project is set ${BUILD_BBB}")

# ##############################################################################
# Executables
# ##############################################################################
if (BUILD_EXECTUABLES)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/app)
endif ()

# ##############################################################################
# testing setup
# ##############################################################################
# if (BUILD_TESTING) message( STATUS "Build unit tests for the project. Tests
# should always be found in the test folder\n" )
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test) endif ()
