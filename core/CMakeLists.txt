# ##############################################################################
# System Dependencies
# ##############################################################################
# find Eigen library
find_package(Eigen3 3.3 REQUIRED)
if (NOT Eigen3_FOUND)
    message(FATAL_ERROR "Google Test library [Eigen3] not found")
endif ()

# ##############################################################################
# Library
# ##############################################################################
# define library name
if (BUILD_SHARED_LIBS)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)
endif ()

# ##############################################################################
# Executables
# ##############################################################################
if (BUILD_EXECTUABLES)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/app)
endif ()

# ##############################################################################
# testing setup
# ##############################################################################
if (BUILD_TESTING)
    message(
        STATUS
            "Build unit tests for the project. Tests should always be found in the test folder\n"
    )
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
endif ()
