# ##############################################################################
# System Dependencies
# ##############################################################################

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
# if (BUILD_TESTING) message( STATUS "Build unit tests for the project. Tests
# should always be found in the test folder\n" )
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test) endif ()
