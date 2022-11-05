# ##############################################################################
# System Dependencies
# ##############################################################################
# find googletest library
find_package(GTest REQUIRED)
if (NOT GTest_FOUND)
    message(FATAL_ERROR "Google Test library [gtest] not found")
endif ()

include(GoogleTest)
set(GOOGLE_LIBRARIES gtest gtest_main)
