# ##############################################################################
# Executables
# ##############################################################################
if (BUILD_EXECTUABLES)

    # if (BUILD_HARDWARE_COMMON)
    #     add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/app/common)
    # endif ()
    # message(STATUS "Build Common Hardware executables ${BUILD_HARDWARE_COMMON}")

    # if (BUILD_NAVIO)
    #     add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/navio)
    # endif ()
    # message(STATUS "Build NAVIO executables ${BUILD_NAVIO}")

    if (BUILD_BBB)
        add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/bbb)
    endif ()
    message(STATUS "Build BBB executables ${BUILD_BBB}")

endif ()