# ##############################################################################
# Create header file list for NAVIO library
# ##############################################################################
set(BBB_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/hardware/include/bbb)
set(BBB_HEADER_LIST ${BBB_INCLUDE_DIR}/sensors/mpu/bbb_mpu9250.hpp
                    ${BBB_INCLUDE_DIR}/utils.hpp)

# ##############################################################################
# Create source file list for NAVIO library
# ##############################################################################
set(BBB_SOURCE_DIR ${PROJECT_SOURCE_DIR}/hardware/src/bbb)
set(BBB_SOURCE_LIST ${BBB_SOURCE_DIR}/sensors/mpu/bbb_mpu9250.cpp
                    ${BBB_SOURCE_DIR}/utils.cpp)

# ##############################################################################
# Build NAVIO library
# ##############################################################################
# Declare a C++ library Make an automatic library - will be static or dynamic
# based on user setting
add_library(${BBB_LIBRARY} ${BBB_SOURCE_LIST}) # ${BBB_HEADER_LIST}

# Global includes
target_include_directories(
    ${BBB_LIBRARY}
    PUBLIC # $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/core/include>
           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
           # $<INSTALL_INTERFACE:${ABSOLUTE_INSTALL_INCLUDEDIR}>
)

# Link required library
target_link_libraries(${BBB_LIBRARY} PRIVATE ${CORE_LIBRARY}
                                             ${HARDWARE_COMMON_LIBRARY})

# All users of this library will need at least C++17
target_compile_features(${BBB_LIBRARY} PUBLIC cxx_std_17)

target_compile_options(${BBB_LIBRARY} PRIVATE ${PROJECT_WARNINGS})

# IDEs should put the headers in a nice place
source_group(
    TREE "${PROJECT_SOURCE_DIR}/hardware/include"
    PREFIX "Header Files"
    FILES ${BBB_HEADER_LIST})

# ##############################################################################
# Install
# ##############################################################################

install(
    TARGETS ${BBB_LIBRARY}
    # IMPORTANT: Add the navio_library to the "export-set"
    EXPORT ${TARGETS_EXPORT_NAME}
    LIBRARY DESTINATION ${ABSOLUTE_INSTALL_LIBDIR} COMPONENT shlib
    ARCHIVE DESTINATION ${ABSOLUTE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${ABSOLUTE_INSTALL_BINDIR} COMPONENT bin
    INCLUDES
    DESTINATION ${ABSOLUTE_INSTALL_INCLUDEDIR}
    # PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hardware/navio
    # COMPONENT dev
)

install(
    DIRECTORY ${BBB_INCLUDE_DIR}/
    DESTINATION ${ABSOLUTE_INSTALL_INCLUDEDIR}
    COMPONENT PATTERN
    "${BBB_HEADER_LIST}")
