# ##############################################################################
# Create header file list for NAVIO library
# ##############################################################################
set(NAVIO_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/hardware/navio/include)
set(NAVIO_HEADER_LIST
    ${NAVIO_INCLUDE_DIR}/navio/communication_abs.hpp
    ${NAVIO_INCLUDE_DIR}/navio/hardware_utils.hpp
    ${NAVIO_INCLUDE_DIR}/navio/spi.hpp
    ${NAVIO_INCLUDE_DIR}/sensors/common/calibrate.hpp
    ${NAVIO_INCLUDE_DIR}/sensors/common/utils.hpp
    ${NAVIO_INCLUDE_DIR}/sensors/lsm/lsm9ds1_register_map.hpp
    ${NAVIO_INCLUDE_DIR}/sensors/lsm/lsm9ds1.hpp
    ${NAVIO_INCLUDE_DIR}/sensors/mpu/mpu9250_register_map.hpp
    ${NAVIO_INCLUDE_DIR}/sensors/mpu/mpu9250.hpp)

# ##############################################################################
# Create source file list for NAVIO library
# ##############################################################################
set(NAVIO_SOURCE_DIR ${PROJECT_SOURCE_DIR}/hardware/navio/src)
set(NAVIO_SOURCE_LIST
    ${NAVIO_SOURCE_DIR}/navio/hardware_utils.cpp
    ${NAVIO_SOURCE_DIR}/navio/spi.cpp
    ${NAVIO_SOURCE_DIR}/sensors/common/calibrate.cpp
    ${NAVIO_SOURCE_DIR}/sensors/common/utils.cpp
    ${NAVIO_SOURCE_DIR}/sensors/lsm/lsm9ds1.cpp
    ${NAVIO_SOURCE_DIR}/sensors/mpu/mpu9250.cpp)

# ##############################################################################
# Build NAVIO library
# ##############################################################################
# Declare a C++ library Make an automatic library - will be static or dynamic
# based on user setting
add_library(${NAVIO_LIBRARY} ${NAVIO_SOURCE_LIST}) # ${NAVIO_HEADER_LIST}

# Global includes
target_include_directories(
    ${NAVIO_LIBRARY}
    PUBLIC # $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/core/include>
           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/navio/include>
           # $<INSTALL_INTERFACE:${ABSOLUTE_INSTALL_INCLUDEDIR}>
)

# Link required library
target_link_libraries(${NAVIO_LIBRARY} PRIVATE ${CORE_LIBRARY} pthread)

# All users of this library will need at least C++17
target_compile_features(${NAVIO_LIBRARY} PUBLIC cxx_std_17)

target_compile_options(${NAVIO_LIBRARY} PRIVATE ${PROJECT_WARNINGS})

# IDEs should put the headers in a nice place
source_group(
    TREE "${PROJECT_SOURCE_DIR}/hardware/navio/include"
    PREFIX "Header Files"
    FILES ${NAVIO_HEADER_LIST})

# ##############################################################################
# Install
# ##############################################################################

install(
    TARGETS ${NAVIO_LIBRARY}
    # IMPORTANT: Add the navio_library to the "export-set"
    EXPORT ${TARGETS_EXPORT_NAME}
    LIBRARY DESTINATION ${ABSOLUTE_INSTALL_LIBDIR} COMPONENT shlib
    ARCHIVE DESTINATION ${ABSOLUTE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${ABSOLUTE_INSTALL_BINDIR} COMPONENT bin
    INCLUDES
    DESTINATION ${ABSOLUTE_INSTALL_INCLUDEDIR}
    # PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hardware/navio COMPONENT dev
)

install(
    DIRECTORY ${NAVIO_INCLUDE_DIR}/
    DESTINATION ${ABSOLUTE_INSTALL_INCLUDEDIR}
    COMPONENT PATTERN
    "${NAVIO_HEADER_LIST}")
