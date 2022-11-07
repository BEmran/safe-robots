# ##############################################################################
# Create header file list for BBB library
# ##############################################################################
set(BBB_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/hardware/bbb/include)
set(BBB_HEADER_LIST
    ${BBB_INCLUDE_DIR}/bbb/communication_abs.hpp
    ${BBB_INCLUDE_DIR}/bbb/hardware_utils.hpp
    ${BBB_INCLUDE_DIR}/bbb/i2c.hpp
    ${BBB_INCLUDE_DIR}/bbb/spi.hpp
    ${BBB_INCLUDE_DIR}/bbb/time.hpp
    ${BBB_INCLUDE_DIR}/sensors/common/calibrate.hpp
    ${BBB_INCLUDE_DIR}/sensors/common/utils.hpp
    ${BBB_INCLUDE_DIR}/sensors/mpu/mpu9250_register_map.hpp
    ${BBB_INCLUDE_DIR}/sensors/mpu/mpu.hpp
    ${BBB_INCLUDE_DIR}/sensors/mpu/mpu9250.hpp)

# ##############################################################################
# Create source file list for BBB library
# ##############################################################################
set(BBB_SOURCE_DIR ${PROJECT_SOURCE_DIR}/hardware/bbb/src)
set(BBB_SOURCE_LIST
    ${BBB_SOURCE_DIR}/bbb/hardware_utils.cpp
    ${BBB_SOURCE_DIR}/bbb/i2c.cpp
    ${BBB_SOURCE_DIR}/bbb/spi.cpp
    ${BBB_SOURCE_DIR}/bbb/time.cpp
    ${BBB_SOURCE_DIR}/sensors/common/calibrate.cpp
    ${BBB_SOURCE_DIR}/sensors/common/utils.cpp
    ${BBB_SOURCE_DIR}/sensors/mpu/mpu.cpp
    ${BBB_SOURCE_DIR}/sensors/mpu/mpu9250.cpp)

# ##############################################################################
# Build BBB library
# ##############################################################################
# Declare a C++ library Make an automatic library - will be static or dynamic
# based on user setting
add_library(${BBB_LIBRARY} ${BBB_SOURCE_LIST}) # ${BBB_HEADER_LIST}

# Global includes
target_include_directories(
    ${BBB_LIBRARY}
    PUBLIC # $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/core/include>
           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/bbb/include>
           # $<INSTALL_INTERFACE:${ABSOLUTE_INSTALL_INCLUDEDIR}>
)

# Link required library
target_link_libraries(${BBB_LIBRARY} PRIVATE ${CORE_LIBRARY} pthread)

# All users of this library will need at least C++17
target_compile_features(${BBB_LIBRARY} PUBLIC cxx_std_17)

target_compile_options(${BBB_LIBRARY} PRIVATE ${PROJECT_WARNINGS})

# IDEs should put the headers in a nice place
source_group(
    TREE "${PROJECT_SOURCE_DIR}/hardware/bbb/include"
    PREFIX "Header Files"
    FILES ${BBB_HEADER_LIST})

# ##############################################################################
# Install
# ##############################################################################

install(
    TARGETS ${BBB_LIBRARY}
    # IMPORTANT: Add the bbb_library to the "export-set"
    EXPORT ${TARGETS_EXPORT_NAME}
    LIBRARY DESTINATION ${ABSOLUTE_INSTALL_LIBDIR} COMPONENT shlib
    ARCHIVE DESTINATION ${ABSOLUTE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${ABSOLUTE_INSTALL_BINDIR} COMPONENT bin
    INCLUDES
    DESTINATION ${ABSOLUTE_INSTALL_INCLUDEDIR}
    # PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hardware/bbb
    # COMPONENT dev
)

install(
    DIRECTORY ${BBB_INCLUDE_DIR}/
    DESTINATION ${ABSOLUTE_INSTALL_INCLUDEDIR}
    COMPONENT PATTERN
    "${BBB_HEADER_LIST}")
