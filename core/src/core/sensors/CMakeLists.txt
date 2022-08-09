# ##############################################################################
# Create header file list for core library
# ##############################################################################
set(SENSORS_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(SENSORS_HEADER_LIST
    ${SENSORS_INCLUDE_DIR}/core/sensors/module_sensor_imu.hpp
    ${SENSORS_INCLUDE_DIR}/core/sensors/module_sensor.hpp)

# ##############################################################################
# Create source file list for core library
# ##############################################################################
set(SENSORS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(SENSORS_SOURCE_LIST
    ${SENSORS_SOURCE_DIR}/module_sensor_imu.cpp
    ${SENSORS_SOURCE_DIR}/module_sensor.cpp)

# ##############################################################################
# Build core library
# ##############################################################################
# Declare a C++ library Make an automatic library - will be static or dynamic
# based on user setting
add_library(${SENSORS_LIBRARY} ${SENSORS_SOURCE_LIST} ${SENSORS_HEADER_LIST})

# Global includes
target_include_directories(
    ${SENSORS_LIBRARY}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${ABSOLUTE_INSTALL_INCLUDEDIR}>
)

# Link required library
target_link_libraries(${SENSORS_LIBRARY} PRIVATE ${CORE_LIBRARY})

# All users of this library will need at least C++17
target_compile_features(${SENSORS_LIBRARY} PUBLIC cxx_std_17)

target_compile_options(${SENSORS_LIBRARY} PRIVATE ${PROJECT_WARNINGS})

# IDEs should put the headers in a nice place
source_group(
    TREE "${PROJECT_SOURCE_DIR}/include"
    PREFIX "Header Files"
    FILES ${SENSORS_HEADER_LIST})

# ##############################################################################
# Install
# ##############################################################################

install(
    TARGETS ${SENSORS_LIBRARY} 
    # IMPORTANT: Add the core_library to the "export-set"
    EXPORT ${TARGETS_EXPORT_NAME}
    LIBRARY DESTINATION ${ABSOLUTE_INSTALL_LIBDIR} COMPONENT shlib
    ARCHIVE DESTINATION ${ABSOLUTE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${ABSOLUTE_INSTALL_BINDIR} COMPONENT bin
    INCLUDES
    DESTINATION ${ABSOLUTE_INSTALL_INCLUDEDIR}
    # PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/core COMPONENT dev
)

install(
    DIRECTORY ${SENSORS_INCLUDE_DIR}/
    DESTINATION ${ABSOLUTE_INSTALL_INCLUDEDIR}
    COMPONENT PATTERN
    "${SENSORS_HEADER_LIST}")