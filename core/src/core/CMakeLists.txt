# ##############################################################################
# Create header file list for core library
# ##############################################################################
set(CORE_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/core/include/core)
set(CORE_HEADER_LIST
    ${CORE_INCLUDE_DIR}/utils/cash.hpp
    ${CORE_INCLUDE_DIR}/utils/data_logger.hpp
    ${CORE_INCLUDE_DIR}/utils/data_struct.hpp
    ${CORE_INCLUDE_DIR}/utils/data_struct_template.hpp
    ${CORE_INCLUDE_DIR}/utils/data.hpp
    ${CORE_INCLUDE_DIR}/utils/date_time.hpp
    ${CORE_INCLUDE_DIR}/utils/event_level.hpp
    ${CORE_INCLUDE_DIR}/utils/labeld_modifier.hpp
    ${CORE_INCLUDE_DIR}/utils/exception.hpp
    ${CORE_INCLUDE_DIR}/utils/formatter.hpp
    ${CORE_INCLUDE_DIR}/utils/logger_helper.hpp
    ${CORE_INCLUDE_DIR}/utils/logger_macros.hpp
    ${CORE_INCLUDE_DIR}/utils/logger_node.hpp
    ${CORE_INCLUDE_DIR}/utils/logger_stream.hpp
    ${CORE_INCLUDE_DIR}/utils/logger.hpp
    ${CORE_INCLUDE_DIR}/utils/math.hpp
    ${CORE_INCLUDE_DIR}/utils/module.hpp
    ${CORE_INCLUDE_DIR}/utils/node.hpp
    ${CORE_INCLUDE_DIR}/utils/socket_exception.hpp
    ${CORE_INCLUDE_DIR}/utils/socket.hpp
    ${CORE_INCLUDE_DIR}/utils/subject.hpp
    ${CORE_INCLUDE_DIR}/utils/system.hpp
    ${CORE_INCLUDE_DIR}/utils/spinner.hpp
    ${CORE_INCLUDE_DIR}/utils/time.hpp
    ${CORE_INCLUDE_DIR}/utils/timer.hpp
    ${CORE_INCLUDE_DIR}/utils/modifier.hpp
    ${CORE_INCLUDE_DIR}/utils/writer_file.hpp
    ${CORE_INCLUDE_DIR}/utils/writer.hpp
    ${CORE_INCLUDE_DIR}/sensors/module_sensor_imu.hpp
    ${CORE_INCLUDE_DIR}/sensors/module_sensor.hpp)

# ##############################################################################
# Create source file list for core library
# ##############################################################################
set(CORE_SOURCE_DIR ${PROJECT_SOURCE_DIR}/core/src/core)
set(CORE_SOURCE_LIST
    ${CORE_SOURCE_DIR}/utils/clock.cpp
    ${CORE_SOURCE_DIR}/utils/date_time.cpp
    ${CORE_SOURCE_DIR}/utils/data_struct.cpp
    ${CORE_SOURCE_DIR}/utils/data.cpp
    ${CORE_SOURCE_DIR}/utils/event_level.cpp
    ${CORE_SOURCE_DIR}/utils/labeld_modifier.cpp
    ${CORE_SOURCE_DIR}/utils/formatter.cpp
    ${CORE_SOURCE_DIR}/utils/modifier.cpp
    ${CORE_SOURCE_DIR}/utils/logger_node.cpp
    ${CORE_SOURCE_DIR}/utils/logger_stream.cpp
    ${CORE_SOURCE_DIR}/utils/logger.cpp
    ${CORE_SOURCE_DIR}/utils/math.cpp
    ${CORE_SOURCE_DIR}/utils/module.cpp
    ${CORE_SOURCE_DIR}/utils/node.cpp
    ${CORE_SOURCE_DIR}/utils/server_socket.cpp
    ${CORE_SOURCE_DIR}/utils/socket.cpp
    ${CORE_SOURCE_DIR}/utils/spinner.cpp
    ${CORE_SOURCE_DIR}/utils/system.cpp
    ${CORE_SOURCE_DIR}/utils/time.cpp
    ${CORE_SOURCE_DIR}/utils/writer_file.cpp
    ${CORE_SOURCE_DIR}/utils/writer.cpp
    ${CORE_SOURCE_DIR}/sensors/module_sensor_imu.cpp
    ${CORE_SOURCE_DIR}/sensors/module_sensor.cpp)

# ##############################################################################
# Build core library
# ##############################################################################
# Declare a C++ library Make an automatic library - will be static or dynamic
# based on user setting
add_library(${CORE_LIBRARY} ${CORE_SOURCE_LIST}) # ${CORE_HEADER_LIST}

# Global includes
target_include_directories(
    ${CORE_LIBRARY} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/core/include>
                           $<INSTALL_INTERFACE:${ABSOLUTE_INSTALL_INCLUDEDIR}>)

# Link required library
target_link_libraries(${CORE_LIBRARY} PRIVATE pthread stdc++fs Eigen3::Eigen)

# All users of this library will need at least C++17
target_compile_features(${CORE_LIBRARY} PUBLIC cxx_std_17)

target_compile_options(${CORE_LIBRARY} PRIVATE ${PROJECT_WARNINGS})

# IDEs should put the headers in a nice place
source_group(
    TREE "${PROJECT_SOURCE_DIR}/core/include"
    PREFIX "Header Files"
    FILES ${CORE_HEADER_LIST})

# ##############################################################################
# Install
# ##############################################################################

install(
    TARGETS ${CORE_LIBRARY}
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
    DIRECTORY ${CORE_INCLUDE_DIR}/
    DESTINATION ${ABSOLUTE_INSTALL_INCLUDEDIR}
    COMPONENT PATTERN
    "${CORE_HEADER_LIST}")
