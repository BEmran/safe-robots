# Listing the source and header files indvisually is recommened over using GLOB
# method for more information check:
# https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1
# ##############################################################################
# Create header file list for core library
# ##############################################################################
set(CORE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(CORE_HEADER_LIST
    ${CORE_INCLUDE_DIR}/core/sensors/module_sensor.hpp
    ${CORE_INCLUDE_DIR}/core/utils/cash.hpp
    ${CORE_INCLUDE_DIR}/core/utils/data.hpp
    ${CORE_INCLUDE_DIR}/core/utils/date_time.hpp
    ${CORE_INCLUDE_DIR}/core/utils/event_level.hpp
    ${CORE_INCLUDE_DIR}/core/utils/exception.hpp
    ${CORE_INCLUDE_DIR}/core/utils/formatter.hpp
    ${CORE_INCLUDE_DIR}/core/utils/system.hpp
    ${CORE_INCLUDE_DIR}/core/utils/terminal.hpp
    ${CORE_INCLUDE_DIR}/core/utils/logger.hpp
    ${CORE_INCLUDE_DIR}/core/utils/module.hpp
    ${CORE_INCLUDE_DIR}/core/utils/node.hpp
    ${CORE_INCLUDE_DIR}/core/utils/writter_console.hpp
    ${CORE_INCLUDE_DIR}/core/utils/writter_file.hpp
    ${CORE_INCLUDE_DIR}/core/utils/writter.hpp)

# ##############################################################################
# Create source file list for core library
# ##############################################################################
set(CORE_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
set(CORE_SOURCE_LIST
    ${CORE_SOURCE_DIR}/core/sensors/module_sensor.cpp
    ${CORE_SOURCE_DIR}/core/utils/date_time.cpp
    ${CORE_SOURCE_DIR}/core/utils/data.cpp
    ${CORE_SOURCE_DIR}/core/utils/event_level.cpp
    ${CORE_SOURCE_DIR}/core/utils/formatter.cpp
    ${CORE_SOURCE_DIR}/core/utils/system.cpp
    ${CORE_SOURCE_DIR}/core/utils/terminal.cpp
    ${CORE_SOURCE_DIR}/core/utils/logger.cpp
    ${CORE_SOURCE_DIR}/core/utils/module.cpp
    ${CORE_SOURCE_DIR}/core/utils/node.cpp
    ${CORE_SOURCE_DIR}/core/utils/writter_console.cpp
    ${CORE_SOURCE_DIR}/core/utils/writter_file.cpp)

# ##############################################################################
# Build core library
# ##############################################################################

# Declare a C++ library Make an automatic library - will be static or dynamic
# based on user setting
add_library(${CORE_LIBRARY} ${CORE_SOURCE_LIST})

# Global includes. Used by all targets We need this directory, and users of our
# library will need it too Note: * header location in project:
# /include/core/*.hpp * header can be included by C++ code '#include
# <core/****.hpp>' Specify additional locations of header files Your package
# locations should be listed before other locations
target_include_directories(
    ${CORE_LIBRARY}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${ABSOLUTE_INSTALL_INCLUDEDIR}>
           ${EIGEN3_INCLUDE_DIR}
           # $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Link required library
target_link_libraries(${CORE_LIBRARY} PRIVATE stdc++fs Eigen3::Eigen)

# All users of this library will need at least C++17
target_compile_features(${CORE_LIBRARY} PUBLIC cxx_std_17)

target_compile_options(${CORE_LIBRARY} PRIVATE ${PROJECT_WARNINGS})

# IDEs should put the headers in a nice place
source_group(
    TREE "${PROJECT_SOURCE_DIR}/include"
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
    "${CORE_HEADER_FILES}")
