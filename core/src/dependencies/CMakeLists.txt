# ##############################################################################
# Create header file list for core library
# ##############################################################################
set(DEPEN_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/core/include/dependencies)
set(DEPEN_HEADER_LIST
    ${DEPEN_INCLUDE_DIR}/entree.hpp
    ${DEPEN_INCLUDE_DIR}/yaml.hpp)

# ##############################################################################
# Create source file list for core library
# ##############################################################################
set(DEPEN_SOURCE_DIR ${PROJECT_SOURCE_DIR}/core/src/dependencies)
set(DEPEN_SOURCE_LIST
    ${DEPEN_SOURCE_DIR}/entree.cpp
    ${DEPEN_SOURCE_DIR}/yaml.cpp)

# ##############################################################################
# Build core library
# ##############################################################################
# Declare a C++ library Make an automatic library - will be static or dynamic
# based on user setting
add_library(${DEPEN_LIBRARY} ${DEPEN_SOURCE_LIST}) # ${DEPEN_HEADER_LIST}

# Global includes
target_include_directories(
    ${DEPEN_LIBRARY} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/core/include>
                            $<INSTALL_INTERFACE:${ABSOLUTE_INSTALL_INCLUDEDIR}>
                            ${YAML_CPP_INCLUDE_DIR})

# Link required library
target_link_libraries(${DEPEN_LIBRARY} PRIVATE pthread stdc++fs ${YAML_CPP_LIBRARIES})

# All users of this library will need at least C++17
target_compile_features(${DEPEN_LIBRARY} PUBLIC cxx_std_17)

target_compile_options(${DEPEN_LIBRARY} PRIVATE ${PROJECT_WARNINGS})
# add_dependencies(${DEPEN_LIBRARY} yaml_cpp_project)
# IDEs should put the headers in a nice place
source_group(
    TREE "${PROJECT_SOURCE_DIR}/core/include"
    PREFIX "Header Files"
    FILES ${DEPEN_HEADER_LIST})

# ##############################################################################
# Install
# ##############################################################################

install(
    TARGETS ${DEPEN_LIBRARY}
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
    DIRECTORY ${DEPEN_INCLUDE_DIR}/
    DESTINATION ${ABSOLUTE_INSTALL_INCLUDEDIR}
    COMPONENT PATTERN
    "${DEPEN_HEADER_LIST}")
