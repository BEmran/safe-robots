# function(initialize_submodule DIRECTORY)
#   if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${DIRECTORY}/.git)
#     find_package(Git QUIET REQUIRED)
#     message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}/${DIRECTORY}/.git does not exist. Initializing ${DIRECTORY} submodule ...")
#     execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init ${DIRECTORY}
#                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#                     RESULT_VARIABLE GIT_EXIT_CODE)
#     if(NOT GIT_EXIT_CODE EQUAL "0")
#       message(FATAL_ERROR "${GIT_EXECUTABLE} submodule update --init dependencies/${DIRECTORY} failed with exit code ${GIT_EXIT_CODE}, please checkout submodules")
#     endif()
#   else()
#     message(STATUS "Submodule at ${DIRECTORY} is checked.")
#   endif()
# endfunction(initialize_submodule)

# initialize_submodule(yaml-cpp)
# add_subdirectory(yaml-cpp EXCLUDE_FROM_ALL)

# include(ExternalProject)

# ExternalProject_Add(
# 	yaml_cpp_project
# 	SOURCE_DIR "${CMAKE_SOURCE_DIR}/dependencies/yaml-cpp/"
# 	INSTALL_DIR "${CMAKE_BINARY_DIR}/prefix"
# 	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
# )
# ExternalProject_Get_Property(yaml_cpp_project BINARY_DIR)
# ExternalProject_Get_Property(yaml_cpp_project SOURCE_DIR)

# # add_library(yaml-cpp UNKNOWN IMPORTED)
# add_library(yaml-cpp STATIC IMPORTED)

# set_target_properties(yaml-cpp PROPERTIES IMPORTED_LOCATION ${BINARY_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}yaml-cpp.${CMAKE_STATIC_LIBRARY_SUFFIX})
# include_directories(${SOURCE_DIR}/include/)
