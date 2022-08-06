find_package(Doxygen REQUIRED dot)
if(Doxygen_FOUND)
  set(DOXYGEN_CALLER_GRAPH YES)
  set(DOXYGEN_CALL_GRAPH YES)
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
  set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/docs)

  doxygen_add_docs(
    docs ${PROJECT_SOURCE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/main_page.md"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/include")
  message(STATUS "Doxygen has been setup and documentation is now available.")
else()
  message(STATUS "Doxygen not found, not building docs")
endif()
