if(ENABLE_DOXYGEN)
    find_package(Doxygen REQUIRED dot)
    if (Doxygen_FOUND)
        set(DOXYGEN_CALLER_GRAPH YES)
        set(DOXYGEN_FULL_PATH_NAMES NO)
        set(DOXYGEN_EXTRACT_ALL YES)
        set(DOXYGEN_EXTRACT_PRIVATE YES) # required to also extract private members

        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.out)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

        # Note: do not put "ALL" - this builds docs together with application EVERY TIME!
        add_custom_target(docs ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )

        # doxygen_add_docs(
        #     docs ${PROJECT_SOURCE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/main_page.md"
        #     WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/core/include")
        message(STATUS "Doxygen has been setup and documentation is now available at ${CMAKE_BINARY_DIR}/docs ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.out")
    else ()
        message(STATUS "Doxygen not found. Doxygen need to be installed to generate the doxygen documentation")
    endif ()
endif ()

