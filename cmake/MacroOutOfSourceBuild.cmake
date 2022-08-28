# Ensures that we do an out of source build

string(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}" insource)
get_filename_component(PARENTDIR ${CMAKE_BINARY_DIR} PATH)
string(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${PARENTDIR}" insourcesubdir)
if (insource OR insourcesubdir)
    message(
        FATAL_ERROR
            "In-source builds not allowed.\n"
            "Please make a build directory outside the source directory and run CMake from there.\n"
    )
endif (insource OR insourcesubdir)
