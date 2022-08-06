# Ensures that we do an out of source build

STRING(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}" insource)
GET_FILENAME_COMPONENT(PARENTDIR ${CMAKE_BINARY_DIR} PATH)
STRING(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${PARENTDIR}" insourcesubdir)
IF(insource OR insourcesubdir)
    message(FATAL_ERROR
        "In-source builds not allowed.\n"
        "Please make a build directory outside the source directory and run CMake from there.\n"
    )
ENDIF(insource OR insourcesubdir)
