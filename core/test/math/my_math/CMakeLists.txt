set(MY_MATH_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(MY_MATH_SOURCE_LIST
    ${MY_MATH_SOURCE_DIR}/dcm.cpp
    ${MY_MATH_SOURCE_DIR}/quaternion.cpp
    ${MY_MATH_SOURCE_DIR}/transformation.cpp
    ${MY_MATH_SOURCE_DIR}/vector.cpp)

# ##############################################################################
# Build core library
# ##############################################################################
add_library(${MY_MATH_LIBRARY} ${MY_MATH_SOURCE_LIST})
target_link_libraries(${MY_MATH_LIBRARY} ${CORE_LIBRARY})

# Global includes
target_include_directories(
    ${MY_MATH_LIBRARY} PUBLIC $<INSTALL_INTERFACE:${ABSOLUTE_INSTALL_INCLUDEDIR}>)

