set(RC_MATH_LIBRARY rc_math)
add_subdirectory(math)

set(MY_MATH_LIBRARY my_math)

set(UTEST_LIBRARIES ${CORE_LIBRARY} ${RC_MATH_LIBRARY} Eigen3::Eigen)

# ##############################################################################
set(UTEST_MATH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_math.cpp
                     ${CMAKE_CURRENT_SOURCE_DIR}/utest_utils.cpp)
package_add_test_with_libraries(utest_math "${UTEST_MATH_FILES}" "${CORE_LIBRARY}" "")

# ##############################################################################
set(UTEST_QUATERNION_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_quaternion.cpp
                           ${CMAKE_CURRENT_SOURCE_DIR}/utest_utils.cpp)
package_add_test_with_libraries(utest_quaternion "${UTEST_QUATERNION_FILES}"
                                "${UTEST_LIBRARIES}" "")

# ##############################################################################
set(UTEST_VECTOR_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_vector.cpp
                       ${CMAKE_CURRENT_SOURCE_DIR}/utest_utils.cpp)
package_add_test_with_libraries(utest_vector "${UTEST_VECTOR_FILES}"
                                "${UTEST_LIBRARIES}" "")

# ##############################################################################
set(UTEST_DCM_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_dcm.cpp
                    ${CMAKE_CURRENT_SOURCE_DIR}/utest_utils.cpp)
package_add_test_with_libraries(utest_dcm "${UTEST_DCM_FILES}"
                                "${UTEST_LIBRARIES}" "")

# ##############################################################################
set(UTEST_TRANSFORMATION_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/utest_transformation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/utest_utils.cpp)
package_add_test_with_libraries(
    utest_transformation "${UTEST_TRANSFORMATION_FILES}" "${UTEST_LIBRARIES}"
    "")

# ##############################################################################
set(MAIN_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp ${UTEST_MATH_FILES} ${UTEST_VECTOR_FILES}
    ${UTEST_QUATERNION_FILES} ${UTEST_DCM_FILES} ${UTEST_TRANSFORMATION_FILES})
package_add_test_with_libraries(utest_main_math "${MAIN_FILES}"
                                "${UTEST_LIBRARIES}" "")
