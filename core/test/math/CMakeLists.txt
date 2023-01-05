set(RC_MATH_LIBRARY rc_math)
add_subdirectory(math)

set(MY_MATH_LIBRARY my_math)
add_subdirectory(my_math)

set(UTEST_LIBRARIES ${CORE_LIBRARY} ${RC_MATH_LIBRARY} ${MY_MATH_LIBRARY} Eigen3::Eigen)

# ##############################################################################
set(UTEST_QUATERNION_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_quaternion.cpp)
package_add_test_with_libraries(utest_quaternion "${UTEST_QUATERNION_FILES}" "${UTEST_LIBRARIES}" "")

# ##############################################################################
set(UTEST_VECTOR_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_vector.cpp)
package_add_test_with_libraries(utest_vector "${UTEST_VECTOR_FILES}" "${UTEST_LIBRARIES}" "")

# ##############################################################################
set(UTEST_DCM_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_dcm.cpp)
package_add_test_with_libraries(utest_dcm "${UTEST_DCM_FILES}" "${UTEST_LIBRARIES}" "")
