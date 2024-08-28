# ##############################################################################
set(UTEST_BASIC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_basic.cpp)
package_add_test_with_libraries(utest_basic "${UTEST_BASIC_FILES}" "${CORE_LIBRARY}" "")

set(UTEST_VECTOR2_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_vector2.cpp)
package_add_test_with_libraries(utest_vector2 "${UTEST_VECTOR2_FILES}" "${CORE_LIBRARY}" "")

set(UTEST_VECTOR3_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_vector3.cpp)
package_add_test_with_libraries(utest_vector3 "${UTEST_VECTOR3_FILES}" "${CORE_LIBRARY}" "")

set(UTEST_MATRIX2X2_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_matrix2x2.cpp)
package_add_test_with_libraries(utest_matrix2x2 "${UTEST_MATRIX2X2_FILES}" "${CORE_LIBRARY}" "")

set(UTEST_MATRIX3X3_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_matrix3x3.cpp)
package_add_test_with_libraries(utest_matrix3x3 "${UTEST_MATRIX3X3_FILES}" "${CORE_LIBRARY}" "")

set(UTEST_QUATERNION_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_quaternion.cpp)
package_add_test_with_libraries(utest_quat "${UTEST_QUATERNION_FILES}" "${CORE_LIBRARY}" "")
