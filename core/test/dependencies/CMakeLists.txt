# ##############################################################################
set(UTEST_YAML_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/utest_yaml.cpp)
package_add_test_with_libraries(
    utest_yaml "${UTEST_YAML_FILES}" "${DEPEN_LIBRARY}" "")

set(UTEST_ENTREE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/utest_entree.cpp)
package_add_test_with_libraries(
    utest_entree "${UTEST_ENTREE_FILES}" "${DEPEN_LIBRARY}" "")
