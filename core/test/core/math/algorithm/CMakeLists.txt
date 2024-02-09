# ##############################################################################
set(UTEST_COMPLEMENTARY_FILTER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/utest_complementary_filter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../math_utils.cpp)
message(STATUS "${UTEST_COMPLEMENTARY_FILTER_FILES}")
package_add_test_with_libraries(
    utest_complementary_filter "${UTEST_COMPLEMENTARY_FILTER_FILES}"
    "${CORE_LIBRARY}" "")
