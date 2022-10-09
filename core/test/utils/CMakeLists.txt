set(UTEST_SYSTEM_EXCEPT ${CMAKE_CURRENT_SOURCE_DIR}/utest_exception.cpp)
package_add_test_with_libraries(utest_exception "${UTEST_SYSTEM_EXCEPT}"
                                "${CORE_LIBRARY}" "")

set(UTEST_TERMINAL_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_terminal.cpp
                         ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)
package_add_test_with_libraries(utest_terminal "${UTEST_TERMINAL_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_DATE_TIME_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_date_time.cpp
                          ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)
package_add_test_with_libraries(utest_date_time "${UTEST_DATE_TIME_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_SYSTEM_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_system.cpp)
package_add_test_with_libraries(utest_system "${UTEST_SYSTEM_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_LABLED_MODIFIER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_labeld_modifier.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)
package_add_test_with_libraries(utest_labeld_modifier "${UTEST_LABLED_MODIFIER_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_FORMATTER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_formatter.cpp
                          ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)
package_add_test_with_libraries(utest_formatter "${UTEST_FORMATTER_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_WRITTER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_writer.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)
package_add_test_with_libraries(utest_writer "${UTEST_WRITTER_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_LOGGER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_logger.cpp
                       ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)
package_add_test_with_libraries(utest_logger "${UTEST_LOGGER_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_NODE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_node.cpp
                     ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)
package_add_test_with_libraries(utest_node "${UTEST_NODE_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_NODE_LOGGER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_node_logger.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)
package_add_test_with_libraries(utest_node_logger "${UTEST_NODE_LOGGER_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_MODULE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_module.cpp)
package_add_test_with_libraries(utest_module "${UTEST_MODULE_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_CASH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_cash.cpp
                     ${CMAKE_CURRENT_SOURCE_DIR}/utils_data.cpp)
package_add_test_with_libraries(utest_cash "${UTEST_CASH_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_MATH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_math.cpp
                     ${CMAKE_CURRENT_SOURCE_DIR}/utils_data.cpp)
package_add_test_with_libraries(utest_math "${UTEST_MATH_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_DATA_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_data.cpp
                     ${CMAKE_CURRENT_SOURCE_DIR}/utils_data.cpp)
package_add_test_with_libraries(utest_data "${UTEST_DATA_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_SUBJECT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_subject.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/utils_data.cpp)
package_add_test_with_libraries(utest_subject "${UTEST_SUBJECT_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_DATA_LOGGER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_data_logger.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)
package_add_test_with_libraries(utest_data_logger "${UTEST_DATA_LOGGER_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_DATA_STRUCT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_data_struct.cpp
                            ${CMAKE_CURRENT_SOURCE_DIR}/utils_data.cpp)
package_add_test_with_libraries(utest_data_struct "${UTEST_DATA_STRUCT_FILES}"
                                "${CORE_LIBRARY}" "")
