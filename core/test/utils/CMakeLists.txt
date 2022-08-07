set(UTEST_SYSTEM_EXCEPT ${CMAKE_CURRENT_SOURCE_DIR}/utest_exception.cpp)
package_add_test_with_libraries(utest_exception ${UTEST_SYSTEM_EXCEPT}
                                ${CORE_LIBRARY} "")

set(UTEST_TERMINAL_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_terminal.cpp)
package_add_test_with_libraries(utest_terminal ${UTEST_TERMINAL_FILES}
                                ${CORE_LIBRARY} "")

set(UTEST_DATE_TIME_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_date_time.cpp)
package_add_test_with_libraries(utest_date_time ${UTEST_DATE_TIME_FILES}
                                ${CORE_LIBRARY} "")

set(UTEST_SYSTEM_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_system.cpp)
package_add_test_with_libraries(utest_system ${UTEST_SYSTEM_FILES}
                                ${CORE_LIBRARY} "")

set(UTEST_EVENT_LEVEL_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_event_level.cpp)
package_add_test_with_libraries(utest_event_level ${UTEST_EVENT_LEVEL_FILES}
                                ${CORE_LIBRARY} "")

set(UTEST_FORMATTER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_formatter.cpp)
package_add_test_with_libraries(utest_formatter ${UTEST_FORMATTER_FILES}
                                ${CORE_LIBRARY} "")

set(UTEST_WRITTER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_writter.cpp)
package_add_test_with_libraries(utest_writter ${UTEST_WRITTER_FILES}
                                ${CORE_LIBRARY} "")

set(UTEST_LOGGER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_logger.cpp)
package_add_test_with_libraries(utest_logger ${UTEST_LOGGER_FILES}
                                ${CORE_LIBRARY} "")

set(UTEST_NODE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_node.cpp)
package_add_test_with_libraries(utest_node ${UTEST_NODE_FILES} ${CORE_LIBRARY}
                                "")

set(UTEST_MODULE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utest_module.cpp)
package_add_test_with_libraries(utest_module ${UTEST_MODULE_FILES}
                                ${CORE_LIBRARY} "")
