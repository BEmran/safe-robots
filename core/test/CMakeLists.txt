# ##############################################################################
# System Dependencies
# ##############################################################################
# find googletest library
find_package(GTest REQUIRED)
if (NOT GTest_FOUND)
    message(FATAL_ERROR "Google Test library [gtest] not found")
endif ()

include(GoogleTest)
set(GOOGLE_LIBRARIES gtest gtest_main)

# ##############################################################################
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/AddTestWithLibraries.cmake)

# ##############################################################################
set(UTEST_SYSTEM_EXCEPT ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_exception.cpp)
package_add_test_with_libraries(utest_exception "${UTEST_SYSTEM_EXCEPT}"
                                "${CORE_LIBRARY}" "")

set(UTEST_TERMINAL_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_terminal.cpp
                         ${CMAKE_CURRENT_SOURCE_DIR}/utils/utils.cpp)
package_add_test_with_libraries(utest_terminal "${UTEST_TERMINAL_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_DATE_TIME_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_date_time.cpp
                          ${CMAKE_CURRENT_SOURCE_DIR}/utils/utils.cpp)
package_add_test_with_libraries(utest_date_time "${UTEST_DATE_TIME_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_SYSTEM_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_system.cpp)
package_add_test_with_libraries(utest_system "${UTEST_SYSTEM_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_EVENT_LEVEL_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_event_level.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/utils.cpp)
package_add_test_with_libraries(utest_event_level "${UTEST_EVENT_LEVEL_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_FORMATTER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_formatter.cpp
                          ${CMAKE_CURRENT_SOURCE_DIR}/utils/utils.cpp)
package_add_test_with_libraries(utest_formatter "${UTEST_FORMATTER_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_WRITTER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_writer.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/utils/utils.cpp)
package_add_test_with_libraries(utest_writer "${UTEST_WRITTER_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_LOGGER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_logger.cpp
                       ${CMAKE_CURRENT_SOURCE_DIR}/utils/utils.cpp)
package_add_test_with_libraries(utest_logger "${UTEST_LOGGER_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_NODE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_node.cpp
                     ${CMAKE_CURRENT_SOURCE_DIR}/utils/utils.cpp)
package_add_test_with_libraries(utest_node "${UTEST_NODE_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_MODULE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_module.cpp)
package_add_test_with_libraries(utest_module "${UTEST_MODULE_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_CASH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_cash.cpp
                     ${CMAKE_CURRENT_SOURCE_DIR}/utils/utils_data.cpp)
package_add_test_with_libraries(utest_cash "${UTEST_CASH_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_MATH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_math.cpp)
package_add_test_with_libraries(utest_math "${UTEST_MATH_FILES}"
                                "${CORE_LIBRARY}" "")

set(UTEST_DATA_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_data.cpp
                     ${CMAKE_CURRENT_SOURCE_DIR}/utils/utils_data.cpp)
package_add_test_with_libraries(utest_data "${UTEST_DATA_FILES}"
                                "${CORE_LIBRARY}" "")

# ##############################################################################
set(UTEST_MODULE_SENSOR_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/sensors/utest_module_sensor.cpp)
package_add_test_with_libraries(
    utest_module_sensor "${UTEST_MODULE_SENSOR_FILES}" "${CORE_LIBRARY}" "")

set(UTEST_MODULE_SENSOR_IMU_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/sensors/utest_module_sensor_imu.cpp)
package_add_test_with_libraries(
    utest_module_sensor_imu "${UTEST_MODULE_SENSOR_IMU_FILES}"
    "${CORE_LIBRARY}" "")
