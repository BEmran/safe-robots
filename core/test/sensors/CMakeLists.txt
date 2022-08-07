set(UTEST_MODULE_SENSOR_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/utest_module_sensor.cpp)
package_add_test_with_libraries(utest_module_sensor
                                ${UTEST_MODULE_SENSOR_FILES} ${CORE_LIBRARY} "")
