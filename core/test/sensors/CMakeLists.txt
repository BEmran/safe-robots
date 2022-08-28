set(UTEST_SENSOR_LIBRARIES ${CORE_LIBRARY} ${SENSORS_LIBRARY})

set(UTEST_MODULE_SENSOR_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/utest_module_sensor.cpp)
package_add_test_with_libraries(
    utest_module_sensor "${UTEST_MODULE_SENSOR_FILES}"
    "${UTEST_SENSOR_LIBRARIES}" "")

set(UTEST_MODULE_SENSOR_IMU_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/utest_module_sensor_imu.cpp)
package_add_test_with_libraries(
    utest_module_sensor_imu "${UTEST_MODULE_SENSOR_IMU_FILES}"
    "${UTEST_SENSOR_LIBRARIES}" "")
