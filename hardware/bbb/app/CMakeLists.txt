# # mpu executables
# add_executable(imu_app imu_app.cpp)
# target_include_directories(imu_app
#                            PRIVATE ${CMAKE_SOURCE_DIR}/hardware/bbb/include)
# target_link_libraries(imu_app PRIVATE ${CORE_LIBRARY} ${BBB_LIBRARY} pthread)

# # mpu server executables
# add_executable(imu_server imu_server.cpp)
# target_include_directories(imu_server
#                            PRIVATE ${CMAKE_SOURCE_DIR}/hardware/bbb/include)
# target_link_libraries(imu_server PRIVATE ${CORE_LIBRARY} ${BBB_LIBRARY}
#                                          pthread)

# # rc mpu executables
# add_executable(rc_calibrate_accel rc_calibrate_accel.cpp)
# target_include_directories(rc_calibrate_accel
#                            PRIVATE ${CMAKE_SOURCE_DIR}/hardware/bbb/include)
# target_link_libraries(rc_calibrate_accel PRIVATE ${CORE_LIBRARY} ${BBB_LIBRARY}
#                                          pthread)

# # rc mpu executables
# add_executable(rc_calibrate_gyro rc_calibrate_gyro.cpp)
# target_include_directories(rc_calibrate_gyro
#                            PRIVATE ${CMAKE_SOURCE_DIR}/hardware/bbb/include)
# target_link_libraries(rc_calibrate_gyro PRIVATE ${CORE_LIBRARY} ${BBB_LIBRARY}
#                                          pthread)
                                         
# # rc mpu executables
# add_executable(rc_calibrate_mag rc_calibrate_mag.cpp)
# target_include_directories(rc_calibrate_mag
#                            PRIVATE ${CMAKE_SOURCE_DIR}/hardware/bbb/include)
# target_link_libraries(rc_calibrate_mag PRIVATE ${CORE_LIBRARY} ${BBB_LIBRARY}
#                                          pthread)

    # rc mpu executables
add_executable(rc_simple mpu/rc_mpu.cpp mpu/i2c.cpp mpu/mpu.cpp)                      
# rc mpu executables
install(
    # TARGETS imu_app imu_server rc_mpu rc_calibrate_accel rc_calibrate_gyro rc_calibrate_mag
    TARGETS rc_lib_mpu rc_simple
    EXPORT ${TARGETS_EXPORT_NAME}
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)
