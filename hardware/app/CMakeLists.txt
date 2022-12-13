# mpu executables
add_executable(imu_app imu_app.cpp)
target_include_directories(imu_app
                           PRIVATE ${CMAKE_SOURCE_DIR}/hardware/navio/include)
target_link_libraries(imu_app PRIVATE ${CORE_LIBRARY} ${NAVIO_LIBRARY} pthread)

# mpu server executables
add_executable(imu_server imu_server.cpp)
target_include_directories(imu_server
                           PRIVATE ${CMAKE_SOURCE_DIR}/hardware/navio/include)
target_link_libraries(imu_server PRIVATE ${CORE_LIBRARY} ${NAVIO_LIBRARY}
                                         pthread)

install(
    TARGETS imu_app imu_server
    EXPORT ${TARGETS_EXPORT_NAME}
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)
