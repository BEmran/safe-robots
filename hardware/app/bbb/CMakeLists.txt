# app library
add_library(app STATIC app.cpp app_utils.cpp)
target_include_directories(app PRIVATE ${CMAKE_SOURCE_DIR}/hardware/include
                                       ${CMAKE_SOURCE_DIR}/hardware/app/bbb)
target_link_libraries(app PRIVATE ${CORE_LIBRARY} ${HARDWARE_COMMON_LIBRARY}
                                  ${BBB_LIBRARY} ${DEPEN_LIBRARY} pthread)

# mpu executables
add_executable(bbb_imu_app bbb_imu_app.cpp)
target_include_directories(
    bbb_imu_app PRIVATE ${CMAKE_SOURCE_DIR}/hardware/include
                        ${CMAKE_SOURCE_DIR}/hardware/app/bbb)
target_link_libraries(bbb_imu_app PRIVATE ${CORE_LIBRARY} app pthread)

# server
add_executable(bbb_imu_server bbb_imu_server.cpp)
target_include_directories(
    bbb_imu_server PRIVATE ${CMAKE_SOURCE_DIR}/hardware/include
                           ${CMAKE_SOURCE_DIR}/hardware/app/bbb)
target_link_libraries(bbb_imu_server PRIVATE ${CORE_LIBRARY} app pthread)

install(
    TARGETS bbb_imu_app bbb_imu_server
    EXPORT ${TARGETS_EXPORT_NAME}
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)
