# mpu executables
add_executable(mpu_app mpu_app.cpp)
target_include_directories(mpu_app
                           PRIVATE ${CMAKE_SOURCE_DIR}/hardware/navio/include)
target_link_libraries(mpu_app PRIVATE ${CORE_LIBRARY} ${NAVIO_LIBRARY} pthread)

# mpu server executables
add_executable(mpu_server mpu_server.cpp)
target_include_directories(mpu_server
                           PRIVATE ${CMAKE_SOURCE_DIR}/hardware/navio/include)
target_link_libraries(mpu_server PRIVATE ${CORE_LIBRARY} ${NAVIO_LIBRARY}
                                         pthread)

install(
    TARGETS mpu_app mpu_server
    EXPORT ${TARGETS_EXPORT_NAME}
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)
