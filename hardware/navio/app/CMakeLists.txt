# add executables
add_executable(imu_app imu_app.cpp)

# include include folder
target_include_directories(imu_app PRIVATE ${CMAKE_SOURCE_DIR}/hardware/navio/include)

# Should be linked to the main library
target_link_libraries(imu_app PRIVATE ${CORE_LIBRARY} ${NAVIO_LIBRARY} pthread)

# add executables
add_executable(mpu_app mpu_app.cpp)

# include include folder
target_include_directories(mpu_app PRIVATE ${CMAKE_SOURCE_DIR}/hardware/navio/include)

# Should be linked to the main library
target_link_libraries(mpu_app PRIVATE ${CORE_LIBRARY} ${NAVIO_LIBRARY})

# add executables
add_executable(my_mpu_app my_mpu_app.cpp)

# include include folder
target_include_directories(my_mpu_app PRIVATE ${CMAKE_SOURCE_DIR}/hardware/navio/include)

# Should be linked to the main library
target_link_libraries(my_mpu_app PRIVATE ${CORE_LIBRARY} ${NAVIO_LIBRARY} pthread)

# add executables
add_executable(my_mpu_server my_mpu_server.cpp)

# include include folder
target_include_directories(my_mpu_server PRIVATE ${CMAKE_SOURCE_DIR}/hardware/navio/include)

# Should be linked to the main library
target_link_libraries(my_mpu_server PRIVATE ${CORE_LIBRARY} ${NAVIO_LIBRARY} pthread)

install(
    TARGETS imu_app my_mpu_app my_mpu_server
    EXPORT ${TARGETS_EXPORT_NAME}
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)
