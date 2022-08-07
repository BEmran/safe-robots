# add executables
add_executable(logger_app logger_app.cpp)

# include include folder
target_include_directories(logger_app PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Should be linked to the main library
target_link_libraries(logger_app PRIVATE ${CORE_LIBRARY})

install(
    TARGETS logger_app
    EXPORT ${TARGETS_EXPORT_NAME}
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)
