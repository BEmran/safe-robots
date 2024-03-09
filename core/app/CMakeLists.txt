# add executables
add_executable(logger_app logger_app.cpp)

# include include folder
target_include_directories(logger_app
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Should be linked to the main library
target_link_libraries(logger_app PRIVATE ${CORE_LIBRARY})

# add executables
add_executable(server_app server_app.cpp)

# include include folder
target_include_directories(server_app
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Should be linked to the main library
target_link_libraries(server_app PRIVATE ${CORE_LIBRARY})

# add executables
add_executable(pipeline_app pipeline_app.cpp)

# include include folder
target_include_directories(pipeline_app
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Should be linked to the main library
target_link_libraries(pipeline_app PRIVATE ${CORE_LIBRARY})


# add executables
add_executable(compare_math_app compare_math_app.cpp)

# include include folder
target_include_directories(compare_math_app
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Should be linked to the main library
target_link_libraries(compare_math_app PRIVATE ${CORE_LIBRARY})

# add executables
# add_executable(rc_test_matrix_app rc_test_matrix.cpp)

# include include folder
# target_include_directories(rc_test_matrix_app
#                            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Should be linked to the main library
# target_link_libraries(rc_test_matrix_app PRIVATE ${CORE_LIBRARY})


# add executables
# add_executable(simple_math_app simple_math_app.cpp)

install(
    TARGETS logger_app server_app pipeline_app compare_math_app
    EXPORT ${TARGETS_EXPORT_NAME}
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)
