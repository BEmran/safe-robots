# add executables
add_executable(add_app add_app.cpp)

# include include folder
target_include_directories(add_app PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Should be linked to the main library
target_link_libraries(add_app PRIVATE ${CORE_LIBRARY})

install(
  TARGETS add_app
  EXPORT ${TARGETS_EXPORT_NAME}
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)
