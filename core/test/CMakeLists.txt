# ##############################################################################
# System Dependencies
# ##############################################################################
# find googletest library
find_package(GTest REQUIRED)
if(NOT GTest_FOUND)
  message(FATAL_ERROR "Google Test library [gtest] not found")
endif()

include(GoogleTest)
set(GOOGLE_LIBRARIES gtest gtest_main)

# ##############################################################################
# addon macro to create tests
# ##############################################################################
# This will allow you to quickly and simply add tests.
macro(package_add_test_with_libraries TESTNAME FILES LIBRARIES
      TEST_WORKING_DIRECTORY)

  # create an exectuable in which the tests will be stored
  add_executable(${TESTNAME} ${FILES})

  # add the binary tree to the search path for include files so that we will
  # find include files
  target_include_directories(
    ${TESTNAME}
    PUBLIC # "${CMAKE_BINARY_DIR}"
           "${CMAKE_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include")

  # link the Google test infrastructure, mocking library, testing libraries and
  # a default main fuction to the test executable. Remove gtest_main if writing
  # your own main function.
  target_link_libraries(${TESTNAME} PUBLIC ${LIBRARIES} ${GOOGLE_LIBRARIES}
                                           pthread)

  # # gtest_discover_tests replaces gtest_add_tests, # see
  # https://cmake.org/cmake/help/v3.10/module/GoogleTest.html for more options
  # to pass to it gtest_discover_tests(${TESTNAME} # set a working directory so
  # your project root so that you can find test data via paths relative to the
  # project root WORKING_DIRECTORY ${TEST_WORKING_DIRECTORY} PROPERTIES
  # VS_TRACEGER_WORKING_DIRECTORY "${TEST_WORKING_DIRECTORY}" )

  # All users of this library will need at least C++17
  target_compile_features(${TESTNAME} PUBLIC cxx_std_17)
  set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
  add_test(NAME ${TESTNAME}_test COMMAND ${TESTNAME})

endmacro()
# example of using the macro package_add_test_with_libraries(test1 test1.cpp
# lib_to_test "${PROJECT_DIR}/test-data/")

# ##############################################################################

set(UTEST_SYSTEM_EXCEPT ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_exception.cpp)
package_add_test_with_libraries(utest_exception ${UTEST_SYSTEM_EXCEPT} ${CORE_LIBRARY} "")

set(UTEST_TERMINAL_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_terminal.cpp)
package_add_test_with_libraries(utest_terminal ${UTEST_TERMINAL_FILES} ${CORE_LIBRARY} "")

set(UTEST_DATE_TIME_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_date_time.cpp)
package_add_test_with_libraries(utest_date_time ${UTEST_DATE_TIME_FILES} ${CORE_LIBRARY} "")

set(UTEST_SYSTEM_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_system.cpp)
package_add_test_with_libraries(utest_system ${UTEST_SYSTEM_FILES} ${CORE_LIBRARY} "")

set(UTEST_EVENT_LEVEL_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_event_level.cpp)
package_add_test_with_libraries(utest_event_level ${UTEST_EVENT_LEVEL_FILES} ${CORE_LIBRARY} "")

set(UTEST_FORMATTER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_formatter.cpp)
package_add_test_with_libraries(utest_formatter ${UTEST_FORMATTER_FILES} ${CORE_LIBRARY} "")

set(UTEST_WRITTER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_writter.cpp)
package_add_test_with_libraries(utest_writter ${UTEST_WRITTER_FILES} ${CORE_LIBRARY} "")

set(UTEST_LOGGER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_logger.cpp)
package_add_test_with_libraries(utest_logger ${UTEST_LOGGER_FILES} ${CORE_LIBRARY} "")

set(UTEST_NODE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/utils/utest_node.cpp)
package_add_test_with_libraries(utest_node ${UTEST_NODE_FILES} ${CORE_LIBRARY}  "")
