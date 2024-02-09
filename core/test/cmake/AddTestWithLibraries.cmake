# ##############################################################################
# addon dunction to create tests
# ##############################################################################
# This will allow you to quickly and simply add tests.
macro (package_add_test_with_libraries TESTNAME FILES LIBRARIES
       TEST_WORKING_DIRECTORY)

    # create an exectuable in which the tests will be stored
    add_executable(${TESTNAME} "${FILES}")

    # add the binary tree to the search path for include files so that we will
    # find include files
    target_include_directories(
        ${TESTNAME}
        PUBLIC # "${CMAKE_BINARY_DIR}"
               "${PROJECT_SOURCE_DIR}/core/include"
               "${PROJECT_SOURCE_DIR}/core/test/include")

    # link the Google test infrastructure, mocking library, testing libraries
    # and a default main fuction to the test executable. Remove gtest_main if
    # writing your own main function.
    target_link_libraries(${TESTNAME} PUBLIC "${LIBRARIES}" ${GTEST_LIBRARIES}
                                             pthread)

    # All users of this library will need at least C++17
    target_compile_features(${TESTNAME} PUBLIC cxx_std_17)
    set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
    add_test(NAME ${TESTNAME}_test COMMAND ${TESTNAME})

endmacro ()
