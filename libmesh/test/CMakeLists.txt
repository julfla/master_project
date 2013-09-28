FILE(GLOB TEST_SRC *.cpp)
add_executable(test_runner test_runner.cxx ${TEST_SRC} ${SRC})
target_link_libraries(test_runner ${CPPUNIT_LIBRARY} ${Boost_LIBRARIES})
