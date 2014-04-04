CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/module)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/lib)

INCLUDE( ExternalProject )
ExternalProject_Add(skp2tri
	SOURCE_DIR ${PROJECT_SOURCE_DIR}/skp2tri
	CMAKE_ARGS
		-DCMAKE_TOOLCHAIN_FILE=${PROJECT_SOURCE_DIR}/skp2tri/Toolchain-windows-mingw32.cmake
		-DEXECUTABLE_OUTPUT_PATH=${EXECUTABLE_OUTPUT_PATH}
	INSTALL_COMMAND ""
)

# Project common include dir
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

# Add all subprojects
ADD_SUBDIRECTORY(mesh)
ADD_SUBDIRECTORY(descriptors)
ADD_SUBDIRECTORY(partial_pointcloud)
ADD_SUBDIRECTORY(tools)

# The tests cannot compiled for now on...

# Find requirements and sources for tests
# FIND_PACKAGE(CppUnit REQUIRED)
# IF( NOT CPPUNIT_FOUND )
#     MESSAGE(FATAL_ERROR "Module CppUnit required and not found.")
# ENDIF()
# INCLUDE_DIRECTORIES(${CPPUNIT_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR})
# FILE(GLOB SRC_TEST */test/*.cpp)
# FILE(GLOB SRC */src/*.cpp)
# ADD_EXECUTABLE(test EXCLUDE_FROM_ALL test_runner.cxx ${SRC_TEST} ${SRC})
# target_link_libraries(test ${CPPUNIT_LIBRARY} ${Boost_LIBRARIES})
