cmake_minimum_required(VERSION 2.8)
INCLUDE( ExternalProject )

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/module)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH})

ExternalProject_Add(skp2tri
	SOURCE_DIR ${PROJECT_SOURCE_DIR}/skp2tri
	CMAKE_ARGS
		-DCMAKE_TOOLCHAIN_FILE=${PROJECT_SOURCE_DIR}/skp2tri/Toolchain-windows-mingw32.cmake
		-DEXECUTABLE_OUTPUT_PATH=${EXECUTABLE_OUTPUT_PATH}
	INSTALL_COMMAND ""
	#INSTALL_COMMAND ./bjam install ${Boost_bjam_ARGS}
)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost 1.49.0 COMPONENTS serialization program_options system REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

include_directories(libmesh/src)
add_subdirectory(libmesh)
add_subdirectory(descriptors)
add_subdirectory(tools)
add_subdirectory(partial_pointcloud)


#Find requirements and sources for tests

FIND_PACKAGE(CppUnit REQUIRED)
IF( NOT CPPUNIT_FOUND )
    MESSAGE(FATAL_ERROR "Module CppUnit required and not found.")
ENDIF()
INCLUDE_DIRECTORIES(${CPPUNIT_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR})

FILE(GLOB SRC_TEST */test/*.cpp)
FILE(GLOB SRC */src/*.cpp)
add_executable(test EXCLUDE_FROM_ALL test_runner.cxx ${SRC_TEST} ${SRC})
target_link_libraries(test ${CPPUNIT_LIBRARY} ${Boost_LIBRARIES})
