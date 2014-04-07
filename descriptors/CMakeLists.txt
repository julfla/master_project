INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include)

FIND_PACKAGE(PCL 1.2 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

FIND_PACKAGE(Boost 1.53.0 COMPONENTS program_options serialization python REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

ADD_LIBRARY(descriptors SHARED src/shape_distribution.cpp src/histogram.cpp)
TARGET_INCLUDE_DIRECTORIES(descriptors PUBLIC ${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
TARGET_LINK_LIBRARIES(descriptors mesh ${Boost_LIBRARIES})

ADD_EXECUTABLE(distribution src/shape_distribution.cxx)
TARGET_LINK_LIBRARIES(distribution descriptors mesh ${Boost_LIBRARIES} ${PCL_LIBRARIES})

ADD_EXECUTABLE(distance src/distance.cxx)
TARGET_LINK_LIBRARIES(distance descriptors ${Boost_LIBRARIES})

# Expose ShapeDistribution to be used with python
FIND_PACKAGE(PythonLibs REQUIRED)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
ADD_LIBRARY(pydescriptors SHARED src/py_descriptors.cpp)
TARGET_LINK_LIBRARIES(pydescriptors descriptors mesh ${Boost_LIBRARIES})

# Libraries do not compile if not needed
SET_TARGET_PROPERTIES(descriptors PROPERTIES EXCLUDE_FROM_ALL true)
SET_TARGET_PROPERTIES(pydescriptors PROPERTIES EXCLUDE_FROM_ALL true)
