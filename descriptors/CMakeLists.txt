FILE(GLOB SRC_DESCRIPTORS src/*.cpp)
add_library(descriptors src/shape_distribution.cpp src/histogram.cpp)
target_link_libraries(descriptors mesh ${Boost_LIBRARIES})
set_target_properties(descriptors PROPERTIES EXCLUDE_FROM_ALL true)

add_executable(distribution src/shape_distribution.cxx ${Mesh_INCLUDE_DIRS})
add_dependencies(distribution descritors mesh)
target_link_libraries(distribution descriptors mesh ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(distance src/distance.cxx ${Mesh_INCLUDE_DIRS})
add_dependencies(distance descritors mesh)
target_link_libraries(distance descriptors ${Boost_LIBRARIES})

add_library(pydescriptors SHARED src/py_descriptors.cpp src/shape_distribution.cpp src/histogram.cpp)
target_link_libraries(pydescriptors mesh ${Boost_LIBRARIES})

set_target_properties(pydescriptors PROPERTIES EXCLUDE_FROM_ALL true)
