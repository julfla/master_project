FILE(GLOB SRC_LIBMESH src/*.cpp)

add_library(mesh SHARED ${SRC_LIBMESH})
target_link_libraries(mesh ${Boost_LIBRARIES})

set_target_properties(mesh PROPERTIES EXCLUDE_FROM_ALL true)
