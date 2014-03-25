FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(GLM REQUIRED)
#GLFW does not have any FindGLFW.cmake file
FIND_PACKAGE(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED libglfw)

include_directories(${GLFW_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
SET(GL_LIBS ${GLEW_LIBRARY} ${OPENGL_LIBRARY} ${GLFW_LIBRARIES} ${GLM_LIBRARIES} )

ADD_EXECUTABLE(partial_view partial_view.cxx partial_view.cpp)
add_dependencies(partial_view descritors)
TARGET_LINK_LIBRARIES(partial_view ${GL_LIBS} ${PCL_LIBRARIES} descriptors)

add_library(pypartialview SHARED py_partial_view.cpp partial_view.cpp)
target_link_libraries(pypartialview ${GL_LIBS} ${PCL_LIBRARIES}  ${Boost_LIBRARIES} descriptors)

set_target_properties(pypartialview PROPERTIES EXCLUDE_FROM_ALL true)
