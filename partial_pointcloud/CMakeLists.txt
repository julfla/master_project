#ADD_DEFINITIONS(-std=c99 -O3 -funsigned-char -freg-struct-return -Wall -W -Wshadow -Wstrict-prototypes -Wpointer-arith -Winline)

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(GLM REQUIRED)
#GLFW does not have any FindGLFW.cmake file
FIND_PACKAGE(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED libglfw)

include_directories(${GLFW_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
SET(GL_LIBS ${GLEW_LIBRARY} ${OPENGL_LIBRARY} ${GLFW_LIBRARIES} ${GLM_LIBRARIES})

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE(main test.cpp common/shader.cpp)
TARGET_LINK_LIBRARIES(main ${GL_LIBS} ${PCL_LIBRARIES})
