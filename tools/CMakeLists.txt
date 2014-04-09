# This folders contains only independant tools, some of them might have to be compiled

#The command will copy the scripts and allow them to be executed
FILE(GLOB SCRIPTS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/*.R ${CMAKE_CURRENT_SOURCE_DIR}/*.rb)
ADD_CUSTOM_TARGET(copy_scripts
    COMMAND cp ${SCRIPTS_FILE} ${EXECUTABLE_OUTPUT_PATH}
    COMMENT "Copying the scripts"
)

# A viewer for PCD files.
FIND_PACKAGE(PCL 1.2 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})
ADD_EXECUTABLE(pcd_viewer pcd_viewer.cxx)
TARGET_LINK_LIBRARIES(pcd_viewer ${PCL_LIBRARIES})

# Tools will be compiled only if required
SET_TARGET_PROPERTIES(pcd_viewer PROPERTIES EXCLUDE_FROM_ALL true)
SET_TARGET_PROPERTIES(copy_scripts PROPERTIES EXCLUDE_FROM_ALL true)












