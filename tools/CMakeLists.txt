#The command will copy the scripts and allow them to be executed
FILE(GLOB SCRIPTS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/*.R ${CMAKE_CURRENT_SOURCE_DIR}/*.rb)
add_custom_target(copy_scripts
    COMMAND cp ${SCRIPTS_FILE} ${EXECUTABLE_OUTPUT_PATH}
    COMMENT "Copying the scripts"
)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE(pcd_viewer pcd_viewer.cxx)
TARGET_LINK_LIBRARIES(pcd_viewer ${PCL_LIBRARIES})












