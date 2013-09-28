#The command will copy the scripts and allow them to be executed
FILE(GLOB SCRIPTS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/*.R ${CMAKE_CURRENT_SOURCE_DIR}/*.rb)
add_custom_target(copy_scripts
    COMMAND cp ${SCRIPTS_FILE} ${EXECUTABLE_OUTPUT_PATH}
    COMMENT "Copying the scripts"
)












