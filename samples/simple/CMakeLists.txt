add_custom_command(TARGET simple PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/models ${EXECUTABLE_OUTPUT_PATH}/models)

include_directories(${CMAKE_SOURCE_DIR}/includes)
add_executable(simple main.cpp)
