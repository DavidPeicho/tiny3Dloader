add_executable(check main.cpp)

add_custom_command(TARGET check PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/models ${EXECUTABLE_OUTPUT_PATH}/models)

TARGET_INCLUDE_DIRECTORIES(check PRIVATE ${CMAKE_SOURCE_DIR}/includes)