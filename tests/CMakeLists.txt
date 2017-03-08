add_executable(check main.cpp)

add_custom_command(TARGET check PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/models ${EXECUTABLE_OUTPUT_PATH}/models)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/includes)

TARGET_LINK_LIBRARIES(check ${GTEST_LIBRARIES} pthread)