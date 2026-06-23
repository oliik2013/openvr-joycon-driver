cmake_minimum_required(VERSION 3.14)
project(joycon VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

file(GLOB SOURCE_FILES src/*.cc src/*.h)

add_subdirectory(include/external/JSL/source JoyShockLibrary)

add_library(driver_joycon SHARED ${SOURCE_FILES})
set_target_properties(driver_joycon PROPERTIES PREFIX "")
target_include_directories(
    driver_joycon PUBLIC
    include/external
    include
)

target_link_libraries(driver_joycon
    JoyShockLibrary
    pthread
)

add_custom_command(TARGET driver_joycon POST_BUILD 
  COMMAND "${CMAKE_COMMAND}" -E make_directory
     "${CMAKE_CURRENT_SOURCE_DIR}/joycon/bin/linux64"
  COMMAND "${CMAKE_COMMAND}" -E copy 
     "$<TARGET_FILE:driver_joycon>"
     "${CMAKE_CURRENT_SOURCE_DIR}/joycon/bin/linux64/$<TARGET_FILE_NAME:driver_joycon>"
  COMMAND "${CMAKE_COMMAND}" -E copy
     "$<TARGET_FILE:JoyShockLibrary>"
     "${CMAKE_CURRENT_SOURCE_DIR}/joycon/bin/linux64/$<TARGET_FILE_NAME:JoyShockLibrary>")