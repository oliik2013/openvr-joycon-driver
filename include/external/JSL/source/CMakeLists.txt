add_library (
    JoyShockLibrary
    JoyShockLibrary.cpp    JoyShockLibrary.h
)

target_include_directories (
    JoyShockLibrary PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

find_package(PkgConfig QUIET REQUIRED)
pkg_search_module(HIDAPI REQUIRED IMPORTED_TARGET hidapi-hidraw)

target_link_libraries (
    JoyShockLibrary PUBLIC
    PkgConfig::HIDAPI
)
