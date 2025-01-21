# libosmium
FetchContent_Declare(libosmium
        GIT_REPOSITORY https://github.com/osmcode/libosmium.git
        GIT_TAG v2.21.0
        EXCLUDE_FROM_ALL)

set(BUILD_EXAMPLES OFF CACHE BOOL "do not build libosmium examples" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "do not build libosmium tests" FORCE)

FetchContent_GetProperties(libosmium)
if (NOT libosmium_POPULATED)
    FetchContent_Populate(libosmium)
endif ()

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${libosmium_SOURCE_DIR}/cmake" CACHE STRING "cmake module path" FORCE)
set(Osmium_ROOT "${libosmium_SOURCE_DIR}" CACHE PATH "osmium root path" FORCE)

find_package(Osmium MODULE REQUIRED COMPONENTS io)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${libosmium_SOURCE_DIR}/LICENSE"
            DESTINATION doc/libosmium)
endif ()
