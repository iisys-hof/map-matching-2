# libosmium
set(libosmium_SOURCE_DIR "${FETCHCONTENT_BASE_DIR}/libosmium-src")
if (EXISTS "${libosmium_SOURCE_DIR}")
    set(libosmium_EXISTS ON)
    set(FETCHCONTENT_FULLY_DISCONNECTED ON)
endif ()

FetchContent_Declare(libosmium
        GIT_REPOSITORY https://github.com/osmcode/libosmium.git
        GIT_TAG v2.22.0
        GIT_SUBMODULES ""
        SOURCE_SUBDIR this-directory-does-not-exist
        EXCLUDE_FROM_ALL)

set(BUILD_EXAMPLES OFF CACHE BOOL "do not build libosmium examples" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "do not build libosmium tests" FORCE)

FetchContent_MakeAvailable(libosmium)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${libosmium_SOURCE_DIR}/cmake" CACHE STRING "cmake module path" FORCE)
set(Osmium_ROOT "${libosmium_SOURCE_DIR}" CACHE PATH "osmium root path" FORCE)

find_package(Osmium MODULE REQUIRED COMPONENTS io)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${libosmium_SOURCE_DIR}/LICENSE"
            DESTINATION doc/libosmium)
endif ()

unset(FETCHCONTENT_FULLY_DISCONNECTED)
