# protozero
set(protozero_SOURCE_DIR "${FETCHCONTENT_BASE_DIR}/protozero-src")
if (EXISTS "${protozero_SOURCE_DIR}")
    set(protozero_EXISTS ON)
    set(FETCHCONTENT_FULLY_DISCONNECTED ON)
endif ()

FetchContent_Declare(protozero
        GIT_REPOSITORY https://github.com/mapbox/protozero.git
        GIT_TAG v1.8.1
        EXCLUDE_FROM_ALL)

set(BUILD_TESTING OFF CACHE BOOL "do not build protozero tests" FORCE)

FetchContent_MakeAvailable(protozero)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${protozero_SOURCE_DIR}/cmake" CACHE STRING "cmake module path" FORCE)
set(Protozero_ROOT "${protozero_SOURCE_DIR}" CACHE PATH "protozero root path" FORCE)

find_package(Protozero MODULE REQUIRED)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${protozero_SOURCE_DIR}/LICENSE.md"
            DESTINATION doc/protozero)
endif ()

unset(FETCHCONTENT_FULLY_DISCONNECTED)
