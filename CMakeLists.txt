cmake_minimum_required(VERSION 3.16)

project(map_matching_2
        VERSION 0.3.0
        DESCRIPTION "Map-Matching 2"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "No developer warnings")

option(ENABLE_TESTS "Enable unit tests" ON)
option(NATIVE_BUILD "Enable native build" OFF)

add_subdirectory(external)

if (${NATIVE_BUILD})
    include(cmake/native.cmake)
    add_compile_options(${_CXX_FLAGS})
endif ()

add_subdirectory(src)

if (${ENABLE_TESTS})
    enable_testing()
    add_subdirectory(tests)
endif ()
