include(FetchContent)

# date
FetchContent_Declare(date
        GIT_REPOSITORY https://github.com/HowardHinnant/date.git
        GIT_TAG v3.0.1)

FetchContent_GetProperties(date)
if (NOT date_POPULATED)
    FetchContent_Populate(date)
endif ()

add_subdirectory("${date_SOURCE_DIR}" "${date_BINARY_DIR}" EXCLUDE_FROM_ALL)

# csv parser
FetchContent_Declare(csv_parser
        GIT_REPOSITORY https://github.com/vincentlaucsb/csv-parser.git
        GIT_TAG 2.1.3)

FetchContent_GetProperties(csv_parser)
if (NOT csv_parser_POPULATED)
    FetchContent_Populate(csv_parser)
endif ()

add_subdirectory("${csv_parser_SOURCE_DIR}" "${csv_parser_BINARY_DIR}" EXCLUDE_FROM_ALL)
set(CSV_PARSER_INCLUDE_DIRS "${csv_parser_SOURCE_DIR}/single_include" CACHE PATH "csv parser include dir" FORCE)

# gpx
FetchContent_Declare(gpx
        GIT_REPOSITORY https://github.com/irdvo/gpxlib.git)

FetchContent_GetProperties(gpx)
if (NOT gpx_POPULATED)
    FetchContent_Populate(gpx)
endif ()

set(BUILD_EXAMPLES OFF CACHE BOOL "do not build gpx examples" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "do not build gpx tests" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "disable shared libs" FORCE)

# fix for wrong directory in library when using as subdirectory
FILE(READ "${gpx_SOURCE_DIR}/CMakeLists.txt" gpx_CMAKE_LISTS)
STRING(REGEX REPLACE "{CMAKE_SOURCE_DIR}" "{CMAKE_CURRENT_SOURCE_DIR}" gpx_CMAKE_LISTS_MOD "${gpx_CMAKE_LISTS}")
STRING(REGEX REPLACE "{CMAKE_BINARY_DIR}" "{CMAKE_CURRENT_BINARY_DIR}" gpx_CMAKE_LISTS_MOD "${gpx_CMAKE_LISTS_MOD}")
FILE(WRITE "${gpx_SOURCE_DIR}/CMakeLists.txt" "${gpx_CMAKE_LISTS_MOD}")

add_subdirectory("${gpx_SOURCE_DIR}" "${gpx_BINARY_DIR}" EXCLUDE_FROM_ALL)
set(GPX_INCLUDE_DIRS "${gpx_SOURCE_DIR}" CACHE PATH "gpx include dir" FORCE)

## google sparsehash
#FetchContent_Declare(sparsehash_c11
#        GIT_REPOSITORY https://github.com/sparsehash/sparsehash-c11.git
#        GIT_TAG v2.11.1)
#
#FetchContent_GetProperties(sparsehash_c11)
#if (NOT sparsehash_c11_POPULATED)
#    FetchContent_Populate(sparsehash_c11)
#endif ()
#
#add_subdirectory("${sparsehash_c11_SOURCE_DIR}" "${sparsehash_c11_BINARY_DIR}" EXCLUDE_FROM_ALL)
#set(SPARSEHASH_INCLUDE_DIRS "${sparsehash_c11_SOURCE_DIR}" CACHE PATH "sparsehash include dir" FORCE)

# parallel hash map
#FetchContent_Declare(parallel_hashmap
#        GIT_REPOSITORY https://github.com/greg7mdp/parallel-hashmap.git
#        GIT_TAG 1.33)
#
#FetchContent_MakeAvailable(parallel_hashmap)
#set(PHMAP_INCLUDE_DIRS "${parallel_hashmap_SOURCE_DIR}/parallel_hashmap" CACHE PATH "parallel hashmap include dir" FORCE)

# rpmalloc
FetchContent_Declare(rpmalloc
        GIT_REPOSITORY https://github.com/mjansson/rpmalloc.git
        GIT_TAG 1.4.3)

FetchContent_GetProperties(rpmalloc)
if (NOT rpmalloc_POPULATED)
    FetchContent_Populate(rpmalloc)
endif ()

file(WRITE "${rpmalloc_SOURCE_DIR}/CMakeLists.txt"
        [[
project(rpmalloc
        VERSION 1.4.3
        DESCRIPTION "rpmalloc"
        LANGUAGES C)

add_library(rpmalloc STATIC)

target_compile_definitions(rpmalloc
        PRIVATE
        _GNU_SOURCE
        ENABLE_PRELOAD=1
        ENABLE_OVERRIDE=1
        DEFAULT_SPAN_MAP_COUNT=256)

target_sources(rpmalloc
        PRIVATE
        rpmalloc/rpmalloc.c)

target_include_directories(rpmalloc
        PUBLIC
        rpmalloc)

target_link_libraries(rpmalloc
        PRIVATE
        ${CMAKE_DL_LIBS})
]])

add_subdirectory("${rpmalloc_SOURCE_DIR}" "${rpmalloc_BINARY_DIR}" EXCLUDE_FROM_ALL)
set(RPMALLOC_INCLUDE_DIRS "${rpmalloc_SOURCE_DIR}/rpmalloc" CACHE PATH "rpmalloc include dir" FORCE)

# boost
FetchContent_Declare(boost
        URL https://boostorg.jfrog.io/artifactory/main/release/1.78.0/source/boost_1_78_0.tar.gz
        URL_HASH SHA256=94ced8b72956591c4775ae2207a9763d3600b30d9d7446562c552f0a14a63be7)

FetchContent_GetProperties(boost)
if (NOT boost_POPULATED)
    FetchContent_Populate(boost)
endif ()

if (NOT EXISTS "${boost_SOURCE_DIR}/b2")
    execute_process(COMMAND ./bootstrap.sh --prefix="${boost_BINARY_DIR}"
            WORKING_DIRECTORY "${boost_SOURCE_DIR}")
endif ()

set(BOOST_ROOT "${boost_BINARY_DIR}" CACHE PATH "Boost root" FORCE)
set(Boost_NO_SYSTEM_PATHS ON CACHE BOOL "Boost no system paths" FORCE)

find_package(Boost QUIET CONFIG HINTS "${boost_BINARY_DIR}" NO_DEFAULT_PATH)

if (NOT Boost_FOUND)
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(_variant "debug")
    else ()
        set(_variant "release")
    endif ()
    message(STATUS "Set boost variant to ${_variant}")

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(_toolset "gcc")
    endif ()
    message(STATUS "Set boost toolset to ${_toolset}")

    execute_process(COMMAND ./b2 install link=shared threading=multi variant=release toolset=${_toolset}
            WORKING_DIRECTORY "${boost_SOURCE_DIR}")
endif ()

find_package(Boost QUIET REQUIRED CONFIG HINTS "${boost_BINARY_DIR}" NO_DEFAULT_PATH)

# geometry develop for closest points feature
FetchContent_Declare(geometry_develop
        GIT_REPOSITORY https://github.com/boostorg/geometry.git
        GIT_TAG develop)

FetchContent_MakeAvailable(geometry_develop)
set(GEOMETRY_DEVELOP_INCLUDE_DIRS "${geometry_develop_SOURCE_DIR}/include" CACHE PATH "shortest points new include dir" FORCE)

# apply shortest points new patch file
message(STATUS "Git reset ${geometry_develop_SOURCE_DIR}")
execute_process(COMMAND git reset --hard
        WORKING_DIRECTORY "${geometry_develop_SOURCE_DIR}")
message(STATUS "Git apply rtree_deserialization.patch")
execute_process(COMMAND git apply -v "${CMAKE_CURRENT_SOURCE_DIR}/rtree_deserialization.patch"
        WORKING_DIRECTORY "${geometry_develop_SOURCE_DIR}")

# protozero
FetchContent_Declare(protozero
        GIT_REPOSITORY https://github.com/mapbox/protozero.git
        GIT_TAG v1.7.0)

FetchContent_GetProperties(protozero)
if (NOT protozero_POPULATED)
    FetchContent_Populate(protozero)
endif ()

set(PROTOZERO_INCLUDE_DIR "${protozero_SOURCE_DIR}/include" CACHE PATH "protozero include dir" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "do not build protozero tests" FORCE)
add_subdirectory("${protozero_SOURCE_DIR}" "${protozero_BINARY_DIR}" EXCLUDE_FROM_ALL)

# libosmium
FetchContent_Declare(libosmium
        GIT_REPOSITORY https://github.com/osmcode/libosmium.git
        GIT_TAG v2.17.0)

FetchContent_GetProperties(libosmium)
if (NOT libosmium_POPULATED)
    FetchContent_Populate(libosmium)
endif ()

list(APPEND CMAKE_MODULE_PATH "${libosmium_SOURCE_DIR}/cmake")
set(OSMIUM_INCLUDE_DIR "${libosmium_SOURCE_DIR}" CACHE PATH "osmium include dir" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "do not build libosmium examples" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "do not build libosmium tests" FORCE)

#add_subdirectory("${libosmium_SOURCE_DIR}" "${libosmium_BINARY_DIR}" EXCLUDE_FROM_ALL)

find_package(Osmium QUIET REQUIRED MODULE)
