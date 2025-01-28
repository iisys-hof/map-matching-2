# gpx
FetchContent_Declare(gpx
        GIT_REPOSITORY https://github.com/irdvo/gpxlib.git
        GIT_TAG 43df92be29412b7f3b58e63ebea516df9f15b883
        SOURCE_SUBDIR this-directory-does-not-exist
        EXCLUDE_FROM_ALL)

set(BUILD_EXAMPLES OFF CACHE BOOL "do not build gpx examples" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "do not build gpx tests" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "disable shared libs" FORCE)

FetchContent_MakeAvailable(gpx)

# fix for wrong directory and deprecated cmake version in library when using as subdirectory
FILE(READ "${gpx_SOURCE_DIR}/CMakeLists.txt" gpx_CMAKE_LISTS)
STRING(REGEX REPLACE "cmake_minimum_required\\(VERSION [0-9\\.]+\\)" "cmake_minimum_required(VERSION 3.5)" gpx_CMAKE_LISTS "${gpx_CMAKE_LISTS}")
STRING(REPLACE "\${CMAKE_SOURCE_DIR}" "\${CMAKE_CURRENT_SOURCE_DIR}" gpx_CMAKE_LISTS "${gpx_CMAKE_LISTS}")
STRING(REPLACE "\${CMAKE_BINARY_DIR}" "\${CMAKE_CURRENT_BINARY_DIR}" gpx_CMAKE_LISTS "${gpx_CMAKE_LISTS}")
FILE(WRITE "${gpx_SOURCE_DIR}/CMakeLists.txt" "${gpx_CMAKE_LISTS}")

# fix deprecated cmake version in subdir
FILE(READ "${gpx_SOURCE_DIR}/gpx/CMakeLists.txt" gpx_CMAKE_LISTS)
STRING(REGEX REPLACE "cmake_minimum_required\\(VERSION [0-9\\.]+\\)" "cmake_minimum_required(VERSION 3.5)" gpx_CMAKE_LISTS "${gpx_CMAKE_LISTS}")
FILE(WRITE "${gpx_SOURCE_DIR}/gpx/CMakeLists.txt" "${gpx_CMAKE_LISTS}")

add_subdirectory("${gpx_SOURCE_DIR}" "${gpx_BINARY_DIR}" EXCLUDE_FROM_ALL)

set(GPX_INCLUDE_DIR "${gpx_SOURCE_DIR}" CACHE PATH "gpx include dir" FORCE)

target_compile_definitions(gpx PRIVATE _USE_MATH_DEFINES)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${gpx_SOURCE_DIR}/COPYING.LESSER"
            DESTINATION doc/gpx)
endif ()
