# bz2
FetchContent_Declare(bz2
        GIT_REPOSITORY https://gitlab.com/bzip2/bzip2.git
        GIT_TAG 66c46b8c9436613fd81bc5d03f63a61933a4dcc3
        EXCLUDE_FROM_ALL)

FetchContent_GetProperties(bz2)
if (NOT bz2_POPULATED)
    FetchContent_Populate(bz2)

    # override to 3.15 for enabling CMAKE_MSVC_RUNTIME_LIBRARY
    FILE(READ "${bz2_SOURCE_DIR}/CMakeLists.txt" bz2_CMAKE_LISTS)
    STRING(REGEX REPLACE "cmake_minimum_required\\(VERSION [0-9\\.]+\\)" "cmake_minimum_required(VERSION 3.15)" bz2_CMAKE_LISTS "${bz2_CMAKE_LISTS}")
    FILE(WRITE "${bz2_SOURCE_DIR}/CMakeLists.txt" "${bz2_CMAKE_LISTS}")

    execute_process(COMMAND ${CMAKE_COMMAND}
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DCMAKE_GENERATOR=${CMAKE_GENERATOR}
            -DCMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}
            -DENABLE_DOCS=OFF
            -DENABLE_TESTS=OFF
            -DENABLE_LIB_ONLY=ON
            -DENABLE_SHARED_LIB=ON
            -DENABLE_STATIC_LIB=ON
            -S "${bz2_SOURCE_DIR}"
            -B "${bz2_BINARY_DIR}")
    execute_process(COMMAND ${CMAKE_COMMAND}
            --build "${bz2_BINARY_DIR}")
endif ()

set(BZip2_ROOT "${bz2_BINARY_DIR};${bz2_SOURCE_DIR}" CACHE PATH "bz2 root path" FORCE)

find_library(BZIP2_LIBRARIES NAMES bz2_static PATHS ${BZip2_ROOT} REQUIRED)
find_package(BZip2 REQUIRED)

install(FILES
        "${bz2_SOURCE_DIR}/COPYING"
        DESTINATION doc/bz2)
