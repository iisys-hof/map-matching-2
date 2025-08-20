# csv parser
set(csv_parser_SOURCE_DIR "${FETCHCONTENT_BASE_DIR}/csv_parser-src")
if (EXISTS "${csv_parser_SOURCE_DIR}")
    set(csv_parser_EXISTS ON)
    set(FETCHCONTENT_FULLY_DISCONNECTED ON)
endif ()

FetchContent_Declare(csv_parser
        GIT_REPOSITORY https://github.com/vincentlaucsb/csv-parser.git
        GIT_TAG 2.3.0
        GIT_SUBMODULES ""
        SOURCE_SUBDIR this-directory-does-not-exist
        EXCLUDE_FROM_ALL)

set(CSV_BUILD_PROGRAMS OFF CACHE BOOL "do not build csv-parser programs" FORCE)

FetchContent_MakeAvailable(csv_parser)

if (NOT csv_parser_EXISTS)
    # fix deprecated cmake version
    FILE(READ "${csv_parser_SOURCE_DIR}/CMakeLists.txt" csv_parser_CMAKE_LISTS)
    STRING(REGEX REPLACE "cmake_minimum_required\\(VERSION [0-9\\.]+\\)" "cmake_minimum_required(VERSION 3.10)" csv_parser_CMAKE_LISTS "${csv_parser_CMAKE_LISTS}")
    FILE(WRITE "${csv_parser_SOURCE_DIR}/CMakeLists.txt" "${csv_parser_CMAKE_LISTS}")
endif ()

add_subdirectory("${csv_parser_SOURCE_DIR}" "${csv_parser_BINARY_DIR}" EXCLUDE_FROM_ALL)

set(CSV_INCLUDE_DIR "${csv_parser_SOURCE_DIR}/single_include" CACHE PATH "csv parser include dir" FORCE)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${csv_parser_SOURCE_DIR}/LICENSE"
            DESTINATION doc/csv_parser)
endif ()

unset(FETCHCONTENT_FULLY_DISCONNECTED)
