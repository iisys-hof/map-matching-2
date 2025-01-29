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
        EXCLUDE_FROM_ALL)

set(CSV_BUILD_PROGRAMS OFF CACHE BOOL "do not build csv-parser programs" FORCE)

FetchContent_MakeAvailable(csv_parser)

set(CSV_INCLUDE_DIR "${csv_parser_SOURCE_DIR}/single_include" CACHE PATH "csv parser include dir" FORCE)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${csv_parser_SOURCE_DIR}/LICENSE"
            DESTINATION doc/csv_parser)
endif ()

unset(FETCHCONTENT_FULLY_DISCONNECTED)
