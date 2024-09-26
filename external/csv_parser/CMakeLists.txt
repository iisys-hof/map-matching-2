# csv parser
FetchContent_Declare(csv_parser
        GIT_REPOSITORY https://github.com/vincentlaucsb/csv-parser.git
        GIT_TAG 2.3.0
        EXCLUDE_FROM_ALL)

set(CSV_BUILD_PROGRAMS OFF CACHE BOOL "do not build csv-parser programs" FORCE)

FetchContent_MakeAvailable(csv_parser)

set(CSV_INCLUDE_DIR "${csv_parser_SOURCE_DIR}/single_include" CACHE PATH "csv parser include dir" FORCE)

install(FILES
        "${csv_parser_SOURCE_DIR}/LICENSE"
        DESTINATION doc/csv_parser)
