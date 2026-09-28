# csv parser
set(CSV_BUILD_PROGRAMS OFF CACHE BOOL "do not build csv-parser programs" FORCE)

Managed_FetchContent(csv_parser
        GIT_REPOSITORY https://github.com/vincentlaucsb/csv-parser.git
        GIT_TAG v5.4.0
        GIT_SUBMODULES ""
        SOURCE_SUBDIR this-directory-does-not-exist
        EXCLUDE_FROM_ALL)

add_subdirectory("${csv_parser_SOURCE_DIR}" "${csv_parser_BINARY_DIR}" EXCLUDE_FROM_ALL)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${csv_parser_SOURCE_DIR}/LICENSE"
            DESTINATION doc/csv_parser)
endif ()
