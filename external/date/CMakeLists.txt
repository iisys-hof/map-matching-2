# date
FetchContent_Declare(date
        GIT_REPOSITORY https://github.com/HowardHinnant/date.git
        GIT_TAG v3.0.2
        EXCLUDE_FROM_ALL)

FetchContent_MakeAvailable(date)

set(ENABLE_DATE_INSTALL OFF CACHE BOOL "disable date install" FORCE)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${date_SOURCE_DIR}/LICENSE.txt"
            DESTINATION doc/date)
endif ()
