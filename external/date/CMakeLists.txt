# date
FetchContent_Declare(date
        GIT_REPOSITORY https://github.com/HowardHinnant/date.git
        GIT_TAG v3.0.1
        EXCLUDE_FROM_ALL)

FetchContent_MakeAvailable(date)

install(FILES
        "${date_SOURCE_DIR}/LICENSE.txt"
        DESTINATION doc/date)
