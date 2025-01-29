# rpmalloc
set(rpmalloc_SOURCE_DIR "${FETCHCONTENT_BASE_DIR}/rpmalloc-src")
if (EXISTS "${rpmalloc_SOURCE_DIR}")
    set(rpmalloc_EXISTS ON)
    set(FETCHCONTENT_FULLY_DISCONNECTED ON)
endif ()

FetchContent_Declare(rpmalloc
        GIT_REPOSITORY https://github.com/mjansson/rpmalloc.git
        GIT_TAG 1.4.5
        SOURCE_SUBDIR this-directory-does-not-exist
        EXCLUDE_FROM_ALL)

FetchContent_MakeAvailable(rpmalloc)

if (NOT rpmalloc_EXISTS)
    file(WRITE "${rpmalloc_SOURCE_DIR}/CMakeLists.txt"
            [[
project(rpmalloc
        VERSION 1.4.5
        DESCRIPTION "rpmalloc"
        LANGUAGES C)

add_library(rpmalloc STATIC)

target_compile_definitions(rpmalloc
        PRIVATE
        _GNU_SOURCE
        ENABLE_OVERRIDE=1
        DEFAULT_SPAN_MAP_COUNT=256
        PUBLIC
        ENABLE_PRELOAD=1)

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
endif ()

add_subdirectory("${rpmalloc_SOURCE_DIR}" "${rpmalloc_BINARY_DIR}" EXCLUDE_FROM_ALL)

set(RPMALLOC_INCLUDE_DIR "${rpmalloc_SOURCE_DIR}/rpmalloc" CACHE PATH "rpmalloc include dir" FORCE)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${rpmalloc_SOURCE_DIR}/LICENSE"
            DESTINATION doc/rpmalloc)
endif ()

unset(FETCHCONTENT_FULLY_DISCONNECTED)
