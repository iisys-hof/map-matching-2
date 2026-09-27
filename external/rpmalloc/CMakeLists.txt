# rpmalloc
Managed_FetchContent(rpmalloc
        GIT_REPOSITORY https://github.com/mjansson/rpmalloc.git
        GIT_TAG 2.0.1
        SOURCE_SUBDIR this-directory-does-not-exist
        EXCLUDE_FROM_ALL)

if (rpmalloc_NEEDS_REFRESH)
    file(WRITE "${rpmalloc_SOURCE_DIR}/CMakeLists.txt"
            [[
project(rpmalloc
        VERSION 2.0.1
        DESCRIPTION "rpmalloc"
        LANGUAGES C)

add_library(rpmalloc STATIC)

set_target_properties(rpmalloc PROPERTIES
        C_STANDARD 11
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS ON)

target_compile_options(rpmalloc
        PRIVATE
        $<$<C_COMPILER_ID:MSVC>:/experimental:c11atomics>)

target_compile_definitions(rpmalloc
        PRIVATE
        _GNU_SOURCE
        ENABLE_OVERRIDE=1)

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
