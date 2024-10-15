# zlib
FetchContent_Declare(ZLIB
        GIT_REPOSITORY https://github.com/madler/zlib.git
        GIT_TAG v1.3.1
        EXCLUDE_FROM_ALL)

FetchContent_GetProperties(zlib)
if (NOT zlib_POPULATED)
    FetchContent_Populate(zlib)

    execute_process(COMMAND ${CMAKE_COMMAND}
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DCMAKE_GENERATOR=${CMAKE_GENERATOR}
            -DCMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}
            -DZLIB_BUILD_EXAMPLES=OFF
            -S "${zlib_SOURCE_DIR}"
            -B "${zlib_BINARY_DIR}")
    execute_process(COMMAND ${CMAKE_COMMAND}
            --build "${zlib_BINARY_DIR}")

    if (EXISTS ${zlib_SOURCE_DIR}/zconf.h.included)
        # in case the zconf.h file was renamed, rename it back
        file(RENAME ${zlib_SOURCE_DIR}/zconf.h.included ${zlib_SOURCE_DIR}/zconf.h)
    endif ()
endif ()

set(ZLIB_ROOT "${zlib_BINARY_DIR};${zlib_SOURCE_DIR}" CACHE PATH "zlib root path" FORCE)
set(ZLIB_USE_STATIC_LIBS ON CACHE BOOL "zlib use static libs" FORCE)

find_package(ZLIB REQUIRED)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${zlib_SOURCE_DIR}/LICENSE"
            DESTINATION doc/zlib)
endif ()
