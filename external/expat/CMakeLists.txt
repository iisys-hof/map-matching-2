# expat
set(expat_SOURCE_DIR "${FETCHCONTENT_BASE_DIR}/expat-src")
if (EXISTS "${expat_SOURCE_DIR}")
    set(expat_EXISTS ON)
    set(FETCHCONTENT_FULLY_DISCONNECTED ON)
endif ()

FetchContent_Declare(expat
        GIT_REPOSITORY https://github.com/libexpat/libexpat.git
        GIT_TAG R_2_6_4
        SOURCE_SUBDIR expat
        EXCLUDE_FROM_ALL)

FetchContent_MakeAvailable(expat)

if (MSVC)
    if (USE_STATIC_MSVC_RUNTIME_LIBRARY)
        set(EXPAT_MSVC_STATIC_CRT "-DEXPAT_MSVC_STATIC_CRT=ON")
    endif ()
endif ()

if (NOT expat_EXISTS)
    execute_process(COMMAND ${CMAKE_COMMAND}
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DCMAKE_GENERATOR=${CMAKE_GENERATOR}
            ${EXPAT_MSVC_STATIC_CRT}
            -DEXPAT_BUILD_EXAMPLES=OFF
            -DEXPAT_BUILD_TESTS=OFF
            -DEXPAT_BUILD_TOOLS=OFF
            -DEXPAT_SHARED_LIBS=OFF
            -DEXPAT_ENABLE_INSTALL=OFF
            -S "${expat_SOURCE_DIR}/expat"
            -B "${expat_BINARY_DIR}"
            COMMAND_ECHO STDOUT)
    execute_process(COMMAND ${CMAKE_COMMAND}
            --build "${expat_BINARY_DIR}")
endif ()

set(EXPAT_ROOT "${expat_BINARY_DIR};${expat_SOURCE_DIR}/expat/lib" CACHE PATH "expat root path" FORCE)

find_package(EXPAT REQUIRED)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${expat_SOURCE_DIR}/COPYING"
            DESTINATION doc/expat)
endif ()

unset(FETCHCONTENT_FULLY_DISCONNECTED)
