option(BUILD_ONLY_LIBRARY "Build only library" OFF)
cmake_dependent_option(INSTALL_ALL_SHARED_LIBS "Install all shared libraries to install directory" ON "NOT MSVC" OFF)

message(STATUS "Build only library: ${BUILD_ONLY_LIBRARY}")
message(STATUS "Install all shared libraries: ${INSTALL_ALL_SHARED_LIBS}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
if (MSVC)
    if (USE_STATIC_MSVC_RUNTIME_LIBRARY)
        set(Boost_USE_STATIC_RUNTIME ON)
    endif ()
endif ()

add_subdirectory(library)

if (NOT BUILD_ONLY_LIBRARY)
    include(GNUInstallDirs)

    if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/run" CACHE PATH "base install dir" FORCE)

        if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            set(COMPILER_SUBDIR "gcc")
        elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
            set(COMPILER_SUBDIR "clang")
        elseif (MSVC)
            set(COMPILER_SUBDIR "msvc")
        else ()
            set(COMPILER_SUBDIR "unknown")
        endif ()

        if (CMAKE_BUILD_TYPE STREQUAL "Release")
            set(CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/${COMPILER_SUBDIR}/release" CACHE PATH "release install dir" FORCE)
        elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
            set(CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/${COMPILER_SUBDIR}/debug" CACHE PATH "debug install dir" FORCE)
        endif ()

        message(STATUS "Set default install prefix: ${CMAKE_INSTALL_PREFIX}")
    else ()
        message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
    endif ()

    add_subdirectory(app)

    add_executable(map_matching_2)

    target_sources(map_matching_2
            PRIVATE
            main.cpp)

    if (ENABLE_RPMALLOC)
        target_link_libraries(map_matching_2
                PRIVATE
                rpmalloc)
    endif ()

    target_link_libraries(map_matching_2
            PRIVATE
            app)

    if (OVERRIDE_RPATH)
        set_target_properties(
                map_matching_2
                PROPERTIES
                INSTALL_RPATH "\$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
        message(STATUS "Rpath set to: \$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
    endif ()

    if (INSTALL_ALL_SHARED_LIBS)
        message(STATUS "Install all libraries")
    else ()
        message(STATUS "Install only necessary libraries")
        set(_POST_INCLUDE_REGEXES "^${CMAKE_BINARY_DIR}")
        set(_POST_EXCLUDE_REGEXES ".*")
    endif ()

    if (MSVC)
        set(_PRE_EXCLUDE_REGEXES "api-ms-" "ext-ms-")
        set(_POST_EXCLUDE_REGEXES ".*system32/.*\\.dll")
    endif ()

    install(TARGETS
            map_matching_2
            RUNTIME_DEPENDENCIES
            PRE_EXCLUDE_REGEXES "${_PRE_EXCLUDE_REGEXES}"
            POST_INCLUDE_REGEXES "${_POST_INCLUDE_REGEXES}"
            POST_EXCLUDE_REGEXES "${_POST_EXCLUDE_REGEXES}")

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        install(FILES
                "${PROJECT_SOURCE_DIR}/external/libstdc++/COPYING3"
                "${PROJECT_SOURCE_DIR}/external/libstdc++/COPYING3.LIB"
                "${PROJECT_SOURCE_DIR}/external/libstdc++/COPYING.RUNTIME"
                DESTINATION doc/libstdc++)
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        install(FILES
                "${PROJECT_SOURCE_DIR}/external/libc++/LICENSE.TXT"
                DESTINATION doc/libc++)
    elseif (MSVC)
        install(FILES
                "${PROJECT_SOURCE_DIR}/external/STL/LICENSE.txt"
                DESTINATION doc/STL)
    endif ()

    install(FILES
            "${PROJECT_SOURCE_DIR}/LICENSE.md"
            DESTINATION doc/map_matching_2)
endif ()
