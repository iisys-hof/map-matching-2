include(GNUInstallDirs)

set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/run" CACHE PATH "install location" FORCE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")

option(INSTALL_ALL_SHARED_LIBS "Install all shared libraries to install directory" OFF)

add_subdirectory(environment)
add_subdirectory(geometry)
add_subdirectory(io)
add_subdirectory(learning)
add_subdirectory(matching)
add_subdirectory(util)

find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(map_matching_2)

target_sources(map_matching_2
        PRIVATE
        main.cpp)

target_compile_definitions(map_matching_2
        PRIVATE
        BOOST_ALLOW_DEPRECATED_HEADERS)

target_link_libraries(map_matching_2
        PRIVATE
        util
        environment
        geometry
        io
        learning
        matching
        rpmalloc
        absl::flat_hash_set
        absl::flat_hash_map
        Boost::boost
        Boost::program_options)

install(TARGETS map_matching_2
        CONFIGURATIONS Release)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    install(CODE "set(INSTALL_ALL_SHARED_LIBS \"${INSTALL_ALL_SHARED_LIBS}\")")
    install(CODE "set(CMAKE_INSTALL_RPATH \"${CMAKE_INSTALL_RPATH}\")")

    install(CODE [[
    if (INSTALL_ALL_SHARED_LIBS)
        message(STATUS "Install all libraries")
    else ()
        message(STATUS "Install only necessary libraries")
        set(POST_INCLUDE_REGEXES "^${CMAKE_BINARY_DIR}")
        set(POST_EXCLUDE_REGEXES ".*")
    endif ()

    file(GET_RUNTIME_DEPENDENCIES
            RESOLVED_DEPENDENCIES_VAR missing
            EXECUTABLES "$<TARGET_FILE:map_matching_2>"
            POST_INCLUDE_REGEXES "${POST_INCLUDE_REGEXES}"
            POST_EXCLUDE_REGEXES "${POST_EXCLUDE_REGEXES}")

    file(INSTALL ${missing}
            DESTINATION "${CMAKE_INSTALL_RPATH}"
            TYPE SHARED_LIBRARY
            FOLLOW_SYMLINK_CHAIN)
    ]])
endif ()
