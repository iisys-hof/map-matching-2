set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread program_options)

add_library(app STATIC)

target_sources(app
        PRIVATE
        src/app/compare.cpp
        src/app/global.cpp
        src/app/match.cpp
        src/app/match_cli.cpp
        src/app/mode.cpp
        src/app/network.cpp
        src/app/options.cpp
        src/app/prepare.cpp
        src/app/settings.cpp)

target_compile_definitions(app
        PUBLIC
        BOOST_THREAD_VERSION=5
        BOOST_ALLOW_DEPRECATED_HEADERS
        BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL)

target_include_directories(app
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}/include)

target_link_libraries(app
        PUBLIC
        library
        Boost::program_options)
