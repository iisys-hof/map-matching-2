set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

add_library(util INTERFACE)

target_precompile_headers(util
        INTERFACE
        <functional>
        <chrono>)

target_compile_definitions(util
        INTERFACE
        BOOST_ALLOW_DEPRECATED_HEADERS)

target_include_directories(util
        INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/..)
