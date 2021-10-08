set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

add_library(util STATIC)

target_sources(util
        PRIVATE
        benchmark.cpp)

target_compile_definitions(util
        PUBLIC
        BOOST_ALLOW_DEPRECATED_HEADERS)

target_include_directories(util
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(matching
        PUBLIC
        Threads::Threads
        Boost::boost
        Boost::thread)
