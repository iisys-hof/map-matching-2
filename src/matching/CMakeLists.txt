set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

add_library(matching STATIC)

target_sources(matching
        PRIVATE
        candidate.cpp
        candidate_edge.cpp
        candidate_node.cpp
        detector.cpp
        matcher.cpp)

target_compile_definitions(matching
        PUBLIC
        BOOST_ALLOW_DEPRECATED_HEADERS)

target_include_directories(matching
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_include_directories(matching SYSTEM
        PUBLIC
        ${GEOMETRY_DEVELOP_INCLUDE_DIRS})

target_link_libraries(matching
        PUBLIC
        geometry
        learning
        util
        Threads::Threads
        Boost::boost
        Boost::thread)
