set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS graph serialization thread)

list(APPEND CMAKE_MODULE_PATH "${OSMIUM_INCLUDE_DIR}/cmake")
find_package(Osmium REQUIRED COMPONENTS io proj)

add_library(geometry STATIC)

target_sources(geometry
        PRIVATE
        compare.cpp
        multi_rich_line.cpp
        rich_line.cpp
        rich_segment.cpp
        substring.cpp
        util.cpp
        network/edge.cpp
        network/network.cpp
        network/node.cpp
        network/route.cpp
        network/tag_helper.cpp
        track/measurement.cpp
        track/multi_track.cpp
        track/track.cpp)

target_compile_definitions(geometry
        PUBLIC
        _USE_MATH_DEFINES
        BOOST_ALLOW_DEPRECATED_HEADERS
        BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL)

target_include_directories(geometry
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_include_directories(geometry SYSTEM
        PUBLIC
        ${GEOMETRY_DEVELOP_INCLUDE_DIRS}
        ${PROTOZERO_INCLUDE_DIR}
        ${OSMIUM_INCLUDE_DIR}/include)

target_link_libraries(geometry
        PUBLIC
        io
        Threads::Threads
        Boost::boost
        Boost::graph
        Boost::serialization
        Boost::thread
        ${OSMIUM_LIBRARIES})
