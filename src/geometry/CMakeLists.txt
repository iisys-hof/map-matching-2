find_package(Boost REQUIRED COMPONENTS graph)

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
        BOOST_ALLOW_DEPRECATED_HEADERS)

target_include_directories(geometry
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_include_directories(geometry SYSTEM
        PUBLIC
        ${SHORTEST_POINTS_NEW_INCLUDE_DIRS}
        ${PROTOZERO_INCLUDE_DIR}
        ${OSMIUM_INCLUDE_DIR}/include)

target_link_libraries(geometry
        PUBLIC
        io
        Boost::boost
        Boost::graph
        ${OSMIUM_LIBRARIES})
