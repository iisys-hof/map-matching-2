set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS serialization)

list(APPEND CMAKE_MODULE_PATH "${OSMIUM_INCLUDE_DIR}/cmake")
find_package(Osmium REQUIRED COMPONENTS io proj)

add_library(io STATIC)

target_sources(io
        PRIVATE
        importer.cpp
        exporter.cpp
        file_exporter.cpp
        csv_importer.cpp
        csv_exporter.cpp
        network/arc_node_importer.cpp
        network/network_loader.cpp
        network/network_saver.cpp
        network/osm_car_importer.cpp
        network/osm_exporter.cpp
        network/osm_handler.cpp
        network/osm_importer.cpp
        network/osm_relations_manager.cpp
        track/csv_track_importer.cpp
        track/edges_list_importer.cpp
        track/gpx_track_importer.cpp
        track/input_importer.cpp)

target_compile_definitions(io
        PRIVATE
        BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL)

target_include_directories(io
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_include_directories(io SYSTEM
        PUBLIC
        ${GPX_INCLUDE_DIRS}
        ${CSV_PARSER_INCLUDE_DIRS}
        ${OSMIUM_INCLUDE_DIR}/include)

target_link_libraries(io
        PUBLIC
        geometry
        Threads::Threads
        absl::flat_hash_set
        absl::flat_hash_map
        Boost::serialization
        date::date
        gpx
        ${OSMIUM_LIBRARIES})
