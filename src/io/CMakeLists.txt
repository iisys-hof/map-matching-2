set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS serialization)

list(APPEND CMAKE_MODULE_PATH "${OSMIUM_INCLUDE_DIR}/cmake")
find_package(Osmium REQUIRED COMPONENTS io)

add_library(io INTERFACE)

target_precompile_headers(io
        INTERFACE
        <absl/container/flat_hash_map.h>
        <absl/container/flat_hash_set.h>
        <date/date.h>
        <csv.hpp>
        <gpx/Parser.h>
        <gpx/ReportCerr.h>
        <osmium/osm.hpp>
        <osmium/handler.hpp>
        <osmium/builder/osm_object_builder.hpp>
        <osmium/io/any_output.hpp>
        <osmium/io/any_input.hpp>
        <osmium/visitor.hpp>
        <osmium/tags/tags_filter.hpp>
        <osmium/relations/relations_manager.hpp>
        <boost/serialization/serialization.hpp>
        <boost/uuid/uuid_io.hpp>
        <boost/interprocess/managed_mapped_file.hpp>
        <boost/interprocess/allocators/private_node_allocator.hpp>
        <boost/interprocess/containers/vector.hpp>
        <boost/interprocess/containers/list.hpp>
        <boost/algorithm/string.hpp>
        <boost/tokenizer.hpp>
        <boost/archive/binary_iarchive.hpp>
        <boost/archive/binary_oarchive.hpp>
        <boost/geometry/geometry.hpp>
        <boost/geometry/geometries/geometries.hpp>)

target_compile_definitions(io
        INTERFACE
        BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL)

target_include_directories(io
        INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_include_directories(io SYSTEM
        INTERFACE
        ${GPX_INCLUDE_DIRS}
        ${CSV_PARSER_INCLUDE_DIRS}
        ${OSMIUM_INCLUDE_DIR}/include)

target_link_libraries(io
        INTERFACE
        geometry
        Threads::Threads
        absl::flat_hash_set
        absl::flat_hash_map
        Boost::serialization
        date::date
        gpx
        ${OSMIUM_LIBRARIES})
