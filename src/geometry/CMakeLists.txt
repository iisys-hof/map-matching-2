set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS graph serialization thread)

list(APPEND CMAKE_MODULE_PATH "${OSMIUM_INCLUDE_DIR}/cmake")
find_package(Osmium REQUIRED COMPONENTS io)

add_library(geometry INTERFACE)

target_precompile_headers(geometry
        INTERFACE
        <cstddef>
        <cstdint>
        <cmath>
        <limits>
        <stdexcept>
        <type_traits>
        <utility>
        <algorithm>
        <string>
        <sstream>
        <tuple>
        <vector>
        <list>
        <iterator>
        <atomic>
        <mutex>
        <absl/container/flat_hash_set.h>
        <absl/container/flat_hash_map.h>
        <absl/container/btree_set.h>
        <boost/serialization/serialization.hpp>
        <boost/serialization/vector.hpp>
        <boost/serialization/unordered_map.hpp>
        <boost/serialization/utility.hpp>
        <boost/serialization/nvp.hpp>
        <boost/interprocess/managed_mapped_file.hpp>
        <boost/interprocess/allocators/private_node_allocator.hpp>
        <boost/interprocess/containers/vector.hpp>
        <boost/interprocess/containers/list.hpp>
        <boost/thread.hpp>
        <boost/bimap.hpp>
        <boost/bimap/unordered_set_of.hpp>
        <boost/geometry/geometry.hpp>
        <boost/geometry/geometries/geometries.hpp>
        <boost/geometry/index/rtree.hpp>
        <boost/geometry/srs/epsg.hpp>
        <boost/geometry/srs/projection.hpp>
        <boost/geometry/srs/transformation.hpp>
        <boost/graph/adjacency_list.hpp>
        <boost/graph/adj_list_serialize.hpp>
        <boost/graph/properties.hpp>
        <boost/graph/copy.hpp>
        <boost/graph/dijkstra_shortest_paths.hpp>
        <boost/property_map/transform_value_property_map.hpp>
        <osmium/osm/types.hpp>)

target_compile_definitions(geometry
        INTERFACE
        _USE_MATH_DEFINES
        BOOST_ALLOW_DEPRECATED_HEADERS
        BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL
        BOOST_THREAD_VERSION=5)

target_include_directories(geometry
        INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_include_directories(geometry SYSTEM
        INTERFACE
        ${GEOMETRY_DEVELOP_INCLUDE_DIRS}
        ${PROTOZERO_INCLUDE_DIR}
        ${OSMIUM_INCLUDE_DIR}/include)

target_link_libraries(geometry
        INTERFACE
        io
        Threads::Threads
        absl::flat_hash_set
        absl::flat_hash_map
        absl::btree
        Boost::boost
        Boost::graph
        Boost::serialization
        Boost::thread
        ${OSMIUM_LIBRARIES})
