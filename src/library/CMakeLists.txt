set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread chrono)
find_package(Osmium REQUIRED COMPONENTS io)

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/src/util/version.cpp.in
        ${CMAKE_CURRENT_BINARY_DIR}/src/util/version.cpp
        @ONLY)

add_library(library STATIC)

target_sources(library
        PRIVATE
        src/compare/comparator/comparator_forwarder_compares.cpp
        src/compare/comparator/comparator_forwarder_matches.cpp
        src/compare/comparator/comparator_forwarder_data.cpp
        src/compare/comparator/compare_task.cpp
        src/compare/comparator.cpp
        src/geometry/algorithm/reproject.cpp
        src/geometry/cache.cpp
        src/geometry/srs.cpp
        src/io/memory_mapped/utils.cpp
        src/io/track/track_exporter.cpp
        src/io/csv_importer.cpp
        src/io/file_exporter.cpp
        src/io/importer.cpp
        src/matching/matcher/match_task.cpp
        src/matching/defect.cpp
        src/util/checks.cpp
        src/util/chrono.cpp
        src/util/compiler.cpp
        src/util/time_helper.cpp
        src/util/version.cpp)

if (EXPLICIT_TEMPLATES)
    target_sources(library
            PRIVATE
            src/types/environment/environments/hmm.cpp
            src/types/environment/environments/single.cpp
            src/types/geometry/algorithm/compare.cpp
            src/types/geometry/index/compare_points.cpp
            src/types/geometry/index/compare_segments.cpp
            src/types/geometry/index/index.cpp
            src/types/geometry/index/matcher.cpp
            src/types/geometry/index/median_merge.cpp
            src/types/geometry/index/network_points.cpp
            src/types/geometry/index/network_segments.cpp
            src/types/geometry/network/route/eager_route.cpp
            src/types/geometry/network/route/lazy_route.cpp
            src/types/geometry/network/route/route.cpp
            src/types/geometry/network/route/route_data.cpp
            src/types/geometry/network/node.cpp
            src/types/geometry/network/network.cpp
            src/types/geometry/network/network_base.cpp
            src/types/geometry/network/network_import.cpp
            src/types/geometry/network/edge.cpp
            src/types/geometry/rich_type/eager_multi_rich_line.cpp
            src/types/geometry/rich_type/eager_rich_line.cpp
            src/types/geometry/rich_type/eager_rich_segment.cpp
            src/types/geometry/rich_type/lazy_multi_rich_line.cpp
            src/types/geometry/rich_type/lazy_rich_line.cpp
            src/types/geometry/rich_type/lazy_rich_segment.cpp
            src/types/geometry/rich_type/multi_rich_line.cpp
            src/types/geometry/rich_type/multi_rich_line_data.cpp
            src/types/geometry/rich_type/rich_line.cpp
            src/types/geometry/rich_type/rich_line_data.cpp
            src/types/geometry/rich_type/rich_line_variant.cpp
            src/types/geometry/rich_type/rich_segment.cpp
            src/types/geometry/rich_type/rich_segment_data.cpp
            src/types/geometry/track/multi_track.cpp
            src/types/geometry/track/track.cpp
            src/types/geometry/models.cpp
            src/types/geometry/point.cpp
            src/types/geometry/reprojector.cpp
            src/types/graph/adjacency_list.cpp
            src/types/graph/compressed_sparse_row.cpp
            src/types/io/helper/graph_helper.cpp
            src/types/io/helper/index_build_helper.cpp
            src/types/io/helper/index_helper.cpp
            src/types/io/helper/osm_handler_helper.cpp
            src/types/io/helper/tag_helper.cpp
            src/types/io/memory_mapped/storage/base_storage.cpp
            src/types/io/memory_mapped/storage/graph_storage.cpp
            src/types/io/memory_mapped/storage/index_build_storage.cpp
            src/types/io/memory_mapped/storage/index_storage.cpp
            src/types/io/memory_mapped/storage/osm_handler_storage.cpp
            src/types/io/memory_mapped/storage/tag_storage.cpp
            src/types/io/memory_mapped/types.cpp
            src/types/io/network/arc_node_importer.cpp
            src/types/io/network/giscup_importer.cpp
            src/types/io/network/melbourne_importer.cpp
            src/types/io/network/osm_exporter.cpp
            src/types/io/network/osm_handler.cpp
            src/types/io/network/osm_importer.cpp
            src/types/io/network/seattle_importer.cpp
            src/types/io/track/csv_track_importer.cpp
            src/types/io/track/edges_list_importer.cpp
            src/types/io/track/gpx_track_importer.cpp
            src/types/io/track/input_importer.cpp
            src/types/learning/algorithms/policy_iteration.cpp
            src/types/learning/algorithms/q_learning.cpp
            src/types/learning/algorithms/value_iteration.cpp
            src/types/learning/algorithms/viterbi.cpp
            src/types/matching/algorithm/algorithms.cpp
            src/types/matching/algorithm/candidate_search.cpp
            src/types/matching/algorithm/detect.cpp
            src/types/matching/algorithm/route.cpp
            src/types/matching/candidate/candidate.cpp
            src/types/matching/candidate/candidate_edge.cpp
            src/types/matching/candidate/candidate_node.cpp
            src/types/matching/matcher/matcher_forwarder.cpp
            src/types/matching/matcher.cpp)
endif ()

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND (
        CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 12 OR (
        CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 12 AND
        CMAKE_CXX_COMPILER_VERSION VERSION_LESS 15)))
    # boost/geometry/srs/epsg.hpp dependency severely impacts optimization time
    # see https://github.com/boostorg/geometry/issues/1006
    set_source_files_properties(
            src/geometry/srs.cpp
            PROPERTIES
            COMPILE_FLAGS -O0)
endif ()

target_compile_definitions(library
        PUBLIC
        _USE_MATH_DEFINES
        BOOST_THREAD_VERSION=5
        BOOST_ALLOW_DEPRECATED_HEADERS
        BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL)

if (MSVC)
    target_compile_definitions(library
            PUBLIC
            NOMINMAX
            XML_STATIC)
endif ()

target_include_directories(library
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}/include)

target_include_directories(library SYSTEM
        PUBLIC
        ${GPX_INCLUDE_DIR}
        ${CSV_INCLUDE_DIR}
        ${OSMIUM_INCLUDE_DIRS})

target_link_libraries(library
        PUBLIC
        Threads::Threads
        Boost::boost
        Boost::chrono
        Boost::thread
        Boost::serialization
        date::date
        gpx
        ${OSMIUM_IO_LIBRARIES})

if (UNIX)
    target_link_libraries(library
            PUBLIC
            rt)
endif ()
