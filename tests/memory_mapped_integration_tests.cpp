// Copyright (C) 2022 Adrian Wöltche
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see https://www.gnu.org/licenses/.

#define BOOST_TEST_DYN_LINK

#include <memory>
#include <random>
#include <vector>

#include <boost/test/unit_test.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/allocators/private_adaptive_pool.hpp>
#include <boost/interprocess/containers/vector.hpp>

#include <io/memory_mapped/memory_mapped_file.hpp>

const char graph_vertices_file[] = "graph_vertices.bin";
const char graph_edges_file[] = "graph_edges.bin";
const char points_index_file[] = "points_index.bin";

std::size_t graph_vertices_size = 10 * 1024;
std::size_t graph_edges_size = 10 * 1024;
std::size_t points_index_size = 100 * 1024;

const char graph_vertices_name[] = "nodes";
const char graph_edges_name[] = "edges";
const char points_index_name[] = "points_rtree";

std::size_t area_size = 1000;
std::size_t vertices_size = 1000;
std::size_t edges_size = 1000;

std::random_device random_device;
std::default_random_engine random_engine{random_device()};
std::uniform_int_distribution<std::size_t> random_index{0, vertices_size - 1};
std::uniform_real_distribution<double> random_double{-((double) area_size), (double) area_size};

using point_type = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
using segment_type = boost::geometry::model::segment<point_type>;

using point_allocator_type = boost::interprocess::private_adaptive_pool<point_type,
        boost::interprocess::managed_mapped_file::segment_manager>;
using segment_allocator_type = boost::interprocess::private_adaptive_pool<segment_type,
        boost::interprocess::managed_mapped_file::segment_manager>;

using graph_vertices_type = boost::interprocess::vector<point_type, point_allocator_type>;
using graph_edges_type = boost::interprocess::vector<segment_type, segment_allocator_type>;

using graph_type = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
        boost::property<boost::vertex_index_t, std::size_t>, boost::property<boost::edge_index_t, std::size_t>>;

using vertex_index_map_type = typename boost::property_map<graph_type, boost::vertex_index_t>::type;
using vertex_map_type = boost::iterator_property_map<typename graph_vertices_type::iterator, vertex_index_map_type>;

using edge_index_map_type = typename boost::property_map<graph_type, boost::edge_index_t>::type;
using edge_map_type = boost::iterator_property_map<typename graph_edges_type::iterator, edge_index_map_type>;

using points_index_value_type = std::pair<point_type, std::size_t>;
using points_index_params_type = boost::geometry::index::rstar<16>;
using points_index_indexable_type = boost::geometry::index::indexable<points_index_value_type>;
using points_index_equal_to_type = boost::geometry::index::equal_to<points_index_value_type>;
using points_index_allocator_type = boost::interprocess::allocator<points_index_value_type,
        boost::interprocess::managed_mapped_file::segment_manager>;

using points_index_type = boost::geometry::index::rtree<
        points_index_value_type, points_index_params_type, points_index_indexable_type,
        points_index_equal_to_type, points_index_allocator_type>;

BOOST_AUTO_TEST_SUITE(memory_mapped_integration_tests)

    BOOST_AUTO_TEST_CASE(memory_mapped_integration_graph_test) {
        std::vector<point_type> points;
        points.reserve(vertices_size);
        for (std::size_t i = 0; i < points.capacity(); ++i) {
            points.emplace_back(point_type{random_double(random_engine), random_double(random_engine)});
        }

        std::vector<std::pair<std::size_t, std::size_t>> edge_indices;
        edge_indices.reserve(edges_size);
        for (std::size_t i = 0; i < edge_indices.capacity(); ++i) {
            edge_indices.emplace_back(std::pair{random_index(random_engine), random_index(random_engine)});
        }

        map_matching_2::io::memory_mapped::mmap_start_remover mmap_vertices_remover{graph_vertices_file};
        map_matching_2::io::memory_mapped::mmap_start_remover mmap_edges_remover{graph_edges_file};

        {
            auto *graph_vertices_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_or_create, graph_vertices_file, graph_vertices_size};
            auto *graph_vertices_allocator = new point_allocator_type{graph_vertices_mmap_file->get_segment_manager()};
            auto *graph_vertices = graph_vertices_mmap_file->construct<graph_vertices_type>(graph_vertices_name)(
                    *graph_vertices_allocator);

            auto *graph_edges_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_or_create, graph_edges_file, graph_edges_size};
            auto *graph_edges_allocator = new segment_allocator_type{graph_edges_mmap_file->get_segment_manager()};
            auto *graph_edges = graph_edges_mmap_file->construct<graph_edges_type>(graph_edges_name)(
                    *graph_edges_allocator);

            graph_type graph_import;

            BOOST_TEST_MESSAGE("vertices file size: " << graph_vertices_mmap_file->get_size());
            BOOST_TEST_MESSAGE("vertices file free size: " << graph_vertices_mmap_file->get_free_memory());
            BOOST_TEST_MESSAGE("edges file size: " << graph_edges_mmap_file->get_size());
            BOOST_TEST_MESSAGE("edges file free size: " << graph_edges_mmap_file->get_free_memory());

            vertex_index_map_type vertex_index_map = boost::get(boost::vertex_index, graph_import);
            edge_index_map_type edge_index_map = boost::get(boost::edge_index, graph_import);

            std::size_t i = 0;
            while (i < vertices_size) {
                map_matching_2::io::memory_mapped::emplace_back_auto_grow(
                        graph_vertices_mmap_file, graph_vertices_allocator, graph_vertices, graph_vertices_file,
                        graph_vertices_size, graph_vertices_name, points[i]);

                const auto &vertex_descriptor = boost::add_vertex(i, graph_import);
                i++;
            }

            i = 0;
            while (i < edges_size) {
                const auto &_edge_index = edge_indices[i];
                map_matching_2::io::memory_mapped::emplace_back_auto_grow(
                        graph_edges_mmap_file, graph_edges_allocator, graph_edges, graph_edges_file,
                        graph_edges_size, graph_edges_name,
                        segment_type{points[_edge_index.first], points[_edge_index.second]});

                const auto &edge_descriptor = boost::add_edge(
                        boost::vertex(_edge_index.first, graph_import),
                        boost::vertex(_edge_index.second, graph_import), i, graph_import).first;
                i++;
            }

            BOOST_TEST_MESSAGE("vertices file size: " << graph_vertices_mmap_file->get_size());
            BOOST_TEST_MESSAGE("vertices file free size: " << graph_vertices_mmap_file->get_free_memory());
            BOOST_TEST_MESSAGE("edges file size: " << graph_edges_mmap_file->get_size());
            BOOST_TEST_MESSAGE("edges file free size: " << graph_edges_mmap_file->get_free_memory());

            delete graph_vertices_allocator;
            delete graph_vertices_mmap_file;
            delete graph_edges_allocator;
            delete graph_edges_mmap_file;
        }

        boost::interprocess::managed_mapped_file::shrink_to_fit(graph_vertices_file);
        boost::interprocess::managed_mapped_file::shrink_to_fit(graph_edges_file);

        {
            auto *graph_vertices_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_only, graph_vertices_file};
            auto *graph_vertices = graph_vertices_mmap_file->find<graph_vertices_type>(graph_vertices_name).first;

            auto *graph_edges_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_only, graph_edges_file};
            auto *graph_edges = graph_edges_mmap_file->find<graph_edges_type>(graph_edges_name).first;

            BOOST_TEST_MESSAGE("vertices file size: " << graph_vertices_mmap_file->get_size());
            BOOST_TEST_MESSAGE("vertices file free size: " << graph_vertices_mmap_file->get_free_memory());
            BOOST_TEST_MESSAGE("edges file size: " << graph_edges_mmap_file->get_size());
            BOOST_TEST_MESSAGE("edges file free size: " << graph_edges_mmap_file->get_free_memory());

            graph_type graph_import;

            vertex_index_map_type vertex_index_map = boost::get(boost::vertex_index, graph_import);
            vertex_map_type vertex_map = boost::make_iterator_property_map(graph_vertices->begin(), vertex_index_map);

            edge_index_map_type edge_index_map = boost::get(boost::edge_index, graph_import);
            edge_map_type edge_map = boost::make_iterator_property_map(graph_edges->begin(), edge_index_map);

            for (std::size_t i = 0; i < vertices_size; ++i) {
                boost::add_vertex(i, graph_import);
            }

            for (std::size_t i = 0; i < edges_size; ++i) {
                const auto &_edge_index = edge_indices[i];
                boost::add_edge(
                        boost::vertex(_edge_index.first, graph_import),
                        boost::vertex(_edge_index.second, graph_import), i, graph_import).first;
            }

            std::size_t i = 0;
            for (const auto &vertex_descriptor: boost::make_iterator_range(boost::vertices(graph_import))) {
                if (i % (vertices_size / 10) == 0) {
                    point_type &point = boost::get(vertex_map, vertex_descriptor);
                    BOOST_CHECK_EQUAL(boost::geometry::get<0>(point), boost::geometry::get<0>(points[i]));
                    BOOST_CHECK_EQUAL(boost::geometry::get<1>(point), boost::geometry::get<1>(points[i]));
                }
                i++;
            }

            i = 0;
            for (const auto &edge_descriptor: boost::make_iterator_range(boost::edges(graph_import))) {
                if (i % (edges_size / 10) == 0) {
                    const auto &_edge_index = edge_indices[i];
                    segment_type &segment = boost::get(edge_map, edge_descriptor);
                    BOOST_CHECK_EQUAL(boost::geometry::get<0>(segment.first),
                                      boost::geometry::get<0>(points[_edge_index.first]));
                    BOOST_CHECK_EQUAL(boost::geometry::get<1>(segment.first),
                                      boost::geometry::get<1>(points[_edge_index.first]));
                    BOOST_CHECK_EQUAL(boost::geometry::get<0>(segment.second),
                                      boost::geometry::get<0>(points[_edge_index.second]));
                    BOOST_CHECK_EQUAL(boost::geometry::get<1>(segment.second),
                                      boost::geometry::get<1>(points[_edge_index.second]));
                }
                i++;
            }

            delete graph_vertices_mmap_file;
            delete graph_edges_mmap_file;
        }
    }

    BOOST_AUTO_TEST_CASE(memory_mapped_integration_rtree_test) {
        std::vector<points_index_value_type> points;
        points.reserve(vertices_size);
        for (std::size_t i = 0; i < points.capacity(); ++i) {
            points.emplace_back(
                    points_index_value_type{point_type{random_double(random_engine), random_double(random_engine)}, i});
        }

        map_matching_2::io::memory_mapped::mmap_start_remover mmap_points_index_remover{points_index_file};

        {
            auto *points_index_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_or_create, points_index_file, points_index_size};
            auto *points_index_allocator = new points_index_allocator_type{
                    points_index_mmap_file->get_segment_manager()};

            BOOST_TEST_MESSAGE("points index file size: " << points_index_mmap_file->get_size());
            BOOST_TEST_MESSAGE("points index file free size: " << points_index_mmap_file->get_free_memory());

            auto *points_index = points_index_mmap_file->construct<points_index_type>(points_index_name)(
                    points.begin(), points.end(), *points_index_allocator, std::allocator<points_index_value_type>{});

            BOOST_TEST_MESSAGE("points index file size: " << points_index_mmap_file->get_size());
            BOOST_TEST_MESSAGE("points index file free size: " << points_index_mmap_file->get_free_memory());
            BOOST_TEST_MESSAGE("points index size: " << points_index->size());

            delete points_index_allocator;
            delete points_index_mmap_file;
        }

        boost::interprocess::managed_mapped_file::shrink_to_fit(points_index_file);

        {
            auto *points_index_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_only, points_index_file};
            auto *points_index = points_index_mmap_file->find<points_index_type>(points_index_name).first;

            BOOST_TEST_MESSAGE("points index file size: " << points_index_mmap_file->get_size());
            BOOST_TEST_MESSAGE("points index file free size: " << points_index_mmap_file->get_free_memory());
            BOOST_TEST_MESSAGE("points index size: " << points_index->size());

            delete points_index_mmap_file;
        }

    }

BOOST_AUTO_TEST_SUITE_END()
