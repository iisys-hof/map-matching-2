// Copyright (C) 2021-2021 Adrian Wöltche
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
#define BOOST_TEST_MODULE map_matching_2_tests

#include <cmath>

#include <boost/serialization/serialization.hpp>

#include <boost/geometry.hpp>
#include <boost/test/unit_test.hpp>

#include <geometry/network/types.hpp>
#include <geometry/util.hpp>

#include "helper/network_fixture.hpp"

BOOST_AUTO_TEST_SUITE(network_tests)

    BOOST_FIXTURE_TEST_CASE(node_test, network_fixture<network_metric>) {
        const auto node = create_node(1, {2, 3}, {{"key", "value"}});

        BOOST_CHECK_EQUAL(node.index, std::numeric_limits<std::size_t>::max());
        BOOST_CHECK_EQUAL(node.id, 1);
        BOOST_CHECK_CLOSE_FRACTION(node.point.get<0>(), 2, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(node.point.get<1>(), 3, 1e-6);
        BOOST_CHECK_EQUAL(node.tags.size(), 1);
        BOOST_CHECK_EQUAL(tag_helper.tag(node.tags.at(0)).first, "key");
        BOOST_CHECK_EQUAL(tag_helper.tag(node.tags.at(0)).second, "value");

        const auto node_copy = node;
        BOOST_CHECK_EQUAL(node_copy.id, 1);

        const auto node_move = std::move(node_copy);
        BOOST_CHECK_EQUAL(node_move.id, 1);
    }

    BOOST_FIXTURE_TEST_CASE(edge_test, network_fixture<network_metric>) {
        const auto line = create_line({0, 0, 0, 1, 0, 2, 0, 3, 0, 4});

        BOOST_CHECK_EQUAL(line.size(), 5);
        BOOST_CHECK_CLOSE_FRACTION(line.at(2).get<0>(), 0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(line.at(2).get<1>(), 2, 1e-6);

        const auto length = map_matching_2::geometry::length(line);
        const auto azimuth = map_matching_2::geometry::azimuth_line_deg(line);
        const auto directions = map_matching_2::geometry::directions_deg(line);

        BOOST_CHECK_CLOSE_FRACTION(length, 4.0, 1e-6);
        BOOST_CHECK_LE(std::fabs(azimuth), 1e-6);
        BOOST_CHECK_LE(std::fabs(directions), 1e-6);

        const auto edge = create_edge(1, {1, 2, 3, 4, 5}, line, {{"tag",  "test"},
                                                                 {"tag2", "test2"}});
        BOOST_CHECK_EQUAL(edge.index, std::numeric_limits<std::size_t>::max());
        BOOST_CHECK_EQUAL(edge.id, 1);
        BOOST_CHECK_EQUAL(edge.nodes.size(), 5);
        BOOST_CHECK_EQUAL(edge.line.size(), line.size());
        BOOST_CHECK_CLOSE_FRACTION(edge.length, length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(edge.azimuth, azimuth, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(edge.directions, directions, 1e-6);
        BOOST_CHECK_EQUAL(edge.tags.size(), 2);
        BOOST_CHECK_EQUAL(tag_helper.tag(edge.tags.at(0)).first, "tag");
        BOOST_CHECK_EQUAL(tag_helper.tag(edge.tags.at(0)).second, "test");
        BOOST_CHECK_EQUAL(tag_helper.tag(edge.tags.at(1)).first, "tag2");
        BOOST_CHECK_EQUAL(tag_helper.tag(edge.tags.at(1)).second, "test2");
    }

    BOOST_FIXTURE_TEST_CASE(network_creation_test, network_fixture<network_metric>) {
        network_metric network;

        const auto vertex_a = add_vertex(network, 1, {0, 0});
        const auto vertex_b = add_vertex(network, 2, {0, 1});
        const auto vertex_c = add_vertex(network, 3, {1, 1});
        const auto vertex_d = add_vertex(network, 4, {1, 2});
        const auto vertex_e = add_vertex(network, 5, {1, 3});

        BOOST_CHECK_EQUAL(boost::num_vertices(network.graph), 5);
        BOOST_CHECK_EQUAL(network.graph[vertex_a].id, 1);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[vertex_b].point.get<0>(), 0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[vertex_c].point.get<1>(), 1, 1e-6);
        BOOST_CHECK_EQUAL(network.graph[vertex_d].id, 4);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[vertex_e].point.get<0>(), 1, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[vertex_e].point.get<1>(), 3, 1e-6);

        const auto edge_a = add_edge(network, 1, {1, 2});
        const auto edge_b = add_edge(network, 2, {2, 3});
        const auto edge_c = add_edge(network, 3, {3, 4});
        const auto edge_d = add_edge(network, 4, {4, 5}, {2, 2, 2, 3});

        BOOST_CHECK_EQUAL(boost::num_edges(network.graph), 4);
        BOOST_CHECK_EQUAL(network.graph[edge_a].id, 1);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_b].line.at(0).get<0>(), 0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_b].line.at(0).get<1>(), 1, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_b].length, 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_b].azimuth, 90, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_c].line.at(1).get<0>(), 1, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_c].line.at(1).get<1>(), 2, 1e-6);
        BOOST_CHECK_EQUAL(network.graph[edge_d].id, 4);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_d].length, 3.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_d].azimuth, 0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_d].directions, 180, 1e-6);
    }

    BOOST_FIXTURE_TEST_CASE(network_conversion_test, network_fixture<network_metric>) {
        network_metric network;

        add_vertex(network, 1, {0, 0});
        add_vertex(network, 2, {0, 1});
        add_vertex(network, 3, {1, 1});

        add_edge(network, 1, {1, 2});
        add_edge(network, 2, {2, 3});
        add_edge(network, 3, {3, 2});

        BOOST_CHECK_EQUAL(boost::num_vertices(network.graph), 3);
        BOOST_CHECK_EQUAL(boost::num_edges(network.graph), 3);

        network_static_metric network_converted;
        network.convert(network_converted);

        BOOST_CHECK_EQUAL(boost::num_vertices(network_converted.graph), 3);
        BOOST_CHECK_EQUAL(boost::num_edges(network_converted.graph), 3);
    }

    BOOST_FIXTURE_TEST_CASE(network_reprojection_test, network_fixture<network_geographic>) {
        network_geographic network;

        const auto vertex_a = add_vertex(network, 1, {11.8066265, 50.2673889});
        const auto vertex_b = add_vertex(network, 2, {11.9718510, 50.3551141});

        BOOST_CHECK_EQUAL(boost::num_vertices(network.graph), 2);
        BOOST_CHECK_EQUAL(network.graph[vertex_a].id, 1);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[vertex_a].point.get<0>(), 11.8066265, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[vertex_a].point.get<1>(), 50.2673889, 1e-6);
        BOOST_CHECK_EQUAL(network.graph[vertex_b].id, 2);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[vertex_b].point.get<0>(), 11.9718510, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[vertex_b].point.get<1>(), 50.3551141, 1e-6);

        const auto edge_a = add_edge(network, 1, {1, 2}, {11.8066265, 50.3551141});
        const auto edge_b = add_edge(network, 2, {2, 1}, {11.9718510, 50.2673889});
        const auto edge_c = add_edge(network, 3, {1, 2});

        BOOST_CHECK_EQUAL(boost::num_edges(network.graph), 3);
        BOOST_CHECK_EQUAL(network.graph[edge_a].id, 1);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_a].length, 21516.585, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_a].azimuth, 50.274, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_a].directions, -89.936, 1e-2);
        BOOST_CHECK_EQUAL(network.graph[edge_b].id, 2);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_b].length, 21516.585, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_b].azimuth, -129.599, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_b].directions, -90.064, 1e-2);
        BOOST_CHECK_EQUAL(network.graph[edge_c].id, 3);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_c].length, 15288.437, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[edge_c].azimuth, 50.274, 1e-2);
        BOOST_CHECK(not network.graph[edge_c].has_directions);

        const boost::geometry::srs::proj4 srs_wgs84{"+proj=longlat +datum=WGS84 +no_defs"};
        const boost::geometry::srs::proj4 srs_wgs84_pseudo_mercator{
                "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext  +no_defs"};

        network_metric network_reprojected;
        network.reproject(network_reprojected, srs_wgs84, srs_wgs84_pseudo_mercator);

        auto vertex_iterators = boost::vertices(network_reprojected.graph);

        const auto vertex_a_r = *vertex_iterators.first++;
        const auto vertex_b_r = *vertex_iterators.first++;
        const auto vertex_c_r = *vertex_iterators.first++;

        BOOST_CHECK_EQUAL(boost::num_vertices(network_reprojected.graph), 2);
        BOOST_CHECK_EQUAL(network_reprojected.graph[vertex_a_r].id, 1);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.graph[vertex_a_r].point.get<0>(), 1314307.650, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.graph[vertex_a_r].point.get<1>(), 6492712.310, 1e-2);
        BOOST_CHECK_EQUAL(network_reprojected.graph[vertex_b_r].id, 2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.graph[vertex_b_r].point.get<0>(), 1332700.357, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.graph[vertex_b_r].point.get<1>(), 6508004.0, 1e-2);

        auto edge_iterators = boost::edges(network_reprojected.graph);

        const auto edge_a_r = *edge_iterators.first++;
        const auto edge_b_r = *edge_iterators.first++;
        const auto edge_c_r = *edge_iterators.first++;

        BOOST_CHECK_EQUAL(boost::num_edges(network_reprojected.graph), 3);
        BOOST_CHECK_EQUAL(network_reprojected.graph[edge_a_r].id, 1);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.graph[edge_a_r].length, 33684.396, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.graph[edge_a_r].azimuth, 50.260, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.graph[edge_a_r].directions, -90.0, 1e-2);
        BOOST_CHECK_EQUAL(network_reprojected.graph[edge_b_r].id, 2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.graph[edge_b_r].length, 33684.396, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.graph[edge_b_r].azimuth, -129.740, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.graph[edge_b_r].directions, -90.0, 1e-2);
        BOOST_CHECK_EQUAL(network_reprojected.graph[edge_c_r].id, 3);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.graph[edge_c_r].length, 23919.186, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.graph[edge_c_r].azimuth, 50.260, 1e-2);
        BOOST_CHECK(not network_reprojected.graph[edge_c_r].has_directions);
    }

    BOOST_FIXTURE_TEST_CASE(network_simplification_test, network_fixture<network_metric>) {
        network_metric network = generate_simplification_network();

        network.save("network_simplification_test_nodes.csv",
                     "network_simplification_test_edges.csv");

        BOOST_CHECK_EQUAL(boost::num_vertices(network.graph), 18);
        BOOST_CHECK_EQUAL(boost::num_edges(network.graph), 28);
        BOOST_CHECK_CLOSE_FRACTION(network.length(), 23.0 + 5 * std::sqrt(2), 1e-12);

        network.simplify();

        network.save("network_simplification_test_nodes_simplified.csv",
                     "network_simplification_test_edges_simplified.csv");

        BOOST_CHECK_EQUAL(boost::num_vertices(network.graph), 11);
        BOOST_CHECK_EQUAL(boost::num_edges(network.graph), 16);
        BOOST_CHECK_CLOSE_FRACTION(network.length(), 23.0 + 5 * std::sqrt(2), 1e-12);
    }

    BOOST_FIXTURE_TEST_CASE(network_shortest_path_test, network_fixture<network_metric>) {
        network_metric network;

        const auto &vertex_a = add_vertex(network, 1, {0, 0});
        const auto &vertex_b = add_vertex(network, 2, {2, 2});
        const auto &vertex_c = add_vertex(network, 3, {5, 5});
        const auto &vertex_d = add_vertex(network, 4, {7, 7});
        const auto &vertex_e = add_vertex(network, 5, {10, 10});

        add_edge(network, 1, {1, 2});
        add_edge(network, 1, {2, 4}, {8, 1});
        add_edge(network, 1, {4, 5});

        add_edge(network, 2, {1, 5}, {2, 4, 7, 9});

        add_edge(network, 3, {1, 3}, {2, 3});
        add_edge(network, 3, {3, 5}, {7, 8});

        network.save("network_shortest_path_test_nodes.csv",
                     "network_shortest_path_test_edges.csv");

        const auto starts = network.find_node(1);
        const auto goals = network.find_node(5);

        BOOST_CHECK_CLOSE_FRACTION(network.graph[starts.at(0)].point.get<0>(), 0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[starts.at(0)].point.get<1>(), 0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[goals.at(0)].point.get<0>(), 10, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.graph[goals.at(0)].point.get<1>(), 10, 1e-6);

        BOOST_CHECK_EQUAL(network.graph[starts.at(0)].id, network.graph[vertex_a].id);
        BOOST_CHECK_EQUAL(network.graph[goals.at(0)].id, network.graph[vertex_e].id);

        const std::list shortest_list{vertex_a, vertex_c, vertex_e};
        auto shortest_route = network.extract_route(shortest_list);

        std::vector<typename network_metric::vertex_descriptor> vertices;
        vertices.reserve(boost::num_vertices(network.graph));

        auto predecessors = network.make_predecessors();
        auto distances = network.make_distances();
        auto colors = network.make_colors();
        auto heap_index = network.make_heap_index();
        auto queue = network.make_queue();

        auto dijkstra_shortest_paths = network.dijkstra_shortest_paths(
                starts.at(0), {goals.at(0)}, 10.0, 100.0,
                vertices, predecessors, distances, colors, heap_index, queue);

        auto dijkstra_shortest_route = network.extract_route(dijkstra_shortest_paths.at(0));

        BOOST_CHECK_EQUAL_COLLECTIONS(shortest_list.begin(), shortest_list.end(),
                                      dijkstra_shortest_paths.at(0).begin(), dijkstra_shortest_paths.at(0).end());
        BOOST_CHECK_CLOSE_FRACTION(shortest_route.length, dijkstra_shortest_route.length, 1e-12);
    }

    BOOST_FIXTURE_TEST_CASE(network_spatial_index_test, network_fixture<network_metric>) {
        network_metric network = generate_grid_network();

        network.save("network_spatial_index_test_nodes.csv",
                     "network_spatial_index_test_edges.csv");

        BOOST_CHECK_EQUAL(boost::num_vertices(network.graph), 10 * 10);
        BOOST_CHECK_EQUAL(boost::num_edges(network.graph), 4 * 9 * 10);
        BOOST_CHECK_CLOSE_FRACTION(network.length(), 4 * 9 * 10.0, 1e-12);

        network.rebuild_spatial_indices();

        boost::geometry::model::box<typename network_metric::point_type> center_box{{3.5, 3.5},
                                                                                    {5.5, 5.5}};

        auto center_points = network.query_points(boost::geometry::index::intersects(center_box));
        auto center_lines = network.query_segments(boost::geometry::index::intersects(center_box));
        auto center_lines_unique = network.query_segments_unique(boost::geometry::index::intersects(center_box));

        BOOST_CHECK_EQUAL(center_points.size(), 4);
        BOOST_CHECK_EQUAL(center_lines.size(), 24);
        BOOST_CHECK_EQUAL(center_lines_unique.size(), 24);
    }

BOOST_AUTO_TEST_SUITE_END()
