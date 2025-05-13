// Copyright (C) 2021-2025 Adrian Wöltche
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

#include <cmath>

#include <boost/serialization/serialization.hpp>

#include <boost/geometry.hpp>
#include <boost/test/unit_test.hpp>

#include "helper/network_fixture.hpp"

#include "io/helper/graph_helper.hpp"
#include "geometry/network/network.hpp"
#include "geometry/network/node.hpp"

BOOST_AUTO_TEST_SUITE(network_tests)

    BOOST_FIXTURE_TEST_CASE(node_test, network_fixture<network_import_metric>) {
        const auto node = create_node(1, {2, 3}, {{"key", "value"}});
        const auto &tags = tag_helper.node_tags(node.id);

        BOOST_CHECK_EQUAL(node.id, 1);
        BOOST_CHECK_CLOSE_FRACTION(node.point.get<0>(), 2, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(node.point.get<1>(), 3, 1e-6);
        BOOST_CHECK_EQUAL(tags.size(), 1);
        BOOST_CHECK_EQUAL(tag_helper.tag(tags.at(0)).first, "key");
        BOOST_CHECK_EQUAL(tag_helper.tag(tags.at(0)).second, "value");

        auto node_copy = node;
        BOOST_CHECK_EQUAL(node_copy.id, 1);

        auto node_move = std::move(node_copy);
        BOOST_CHECK_EQUAL(node_move.id, 1);
    }

    BOOST_FIXTURE_TEST_CASE(edge_test, network_fixture<network_import_metric>) {
        const auto line = create_line<point_type, line_type>({0, 0, 0, 1, 0, 2, 0, 3, 0, 4});

        BOOST_CHECK_EQUAL(line.size(), 5);
        BOOST_CHECK_CLOSE_FRACTION(line.at(2).get<0>(), 0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(line.at(2).get<1>(), 2, 1e-6);

        const auto length = map_matching_2::geometry::length(line);
        const auto azimuth = map_matching_2::geometry::azimuth_line_deg(line);
        const auto directions = map_matching_2::geometry::directions_deg(line);

        BOOST_CHECK_CLOSE_FRACTION(length, 4.0, 1e-6);
        BOOST_CHECK_LE(std::fabs(azimuth), 1e-6);
        BOOST_CHECK_LE(std::fabs(directions), 1e-6);

        const auto edge = create_edge(1, line, {
                {"tag", "test"},
                {"tag2", "test2"}
        });
        const auto &tags = tag_helper.edge_tags(edge.id);

        BOOST_CHECK_EQUAL(edge.id, 1);
        BOOST_CHECK_EQUAL(edge.rich_line.size(), line.size());
        BOOST_CHECK_CLOSE_FRACTION(edge.rich_line.length(), length, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(edge.rich_line.azimuth(), azimuth, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(edge.rich_line.directions(), directions, 1e-6);
        BOOST_CHECK_EQUAL(tags.size(), 2);
        BOOST_CHECK_EQUAL(tag_helper.tag(tags.at(0)).first, "tag");
        BOOST_CHECK_EQUAL(tag_helper.tag(tags.at(0)).second, "test");
        BOOST_CHECK_EQUAL(tag_helper.tag(tags.at(1)).first, "tag2");
        BOOST_CHECK_EQUAL(tag_helper.tag(tags.at(1)).second, "test2");
    }

    BOOST_FIXTURE_TEST_CASE(network_creation_test, network_fixture<network_import_metric>) {
        network_import_metric network;

        const auto &vertex_a = add_vertex(network, 1, {0, 0});
        const auto &vertex_b = add_vertex(network, 2, {0, 1});
        const auto &vertex_c = add_vertex(network, 3, {1, 1});
        const auto &vertex_d = add_vertex(network, 4, {1, 2});
        const auto &vertex_e = add_vertex(network, 5, {2, 2});
        const auto &vertex_f = add_vertex(network, 6, {2, 3});
        const auto &vertex_g = add_vertex(network, 7, {1, 3});

        const auto &edge_a = add_edge(network, 1, {1, 2});
        const auto &edge_b = add_edge(network, 2, {2, 3});
        const auto &edge_c = add_edge(network, 3, {3, 4});
        const auto &edge_d = add_edge(network, 4, {
                {4, 5},
                {5, 6},
                {6, 7}
        });

        BOOST_CHECK_EQUAL(network.graph().vertices_view().size(), 7);
        BOOST_CHECK_EQUAL(network.vertex(vertex_a).id, 1);
        BOOST_CHECK_CLOSE_FRACTION(network.vertex(vertex_b).point.get<0>(), 0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.vertex(vertex_c).point.get<1>(), 1, 1e-6);
        BOOST_CHECK_EQUAL(network.vertex(vertex_d).id, 4);
        BOOST_CHECK_CLOSE_FRACTION(network.vertex(vertex_e).point.get<0>(), 2, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.vertex(vertex_e).point.get<1>(), 2, 1e-6);

        BOOST_CHECK_EQUAL(network.graph().edges_view().size(), 4);
        BOOST_CHECK_EQUAL(network.edge(edge_a).id, 1);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_b).rich_line.at(0).get<0>(), 0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_b).rich_line.at(0).get<1>(), 1, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_b).rich_line.length(), 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_b).rich_line.azimuth(), 90, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_c).rich_line.at(1).get<0>(), 1, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_c).rich_line.at(1).get<1>(), 2, 1e-6);
        BOOST_CHECK_EQUAL(network.edge(edge_d).id, 4);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_d).rich_line.length(), 3.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_d).rich_line.azimuth(), 0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_d).rich_line.directions(), 180, 1e-6);
    }

    BOOST_FIXTURE_TEST_CASE(network_conversion_test, network_fixture<network_import_metric>) {
        network_import_metric network_import;

        add_vertex(network_import, 1, {0, 0});
        add_vertex(network_import, 2, {0, 1});
        add_vertex(network_import, 3, {1, 1});

        add_edge(network_import, 1, {1, 2});
        add_edge(network_import, 2, {2, 3});
        add_edge(network_import, 3, {3, 2});

        BOOST_CHECK_EQUAL(network_import.graph().vertices_view().size(), 3);
        BOOST_CHECK_EQUAL(network_import.graph().edges_view().size(), 3);

        network_metric network;
        network.graph_helper().from_adjacency_list(network_import.graph_helper(), network_import.tag_helper());

        BOOST_CHECK_EQUAL(network.graph().vertices_view().size(), 3);
        BOOST_CHECK_EQUAL(network.graph().edges_view().size(), 3);
    }

    BOOST_FIXTURE_TEST_CASE(network_reprojection_test, network_fixture<network_import_geographic>) {
        network_import_geographic network;

        const auto vertex_a = add_vertex(network, 1, {11.8066265, 50.2673889});
        const auto vertex_b = add_vertex(network, 2, {11.9718510, 50.3551141});
        const auto vertex_c = add_vertex(network, 3, {11.8066265, 50.3551141});
        const auto vertex_d = add_vertex(network, 4, {11.9718510, 50.2673889});

        BOOST_CHECK_EQUAL(network.graph().vertices_view().size(), 4);
        BOOST_CHECK_EQUAL(network.vertex(vertex_a).id, 1);
        BOOST_CHECK_CLOSE_FRACTION(network.vertex(vertex_a).point.get<0>(), 11.8066265, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.vertex(vertex_a).point.get<1>(), 50.2673889, 1e-6);
        BOOST_CHECK_EQUAL(network.vertex(vertex_b).id, 2);
        BOOST_CHECK_CLOSE_FRACTION(network.vertex(vertex_b).point.get<0>(), 11.9718510, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.vertex(vertex_b).point.get<1>(), 50.3551141, 1e-6);

        const auto edge_a = add_edge(network, 1, {
                {1, 3},
                {3, 2}
        });
        const auto edge_b = add_edge(network, 2, {
                {2, 4},
                {4, 1}
        });
        const auto edge_c = add_edge(network, 3, {1, 2});

        BOOST_CHECK_EQUAL(network.graph().edges_view().size(), 3);
        BOOST_CHECK_EQUAL(network.edge(edge_a).id, 1);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_a).rich_line.length(), 21516.585, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_a).rich_line.azimuth(), 50.274, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_a).rich_line.directions(), -89.936, 1e-2);
        BOOST_CHECK_EQUAL(network.edge(edge_b).id, 2);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_b).rich_line.length(), 21516.585, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_b).rich_line.azimuth(), -129.599, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_b).rich_line.directions(), -90.064, 1e-2);
        BOOST_CHECK_EQUAL(network.edge(edge_c).id, 3);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_c).rich_line.length(), 15288.437, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network.edge(edge_c).rich_line.azimuth(), 50.274, 1e-2);
        BOOST_CHECK(not network.edge(edge_c).rich_line.has_directions());

        auto reprojector = map_matching_2::geometry::create_point_reprojector(
                map_matching_2::geometry::compute_srs_transform(4326, "geographic", 3857, "cartesian"));

        network_import_metric network_reprojected;
        network_fixture<network_import_metric> metric_fixture;

        const auto vertex_a_r = metric_fixture.add_node(
                network_reprojected, 1, {1, point_type{11.8066265, 50.2673889}, reprojector});
        const auto vertex_b_r = metric_fixture.add_node(
                network_reprojected, 2, {2, point_type{11.9718510, 50.3551141}, reprojector});
        const auto vertex_c_r = metric_fixture.add_node(
                network_reprojected, 3, {3, point_type{11.8066265, 50.3551141}, reprojector});
        const auto vertex_d_r = metric_fixture.add_node(
                network_reprojected, 4, {4, point_type{11.9718510, 50.2673889}, reprojector});

        BOOST_CHECK_EQUAL(network_reprojected.graph().vertices_view().size(), 4);
        BOOST_CHECK_EQUAL(network_reprojected.vertex(vertex_a_r).id, 1);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.vertex(vertex_a_r).point.get<0>(), 1314307.650, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.vertex(vertex_a_r).point.get<1>(), 6492712.310, 1e-2);
        BOOST_CHECK_EQUAL(network_reprojected.vertex(vertex_b_r).id, 2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.vertex(vertex_b_r).point.get<0>(), 1332700.357, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.vertex(vertex_b_r).point.get<1>(), 6508004.0, 1e-2);

        const auto edge_a_r = metric_fixture.add_edge(network_reprojected, 1, {
                {1, 3},
                {3, 2}
        });
        const auto edge_b_r = metric_fixture.add_edge(network_reprojected, 2, {
                {2, 4},
                {4, 1}
        });
        const auto edge_c_r = metric_fixture.add_edge(network_reprojected, 3, {1, 2});

        BOOST_CHECK_EQUAL(network_reprojected.graph().edges_view().size(), 3);
        BOOST_CHECK_EQUAL(network_reprojected.edge(edge_a_r).id, 1);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.edge(edge_a_r).rich_line.length(), 33684.396, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.edge(edge_a_r).rich_line.azimuth(), 50.260, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.edge(edge_a_r).rich_line.directions(), -90.0, 1e-2);
        BOOST_CHECK_EQUAL(network_reprojected.edge(edge_b_r).id, 2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.edge(edge_b_r).rich_line.length(), 33684.396, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.edge(edge_b_r).rich_line.azimuth(), -129.740, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.edge(edge_b_r).rich_line.directions(), -90.0, 1e-2);
        BOOST_CHECK_EQUAL(network_reprojected.edge(edge_c_r).id, 3);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.edge(edge_c_r).rich_line.length(), 23919.186, 1e-2);
        BOOST_CHECK_CLOSE_FRACTION(network_reprojected.edge(edge_c_r).rich_line.azimuth(), 50.260, 1e-2);
        BOOST_CHECK(not network_reprojected.edge(edge_c_r).rich_line.has_directions());
    }

    BOOST_FIXTURE_TEST_CASE(network_simplification_test, network_fixture<network_import_metric>) {
        network_import_metric network = generate_simplification_network();

        network.save_csv("network_simplification_test_nodes.csv",
                "network_simplification_test_edges.csv");

        BOOST_CHECK_EQUAL(network.graph().vertices_view().size(), 18);
        BOOST_CHECK_EQUAL(network.graph().edges_view().size(), 28);
        BOOST_CHECK_CLOSE_FRACTION(network.length(), 23.0 + 5 * std::sqrt(2), 1e-12);

        network.simplify();

        network.save_csv("network_simplification_test_nodes_simplified.csv",
                "network_simplification_test_edges_simplified.csv");

        BOOST_CHECK_EQUAL(network.graph().vertices_view().size(), 11);
        BOOST_CHECK_EQUAL(network.graph().edges_view().size(), 16);
        BOOST_CHECK_CLOSE_FRACTION(network.length(), 23.0 + 5 * std::sqrt(2), 1e-12);
    }

    BOOST_FIXTURE_TEST_CASE(network_shortest_path_test, network_fixture<network_import_metric>) {
        network_import_metric network_import;

        add_vertex(network_import, 1, {0, 0});
        add_vertex(network_import, 2, {2, 2});
        add_vertex(network_import, 3, {5, 5});
        add_vertex(network_import, 4, {7, 7});
        add_vertex(network_import, 5, {10, 10});
        add_vertex(network_import, 6, {8, 1});
        add_vertex(network_import, 7, {2, 4});
        add_vertex(network_import, 8, {7, 9});
        add_vertex(network_import, 9, {2, 3});
        add_vertex(network_import, 10, {7, 8});

        add_edge(network_import, 1, {1, 2});
        add_edge(network_import, 1, {
                {2, 6},
                {6, 4}
        });
        add_edge(network_import, 1, {4, 5});

        add_edge(network_import, 2, {
                {1, 7},
                {7, 8},
                {8, 5}
        });

        add_edge(network_import, 3, {
                {1, 9},
                {9, 3}
        });
        add_edge(network_import, 3, {
                {3, 10},
                {10, 5}
        });

        network_import.save_csv("network_shortest_path_test_nodes.csv",
                "network_shortest_path_test_edges.csv");

        network_metric network;
        network.graph_helper().from_adjacency_list(network_import.graph_helper(), network_import.tag_helper());

        const auto &starts = network.find_node(1);
        const auto &goals = network.find_node(5);

        BOOST_CHECK_CLOSE_FRACTION(network.vertex(starts.at(0)).point.get<0>(), 0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.vertex(starts.at(0)).point.get<1>(), 0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.vertex(goals.at(0)).point.get<0>(), 10, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(network.vertex(goals.at(0)).point.get<1>(), 10, 1e-6);

        const auto &vertex_a = network.find_node(1);
        const auto &vertex_c = network.find_node(3);
        const auto &vertex_e = network.find_node(5);

        BOOST_CHECK_EQUAL(network.vertex(starts.at(0)).id, network.vertex(vertex_a.at(0)).id);
        BOOST_CHECK_EQUAL(network.vertex(goals.at(0)).id, network.vertex(vertex_e.at(0)).id);

        const std::list shortest_list{vertex_a.at(0), vertex_c.at(0), vertex_e.at(0)};
        auto shortest_route = network.extract_route(shortest_list);

        std::vector<typename network_metric::vertex_descriptor> vertices;
        vertices.reserve(network.graph().vertices_view().size());

        auto predecessors = map_matching_2::graph::predecessors_type{};
        auto distances = map_matching_2::graph::distances_type<typename network_metric::length_type>{};
        auto colors = map_matching_2::graph::colors_type{};
        auto queue = map_matching_2::graph::priority_queue_type<
            map_matching_2::graph::priority_type<
                typename network_metric::length_type,
                typename network_metric::vertex_descriptor>>{};

        auto dijkstra_shortest_paths = network.dijkstra_shortest_paths(
                starts.at(0), {goals.at(0)}, 100.0, 10.0, predecessors, distances, colors, queue);

        auto dijkstra_shortest_route = network.extract_route(dijkstra_shortest_paths.at(0));

        BOOST_CHECK_EQUAL_COLLECTIONS(shortest_list.begin(), shortest_list.end(),
                dijkstra_shortest_paths.at(0).begin(), dijkstra_shortest_paths.at(0).end());
        BOOST_CHECK_CLOSE_FRACTION(shortest_route.length(), dijkstra_shortest_route.length(), 1e-12);
    }

    BOOST_FIXTURE_TEST_CASE(network_spatial_index_test, network_fixture<network_import_metric>) {
        network_import_metric network_import = generate_grid_network();

        network_import.save_csv("network_spatial_index_test_nodes.csv",
                "network_spatial_index_test_edges.csv");

        BOOST_CHECK_EQUAL(network_import.graph().vertices_view().size(), 10 * 10);
        BOOST_CHECK_EQUAL(network_import.graph().edges_view().size(), 4 * 9 * 10);
        BOOST_CHECK_CLOSE_FRACTION(network_import.length(), 4 * 9 * 10.0, 1e-12);

        network_metric network;
        network.graph_helper().from_adjacency_list(network_import.graph_helper(), network_import.tag_helper());

        network.build_spatial_indices();

        boost::geometry::model::box<typename network_metric::point_type> center_box{
                {3.5, 3.5},
                {5.5, 5.5}
        };

        auto center_points = network.query_points(boost::geometry::index::intersects(center_box));
        auto center_lines = network.query_segments(boost::geometry::index::intersects(center_box));
        auto center_lines_unique = network.query_segments_unique(boost::geometry::index::intersects(center_box));

        BOOST_CHECK_EQUAL(center_points.size(), 4);
        BOOST_CHECK_EQUAL(center_lines.size(), 24);
        BOOST_CHECK_EQUAL(center_lines_unique.size(), 24);
    }

BOOST_AUTO_TEST_SUITE_END()
