// Copyright (C) 2023-2024 Adrian Wöltche
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

#include <random>
#include <set>
#include <format>

#include <boost/test/unit_test.hpp>

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_mapped_file.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/list.hpp>
#include <boost/interprocess/offset_ptr.hpp>

#include "graph/adjacency_list.hpp"
#include "graph/compressed_sparse_row.hpp"
#include "io/memory_mapped/utils.hpp"

// workaround for template count compilation issue in Clang
template<typename Type, typename Allocator>
using boost_interprocess_vector = boost::interprocess::vector<Type, Allocator>;

BOOST_AUTO_TEST_SUITE(graph_tests)

    BOOST_AUTO_TEST_CASE(mutable_directional_graph_construction_test) {

        using adjacency_list = map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::directional, void, void, std::list>;

        adjacency_list graph;

        auto vertex_a = graph.add_vertex();
        auto vertex_b = graph.add_vertex();
        auto vertex_c = graph.add_vertex();

        auto edge_ab = graph.add_edge(vertex_a, vertex_b);
        auto edge_bc = graph.add_edge(vertex_b, vertex_c);

        BOOST_CHECK_EQUAL(graph.vertices_view().size(), 3);
        BOOST_CHECK_EQUAL(graph.out_edges_view(vertex_a).size(), 1);
        BOOST_CHECK_EQUAL(graph.out_degree(vertex_a), 1);
        BOOST_CHECK_EQUAL(graph.out_edges_view(vertex_b).size(), 1);
        BOOST_CHECK_EQUAL(graph.out_degree(vertex_b), 1);
        BOOST_CHECK_EQUAL(graph.out_edges_view(vertex_c).size(), 0);
        BOOST_CHECK_EQUAL(graph.out_degree(vertex_c), 0);
        BOOST_CHECK_EQUAL(graph.edges_view().size(), 2);

        BOOST_CHECK_EQUAL(std::addressof(graph.edges_view()[graph.out_edges_view(vertex_a).front()]),
                std::addressof(graph.edges_view()[edge_ab]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.source(edge_ab)]),
                std::addressof(graph.vertices_view()[vertex_a]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.target(edge_ab)]),
                std::addressof(graph.vertices_view()[vertex_b]));
        BOOST_CHECK_EQUAL(std::addressof(graph.edges_view()[graph.out_edges_view(vertex_b).front()]),
                std::addressof(graph.edges_view()[edge_bc]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.source(edge_bc)]),
                std::addressof(graph.vertices_view()[vertex_b]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.target(edge_bc)]),
                std::addressof(graph.vertices_view()[vertex_c]));

    }

    BOOST_AUTO_TEST_CASE(mutable_undirected_graph_construction_test) {

        using adjacency_list = map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::undirected, void, void, std::list>;

        adjacency_list graph;

        auto vertex_a = graph.add_vertex();
        auto vertex_b = graph.add_vertex();
        auto vertex_c = graph.add_vertex();

        auto edge_ab = graph.add_edge(vertex_a, vertex_b);
        auto edge_bc = graph.add_edge(vertex_b, vertex_c);

        BOOST_CHECK_EQUAL(graph.vertices_view().size(), 3);
        BOOST_CHECK_EQUAL(graph.out_edges_view(vertex_a).size(), 1);
        BOOST_CHECK_EQUAL(graph.out_degree(vertex_a), 1);
        BOOST_CHECK_EQUAL(graph.out_edges_view(vertex_b).size(), 2);
        BOOST_CHECK_EQUAL(graph.out_degree(vertex_b), 2);
        BOOST_CHECK_EQUAL(graph.out_edges_view(vertex_c).size(), 1);
        BOOST_CHECK_EQUAL(graph.out_degree(vertex_c), 1);
        BOOST_CHECK_EQUAL(graph.edges_view().size(), 2);

        BOOST_CHECK_EQUAL(std::addressof(graph.edges_view()[graph.out_edges_view(vertex_a).front()]),
                std::addressof(graph.edges_view()[edge_ab]));
        BOOST_CHECK_EQUAL(std::addressof(graph.edges_view()[graph.out_edges_view(vertex_b).front()]),
                std::addressof(graph.edges_view()[edge_ab]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.source(edge_ab)]),
                std::addressof(graph.vertices_view()[vertex_a]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.target(edge_ab)]),
                std::addressof(graph.vertices_view()[vertex_b]));
        BOOST_CHECK_EQUAL(std::addressof(graph.edges_view()[graph.out_edges_view(vertex_b).at(1)]),
                std::addressof(graph.edges_view()[edge_bc]));
        BOOST_CHECK_EQUAL(std::addressof(graph.edges_view()[graph.out_edges_view(vertex_c).front()]),
                std::addressof(graph.edges_view()[edge_bc]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.source(edge_bc)]),
                std::addressof(graph.vertices_view()[vertex_b]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.target(edge_bc)]),
                std::addressof(graph.vertices_view()[vertex_c]));

    }

    BOOST_AUTO_TEST_CASE(mutable_bidirectional_graph_construction_test) {

        using adjacency_list = map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::bidirectional, void, void, std::list>;

        adjacency_list graph;

        auto vertex_a = graph.add_vertex();
        auto vertex_b = graph.add_vertex();
        auto vertex_c = graph.add_vertex();

        auto edge_ab = graph.add_edge(vertex_a, vertex_b);
        auto edge_bc = graph.add_edge(vertex_b, vertex_c);

        BOOST_CHECK_EQUAL(graph.vertices_view().size(), 3);
        BOOST_CHECK_EQUAL(graph.in_edges_view(vertex_a).size(), 0);
        BOOST_CHECK_EQUAL(graph.out_edges_view(vertex_a).size(), 1);
        BOOST_CHECK_EQUAL(graph.in_degree(vertex_a), 0);
        BOOST_CHECK_EQUAL(graph.out_degree(vertex_a), 1);
        BOOST_CHECK_EQUAL(graph.degree(vertex_a), 1);
        BOOST_CHECK_EQUAL(graph.in_edges_view(vertex_b).size(), 1);
        BOOST_CHECK_EQUAL(graph.out_edges_view(vertex_b).size(), 1);
        BOOST_CHECK_EQUAL(graph.in_degree(vertex_b), 1);
        BOOST_CHECK_EQUAL(graph.out_degree(vertex_b), 1);
        BOOST_CHECK_EQUAL(graph.degree(vertex_b), 2);
        BOOST_CHECK_EQUAL(graph.in_edges_view(vertex_c).size(), 1);
        BOOST_CHECK_EQUAL(graph.out_edges_view(vertex_c).size(), 0);
        BOOST_CHECK_EQUAL(graph.in_degree(vertex_c), 1);
        BOOST_CHECK_EQUAL(graph.out_degree(vertex_c), 0);
        BOOST_CHECK_EQUAL(graph.degree(vertex_c), 1);
        BOOST_CHECK_EQUAL(graph.edges_view().size(), 2);

        BOOST_CHECK_EQUAL(std::addressof(graph.edges_view()[graph.out_edges_view(vertex_a).front()]),
                std::addressof(graph.edges_view()[edge_ab]));
        BOOST_CHECK_EQUAL(std::addressof(graph.edges_view()[graph.in_edges_view(vertex_b).front()]),
                std::addressof(graph.edges_view()[edge_ab]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.source(edge_ab)]),
                std::addressof(graph.vertices_view()[vertex_a]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.target(edge_ab)]),
                std::addressof(graph.vertices_view()[vertex_b]));
        BOOST_CHECK_EQUAL(std::addressof(graph.edges_view()[graph.out_edges_view(vertex_b).front()]),
                std::addressof(graph.edges_view()[edge_bc]));
        BOOST_CHECK_EQUAL(std::addressof(graph.edges_view()[graph.in_edges_view(vertex_c).front()]),
                std::addressof(graph.edges_view()[edge_bc]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.source(edge_bc)]),
                std::addressof(graph.vertices_view()[vertex_b]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.target(edge_bc)]),
                std::addressof(graph.vertices_view()[vertex_c]));

    }

    BOOST_AUTO_TEST_CASE(mutable_data_graph_construction_test) {

        struct node {
            std::size_t index;
        };

        struct edge {
            std::size_t index;
        };

        using adjacency_list = map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::directional, node, edge, std::list>;

        adjacency_list graph;

        auto vertex_a = graph.add_vertex(node(1));
        auto vertex_b = graph.add_vertex(node(2));
        auto vertex_c = graph.add_vertex(node(3));

        auto edge_ab = graph.add_edge(vertex_a, vertex_b, edge(4));
        auto edge_bc = graph.add_edge(vertex_b, vertex_c, edge(5));

        BOOST_CHECK_EQUAL(graph.vertices_view().size(), 3);
        BOOST_CHECK_EQUAL(graph.out_edges_view(vertex_a).size(), 1);
        BOOST_CHECK_EQUAL(graph.out_edges_view(vertex_b).size(), 1);
        BOOST_CHECK_EQUAL(graph.edges_view().size(), 2);

        BOOST_CHECK_EQUAL(std::addressof(graph.edges_view()[graph.out_edges_view(vertex_a).front()]),
                std::addressof(graph.edges_view()[edge_ab]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.source(edge_ab)]),
                std::addressof(graph.vertices_view()[vertex_a]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.target(edge_ab)]),
                std::addressof(graph.vertices_view()[vertex_b]));
        BOOST_CHECK_EQUAL(std::addressof(graph.edges_view()[graph.out_edges_view(vertex_b).front()]),
                std::addressof(graph.edges_view()[edge_bc]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.source(edge_bc)]),
                std::addressof(graph.vertices_view()[vertex_b]));
        BOOST_CHECK_EQUAL(std::addressof(graph.vertices_view()[graph.target(edge_bc)]),
                std::addressof(graph.vertices_view()[vertex_c]));

        BOOST_CHECK_EQUAL(graph.vertices_view()[vertex_a].get().index, 1);
        BOOST_CHECK_EQUAL(graph.vertices_view()[vertex_b].get().index, 2);
        BOOST_CHECK_EQUAL(graph.vertices_view()[vertex_c].get().index, 3);
        BOOST_CHECK_EQUAL(graph.edges_view()[edge_ab].get().index, 4);
        BOOST_CHECK_EQUAL(graph.edges_view()[edge_bc].get().index, 5);

    }

    BOOST_AUTO_TEST_CASE(mutable_graph_variations_test) {

        std::size_t vertex_size = 20;
        std::size_t edge_size = 50;

        std::random_device random_device;
        auto seed = random_device();
        BOOST_TEST_MESSAGE("Used seed: " + std::to_string(seed));
        std::default_random_engine random_engine{seed};
        std::uniform_int_distribution<std::size_t> random_vertex{0, vertex_size - 1};
        std::uniform_int_distribution<std::size_t> random_edge{0, edge_size - 1};

        auto test = [&]<typename Graph>() {
            Graph graph;

            using vertex_descriptor = typename Graph::vertex_descriptor;
            using out_edge_descriptor = typename Graph::out_edge_descriptor;
            using in_edge_descriptor = typename Graph::in_edge_descriptor;
            using edge_descriptor = typename Graph::edge_descriptor;

            std::vector<vertex_descriptor> vertices;
            vertices.reserve(vertex_size);
            for (std::size_t i = 0; i < vertex_size; ++i) {
                vertices.emplace_back(graph.add_vertex());
            }

            std::vector<edge_descriptor> edges;
            edges.reserve(edge_size);
            for (std::size_t i = 0; i < edge_size; ++i) {
                edges.emplace_back(graph.add_edge(vertices[random_vertex(random_engine)],
                        vertices[random_vertex(random_engine)]));
            }

            BOOST_CHECK_EQUAL(graph.vertices_view().size(), vertex_size);
            BOOST_CHECK_EQUAL(graph.edges_view().size(), edge_size);

            auto vertices_v = graph.vertices_view();
            for (vertex_descriptor vertex_desc = vertices_v.begin();
                 vertex_desc != vertices_v.end(); vertex_desc = vertices_v.next(vertex_desc)) {
                auto &vertex = vertices_v[vertex_desc];

                auto out_edges_v = graph.out_edges_view(vertex_desc);
                for (out_edge_descriptor out_edge_desc = out_edges_v.begin();
                     out_edge_desc != out_edges_v.end(); out_edge_desc = out_edges_v.next(out_edge_desc)) {
                    edge_descriptor edge_desc = out_edges_v[out_edge_desc];

                    if constexpr (Graph::is_undirected_v) {
                        BOOST_CHECK((std::addressof(vertices_v[graph.source(edge_desc)]) ==
                                    std::addressof(vertex)) ||
                                (std::addressof(vertices_v[graph.target(edge_desc)]) ==
                                    std::addressof(vertex)));
                    }

                    if constexpr (Graph::is_directional_v || Graph::is_bidirectional_v) {
                        BOOST_CHECK_EQUAL(std::addressof(vertices_v[graph.source(edge_desc)]),
                                std::addressof(vertex));
                    }

                }

                if constexpr (Graph::is_bidirectional_v) {
                    auto in_edges_v = graph.in_edges_view(vertex_desc);
                    for (in_edge_descriptor in_edge_desc = in_edges_v.begin();
                         in_edge_desc != in_edges_v.end(); in_edge_desc = in_edges_v.next(in_edge_desc)) {
                        BOOST_CHECK_EQUAL(std::addressof(vertices_v[graph.target(in_edges_v[in_edge_desc])]),
                                std::addressof(vertex));
                    }
                }

            }
        };

        BOOST_TEST_MESSAGE("undirected std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::undirected, void, void, std::list>>();

        BOOST_TEST_MESSAGE("directional std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::directional, void, void, std::list>>();

        BOOST_TEST_MESSAGE("bidirectional std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::bidirectional, void, void, std::list>>();

        BOOST_TEST_MESSAGE("undirected std::vector");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::vector, std::vector, map_matching_2::graph::undirected, void, void, std::vector>>();

        BOOST_TEST_MESSAGE("directional std::vector");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::vector, std::vector, map_matching_2::graph::directional, void, void, std::vector>>();

        BOOST_TEST_MESSAGE("bidirectional std::vector");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::vector, std::vector, map_matching_2::graph::bidirectional, void, void, std::vector>>();

    }

    BOOST_AUTO_TEST_CASE(mutable_graph_copy_move_test) {

        std::size_t vertex_size = 20;
        std::size_t edge_size = 50;

        std::random_device random_device;
        auto seed = random_device();
        BOOST_TEST_MESSAGE("Used seed: " + std::to_string(seed));
        std::default_random_engine random_engine{seed};
        std::uniform_int_distribution<std::size_t> random_vertex{0, vertex_size - 1};
        std::uniform_int_distribution<std::size_t> random_edge{0, edge_size - 1};

        auto verify = [&]<typename Graph>(const Graph &graph) {
            BOOST_CHECK_EQUAL(graph.vertices_view().size(), vertex_size);
            BOOST_CHECK_EQUAL(graph.edges_view().size(), edge_size);

            const auto vertices_v = graph.vertices_view();
            for (auto vertex_desc = vertices_v.begin();
                 vertex_desc != vertices_v.end(); vertex_desc = vertices_v.next(vertex_desc)) {
                const auto &vertex = vertices_v[vertex_desc];

                const auto out_edges_v = graph.out_edges_view(vertex_desc);
                for (auto out_edge_desc = out_edges_v.begin();
                     out_edge_desc != out_edges_v.end(); out_edge_desc = out_edges_v.next(out_edge_desc)) {
                    const auto &edge_desc = out_edges_v[out_edge_desc];

                    if constexpr (Graph::is_undirected_v) {
                        BOOST_CHECK((std::addressof(vertices_v[graph.source(edge_desc)]) ==
                                    std::addressof(vertex)) ||
                                (std::addressof(vertices_v[graph.target(edge_desc)]) ==
                                    std::addressof(vertex)));
                    }

                    if constexpr (Graph::is_directional_v || Graph::is_bidirectional_v) {
                        BOOST_CHECK_EQUAL(std::addressof(vertices_v[graph.source(edge_desc)]),
                                std::addressof(vertex));
                    }

                }

                if constexpr (Graph::is_bidirectional_v) {
                    const auto in_edges_v = graph.in_edges_view(vertex_desc);
                    for (auto in_edge_desc = in_edges_v.begin();
                         in_edge_desc != in_edges_v.end(); in_edge_desc = in_edges_v.next(in_edge_desc)) {
                        BOOST_CHECK_EQUAL(std::addressof(vertices_v[graph.target(in_edges_v[in_edge_desc])]),
                                std::addressof(vertex));
                    }
                }

            }
        };

        auto create = [&]<typename Graph>() {
            Graph graph;

            using vertex_descriptor = typename Graph::vertex_descriptor;
            using edge_descriptor = typename Graph::edge_descriptor;

            std::vector<vertex_descriptor> vertices;
            vertices.reserve(vertex_size);
            for (std::size_t i = 0; i < vertex_size; ++i) {
                vertices.emplace_back(graph.add_vertex());
            }

            std::vector<edge_descriptor> edges;
            edges.reserve(edge_size);
            for (std::size_t i = 0; i < edge_size; ++i) {
                edges.emplace_back(graph.add_edge(vertices[random_vertex(random_engine)],
                        vertices[random_vertex(random_engine)]));
            }

            return graph;
        };

        auto test = [&]<typename Graph>() {
            auto graph1 = create.template operator()<Graph>();
            verify.template operator()<Graph>(graph1);
            auto graph2 = Graph(graph1);
            verify.template operator()<Graph>(graph2);
            auto graph3 = graph2;
            verify.template operator()<Graph>(graph3);
            auto graph4 = Graph(std::move(graph2));
            verify.template operator()<Graph>(graph4);
            auto graph5 = std::move(graph3);
            verify.template operator()<Graph>(graph5);
        };

        BOOST_TEST_MESSAGE("undirected std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::undirected, void, void, std::list>>();

        BOOST_TEST_MESSAGE("directional std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::directional, void, void, std::list>>();

        BOOST_TEST_MESSAGE("bidirectional std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::bidirectional, void, void, std::list>>();

        BOOST_TEST_MESSAGE("undirected std::vector");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::vector, std::vector, map_matching_2::graph::undirected, void, void, std::vector>>();

        BOOST_TEST_MESSAGE("directional std::vector");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::vector, std::vector, map_matching_2::graph::directional, void, void, std::vector>>();

        BOOST_TEST_MESSAGE("bidirectional std::vector");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::vector, std::vector, map_matching_2::graph::bidirectional, void, void, std::vector>>();

    }

    BOOST_AUTO_TEST_CASE(mutable_graph_delete_test) {

        auto create_graph = []<typename Graph>() ->
            std::tuple<Graph,
                std::vector<typename Graph::vertex_descriptor>,
                std::vector<typename Graph::edge_descriptor>> {
            Graph graph;

            using vertex_descriptor = typename Graph::vertex_descriptor;
            using edge_descriptor = typename Graph::edge_descriptor;

            std::vector<vertex_descriptor> vertices;
            vertices.reserve(5);
            for (std::size_t i = 0; i < 5; ++i) {
                vertices.emplace_back(graph.add_vertex());
            }

            /*
             *  0: 0 --> 0
             *  1: 0 --> 0
             *  2: 0 --> 1
             *  3: 1 --> 2
             *  4: 1 --> 2
             *  5: 2 --> 1
             *  6: 1 --> 3
             *  7: 3 --> 1
             *  8: 1 --> 4
             *  9: 4 --> 3
             * 10: 4 --> 2
             */

            std::vector<edge_descriptor> edges;
            edges.reserve(11);
            edges.emplace_back(graph.add_edge(vertices[0], vertices[0]));
            edges.emplace_back(graph.add_edge(vertices[0], vertices[0]));
            edges.emplace_back(graph.add_edge(vertices[0], vertices[1]));
            edges.emplace_back(graph.add_edge(vertices[1], vertices[2]));
            edges.emplace_back(graph.add_edge(vertices[1], vertices[2]));
            edges.emplace_back(graph.add_edge(vertices[2], vertices[1]));
            edges.emplace_back(graph.add_edge(vertices[1], vertices[3]));
            edges.emplace_back(graph.add_edge(vertices[3], vertices[1]));
            edges.emplace_back(graph.add_edge(vertices[1], vertices[4]));
            edges.emplace_back(graph.add_edge(vertices[4], vertices[3]));
            edges.emplace_back(graph.add_edge(vertices[4], vertices[2]));

            return std::tuple{std::move(graph), std::move(vertices), std::move(edges)};
        };

        auto print = []<typename Graph>(const Graph &graph) {
            const auto vertices_v = graph.vertices_view();
            for (auto vertex_desc = vertices_v.begin();
                 vertex_desc != vertices_v.end(); vertex_desc = vertices_v.next(vertex_desc)) {
                const auto &vertex = vertices_v[vertex_desc];

                BOOST_TEST_MESSAGE(std::format("vertex {}", graph.vertex_index(vertex_desc)));

                const auto out_edges_v = graph.out_edges_view(vertex_desc);
                for (auto out_edge_desc = out_edges_v.begin();
                     out_edge_desc != out_edges_v.end(); out_edge_desc = out_edges_v.next(out_edge_desc)) {
                    const auto &edge_desc = out_edges_v[out_edge_desc];

                    if constexpr (Graph::is_directional_v || Graph::is_bidirectional_v) {
                        BOOST_TEST_MESSAGE(std::format("edge {}: {} --> {}",
                            graph.edge_index(edge_desc),
                            graph.vertex_index(graph.source(edge_desc)),
                            graph.vertex_index(graph.target(edge_desc))));
                    }

                    if constexpr (Graph::is_undirected_v) {
                        BOOST_TEST_MESSAGE(std::format("edge {}: {} --> {}{}",
                            graph.edge_index(edge_desc),
                            graph.vertex_index(graph.source(edge_desc)),
                            graph.vertex_index(graph.target(edge_desc)),
                            std::addressof(vertices_v[graph.source(edge_desc)]) ==
                            std::addressof(vertex) ? "" : " [U]"));
                    }

                }

                if constexpr (Graph::is_bidirectional_v) {
                    const auto in_edges_v = graph.in_edges_view(vertex_desc);
                    for (auto in_edge_desc = in_edges_v.begin();
                         in_edge_desc != in_edges_v.end(); in_edge_desc = in_edges_v.next(in_edge_desc)) {
                        const auto &edge_desc = in_edges_v[in_edge_desc];

                        BOOST_TEST_MESSAGE(std::format("edge {}: {} --> {} [B]",
                            graph.edge_index(edge_desc),
                            graph.vertex_index(graph.source(edge_desc)),
                            graph.vertex_index(graph.target(edge_desc))));
                    }
                }

            }
        };

        auto verify = []<typename Graph>(const Graph &graph,
                const std::set<std::size_t> &delete_vertices,
                const std::set<std::size_t> &delete_edges) {
            const auto vertices_v = graph.vertices_view();
            for (auto vertex_desc = vertices_v.begin();
                 vertex_desc != vertices_v.end(); vertex_desc = vertices_v.next(vertex_desc)) {
                const auto &vertex = vertices_v[vertex_desc];

                BOOST_CHECK(not delete_vertices.contains(graph.vertex_index(vertex_desc)));

                const auto out_edges_v = graph.out_edges_view(vertex_desc);
                for (auto out_edge_desc = out_edges_v.begin();
                     out_edge_desc != out_edges_v.end(); out_edge_desc = out_edges_v.next(out_edge_desc)) {
                    const auto &edge_desc = out_edges_v[out_edge_desc];

                    BOOST_CHECK(not delete_edges.contains(graph.edge_index(edge_desc)));

                    if constexpr (Graph::is_undirected_v) {
                        BOOST_CHECK((std::addressof(vertices_v[graph.source(edge_desc)]) ==
                                    std::addressof(vertex)) ||
                                (std::addressof(vertices_v[graph.target(edge_desc)]) ==
                                    std::addressof(vertex)));
                    }

                    if constexpr (Graph::is_directional_v || Graph::is_bidirectional_v) {
                        BOOST_CHECK_EQUAL(std::addressof(vertices_v[graph.source(edge_desc)]),
                                std::addressof(vertex));
                    }

                }

                if constexpr (Graph::is_bidirectional_v) {
                    const auto in_edges_v = graph.in_edges_view(vertex_desc);
                    for (auto in_edge_desc = in_edges_v.begin();
                         in_edge_desc != in_edges_v.end(); in_edge_desc = in_edges_v.next(in_edge_desc)) {
                        const auto &edge_desc = in_edges_v[in_edge_desc];

                        BOOST_CHECK(not delete_edges.contains(graph.edge_index(edge_desc)));

                        BOOST_CHECK_EQUAL(std::addressof(vertices_v[graph.target(edge_desc)]),
                                std::addressof(vertex));
                    }
                }

            }

            const auto edges_v = graph.edges_view();
            for (auto edge_desc = edges_v.begin(); edge_desc != edges_v.end(); edge_desc = edges_v.next(edge_desc)) {
                BOOST_CHECK(not delete_edges.contains(graph.edge_index(edge_desc)));
            }
        };

        auto test = [&]<typename Graph>() {
            using vertex_descriptor = typename Graph::vertex_descriptor;
            using edge_descriptor = typename Graph::edge_descriptor;

            for (std::size_t i = 0; i < 6; ++i) {
                auto [graph, vertices, edges] = create_graph.template operator()<Graph>();

                BOOST_TEST_MESSAGE("graph before deletion:");
                print(graph);

                std::set<std::size_t> delete_vertices;
                std::set<std::size_t> clear_vertices;
                std::set<std::size_t> delete_edges;
                std::set<std::pair<std::size_t, std::size_t>> delete_edge_pairs;

                switch (i) {
                    case 0:
                        BOOST_TEST_MESSAGE("delete edges: 0");
                        delete_edges = {0};
                        break;
                    case 1:
                        BOOST_TEST_MESSAGE("delete edges: 0, 1");
                        delete_edges = {0, 1};
                        break;
                    case 2:
                        BOOST_TEST_MESSAGE("delete edges: 0, 1, 8, 9, 10");
                        delete_edges = {0, 1, 8, 9, 10};
                        BOOST_TEST_MESSAGE("delete vertices: 4");
                        delete_vertices = {4};
                        break;
                    case 3:
                        BOOST_TEST_MESSAGE("delete edges: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10");
                        delete_edges = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
                        BOOST_TEST_MESSAGE("delete vertices: 0, 1, 2, 3, 4");
                        delete_vertices = {0, 1, 2, 3, 4};
                        break;
                    case 4:
                        BOOST_TEST_MESSAGE("delete edge pairs: 0 --> 0, 0 --> 1, 1 --> 2, 1 --> 4");
                        delete_edge_pairs = {
                                {0, 0},
                                {0, 1},
                                {1, 2},
                                {1, 4}
                        };
                        BOOST_TEST_MESSAGE("delete vertices: 0");
                        delete_vertices = {0};
                        break;
                    case 5:
                        BOOST_TEST_MESSAGE("clear vertices: 1");
                        clear_vertices = {1};
                        break;
                }

                for (const std::size_t delete_edge : delete_edges) {
                    graph.remove_edge(edges.at(delete_edge));
                }

                for (const auto &delete_edge_pair : delete_edge_pairs) {
                    const auto out_edges_v = graph.out_edges_view(vertices.at(delete_edge_pair.first));
                    for (auto it = out_edges_v.begin(); it != out_edges_v.end(); it = out_edges_v.next(it)) {
                        const edge_descriptor &edge_desc = out_edges_v[it];
                        if (graph.vertex_index(graph.target(edge_desc)) == delete_edge_pair.second) {
                            delete_edges.emplace(graph.edge_index(edge_desc));
                        }
                    }

                    graph.remove_edge(vertices.at(delete_edge_pair.first),
                            vertices.at(delete_edge_pair.second));
                }

                for (const std::size_t clear_vertex : clear_vertices) {
                    const auto out_edges_v = graph.out_edges_view(vertices.at(clear_vertex));
                    for (auto it = out_edges_v.begin(); it != out_edges_v.end(); it = out_edges_v.next(it)) {
                        const edge_descriptor &edge_desc = out_edges_v[it];
                        delete_edges.emplace(graph.edge_index(edge_desc));
                    }

                    delete_vertices.emplace(clear_vertex);

                    graph.clear_vertex(vertices.at(clear_vertex));
                }

                for (const std::size_t delete_vertex : delete_vertices) {
                    graph.remove_vertex(vertices.at(delete_vertex));
                }

                BOOST_TEST_MESSAGE("graph after deletion:");
                print(graph);

                BOOST_TEST_MESSAGE("graph verification:");
                verify(graph, delete_vertices, delete_edges);
            }
        };

        BOOST_TEST_MESSAGE("undirected std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::undirected, void, void, std::list>>();

        BOOST_TEST_MESSAGE("directional std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::directional, void, void, std::list>>();

        BOOST_TEST_MESSAGE("bidirectional std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::bidirectional, void, void, std::list>>();

    }

    BOOST_AUTO_TEST_CASE(mutable_graph_edge_iterator_delete_test) {

        std::size_t vertex_size = 100;
        std::size_t edge_size = 300;

        std::random_device random_device;
        auto seed = random_device();
        BOOST_TEST_MESSAGE("Used seed: " + std::to_string(seed));
        std::default_random_engine random_engine{seed};
        std::uniform_int_distribution<std::size_t> random_vertex{0, vertex_size - 1};
        std::uniform_int_distribution<std::size_t> random_edge{0, edge_size - 1};

        auto verify = []<typename Graph>(const Graph &graph, const std::set<std::size_t> &delete_edges) {
            const auto vertices_v = graph.vertices_view();
            for (auto vertex_desc = vertices_v.begin();
                 vertex_desc != vertices_v.end(); vertex_desc = vertices_v.next(vertex_desc)) {
                const auto &vertex = vertices_v[vertex_desc];

                const auto out_edges_v = graph.out_edges_view(vertex_desc);
                for (auto out_edge_desc = out_edges_v.begin();
                     out_edge_desc != out_edges_v.end(); out_edge_desc = out_edges_v.next(out_edge_desc)) {
                    const auto &edge_desc = out_edges_v[out_edge_desc];

                    BOOST_CHECK(not delete_edges.contains(graph.edge_index(edge_desc)));

                    if constexpr (Graph::is_undirected_v) {
                        BOOST_CHECK((std::addressof(vertices_v[graph.source(edge_desc)]) ==
                                    std::addressof(vertex)) ||
                                (std::addressof(vertices_v[graph.target(edge_desc)]) ==
                                    std::addressof(vertex)));
                    }

                    if constexpr (Graph::is_directional_v || Graph::is_bidirectional_v) {
                        BOOST_CHECK_EQUAL(std::addressof(vertices_v[graph.source(edge_desc)]),
                                std::addressof(vertex));
                    }

                }

                if constexpr (Graph::is_bidirectional_v) {
                    const auto in_edges_v = graph.in_edges_view(vertex_desc);
                    for (auto in_edge_desc = in_edges_v.begin();
                         in_edge_desc != in_edges_v.end(); in_edge_desc = in_edges_v.next(in_edge_desc)) {
                        BOOST_CHECK_EQUAL(std::addressof(vertices_v[graph.target(in_edges_v[in_edge_desc])]),
                                std::addressof(vertex));
                    }
                }

            }

            const auto edges_v = graph.edges_view();
            for (auto edge_desc = edges_v.begin(); edge_desc != edges_v.end(); edge_desc = edges_v.next(edge_desc)) {
                BOOST_CHECK(not delete_edges.contains(graph.edge_index(edge_desc)));
            }
        };

        auto test = [&]<typename Graph>() {
            Graph graph;

            using vertex_descriptor = typename Graph::vertex_descriptor;
            using edge_descriptor = typename Graph::edge_descriptor;

            std::vector<vertex_descriptor> vertices;
            vertices.reserve(vertex_size);
            for (std::size_t i = 0; i < vertex_size; ++i) {
                vertices.emplace_back(graph.add_vertex());
            }

            std::vector<edge_descriptor> edges;
            edges.reserve(edge_size);
            for (std::size_t i = 0; i < edge_size; ++i) {
                edges.emplace_back(graph.add_edge(vertices[random_vertex(random_engine)],
                        vertices[random_vertex(random_engine)]));
            }

            BOOST_CHECK_EQUAL(graph.vertices_view().size(), vertex_size);
            BOOST_CHECK_EQUAL(graph.edges_view().size(), edge_size);

            std::size_t edge_deletes = 100;
            std::set<std::size_t> random_delete_edges;
            while (random_delete_edges.size() < edge_deletes) {
                random_delete_edges.emplace(random_edge(random_engine));
            }

            for (std::size_t delete_edge : random_delete_edges) {
                graph.remove_edge(edges.at(delete_edge));
            }

            BOOST_CHECK_EQUAL(graph.vertices_view().size(), vertex_size);
            BOOST_CHECK_EQUAL(graph.edges_view().size(), edge_size - edge_deletes);

            verify(graph, random_delete_edges);
        };

        BOOST_TEST_MESSAGE("undirected std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::undirected, void, void, std::list>>();

        BOOST_TEST_MESSAGE("directional std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::directional, void, void, std::list>>();

        BOOST_TEST_MESSAGE("bidirectional std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::bidirectional, void, void, std::list>>();

    }

    BOOST_AUTO_TEST_CASE(mutable_graph_edge_vertices_delete_test) {

        std::size_t vertex_size = 100;
        std::size_t edge_size = 300;

        std::random_device random_device;
        auto seed = random_device();
        BOOST_TEST_MESSAGE("Used seed: " + std::to_string(seed));
        std::default_random_engine random_engine{seed};
        std::uniform_int_distribution<std::size_t> random_vertex{0, vertex_size - 1};
        std::uniform_int_distribution<std::size_t> random_edge{0, edge_size - 1};

        auto verify = []<typename Graph>(const Graph &graph, const std::set<std::size_t> &delete_edges) {
            const auto vertices_v = graph.vertices_view();
            for (auto vertex_desc = vertices_v.begin();
                 vertex_desc != vertices_v.end(); vertex_desc = vertices_v.next(vertex_desc)) {
                const auto &vertex = vertices_v[vertex_desc];

                const auto out_edges_v = graph.out_edges_view(vertex_desc);
                for (auto out_edge_desc = out_edges_v.begin();
                     out_edge_desc != out_edges_v.end(); out_edge_desc = out_edges_v.next(out_edge_desc)) {
                    const auto &edge_desc = out_edges_v[out_edge_desc];

                    BOOST_CHECK(not delete_edges.contains(graph.edge_index(edge_desc)));

                    if constexpr (Graph::is_undirected_v) {
                        BOOST_CHECK((std::addressof(vertices_v[graph.source(edge_desc)]) ==
                                    std::addressof(vertex)) ||
                                (std::addressof(vertices_v[graph.target(edge_desc)]) ==
                                    std::addressof(vertex)));
                    }

                    if constexpr (Graph::is_directional_v || Graph::is_bidirectional_v) {
                        BOOST_CHECK_EQUAL(std::addressof(vertices_v[graph.source(edge_desc)]),
                                std::addressof(vertex));
                    }

                }

                if constexpr (Graph::is_bidirectional_v) {
                    const auto in_edges_v = graph.in_edges_view(vertex_desc);
                    for (auto in_edge_desc = in_edges_v.begin();
                         in_edge_desc != in_edges_v.end(); in_edge_desc = in_edges_v.next(in_edge_desc)) {
                        BOOST_CHECK_EQUAL(std::addressof(vertices_v[graph.target(in_edges_v[in_edge_desc])]),
                                std::addressof(vertex));
                    }
                }

            }

            const auto edges_v = graph.edges_view();
            for (auto edge_desc = edges_v.begin(); edge_desc != edges_v.end(); edge_desc = edges_v.next(edge_desc)) {
                BOOST_CHECK(not delete_edges.contains(graph.edge_index(edge_desc)));
            }
        };

        auto test = [&]<typename Graph>() {
            Graph graph;

            using vertex_descriptor = typename Graph::vertex_descriptor;
            using edge_descriptor = typename Graph::edge_descriptor;

            std::vector<vertex_descriptor> vertices;
            vertices.reserve(vertex_size);
            for (std::size_t i = 0; i < vertex_size; ++i) {
                vertices.emplace_back(graph.add_vertex());
            }

            std::vector<std::pair<std::size_t, std::size_t>> random_edges;
            random_edges.reserve(edge_size);
            for (std::size_t i = 0; i < edge_size; ++i) {
                random_edges.emplace_back(random_vertex(random_engine), random_vertex(random_engine));
            }

            std::vector<edge_descriptor> edges;
            edges.reserve(edge_size);
            for (std::size_t i = 0; i < edge_size; ++i) {
                edges.emplace_back(graph.add_edge(vertices.at(random_edges.at(i).first),
                        vertices.at(random_edges.at(i).second)));
            }

            BOOST_CHECK_EQUAL(graph.vertices_view().size(), vertex_size);
            BOOST_CHECK_EQUAL(graph.edges_view().size(), edge_size);

            std::size_t edge_deletes = 100;
            std::set<std::size_t> random_delete_edges;
            while (random_delete_edges.size() < edge_deletes) {
                random_delete_edges.emplace(random_edge(random_engine));
            }

            for (std::size_t delete_edge : random_delete_edges) {
                graph.remove_edge(vertices.at(random_edges.at(delete_edge).first),
                        vertices.at(random_edges.at(delete_edge).second));
            }

            BOOST_CHECK_EQUAL(graph.vertices_view().size(), vertex_size);
            BOOST_CHECK_LE(graph.edges_view().size(), edge_size - edge_deletes);

            verify(graph, random_delete_edges);
        };

        BOOST_TEST_MESSAGE("undirected std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::undirected, void, void, std::list>>();

        BOOST_TEST_MESSAGE("directional std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::directional, void, void, std::list>>();

        BOOST_TEST_MESSAGE("bidirectional std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::bidirectional, void, void, std::list>>();

    }

    BOOST_AUTO_TEST_CASE(mutable_graph_vertices_delete_test) {

        std::size_t vertex_size = 100;
        std::size_t edge_size = 300;

        std::random_device random_device;
        auto seed = random_device();
        BOOST_TEST_MESSAGE("Used seed: " + std::to_string(seed));
        std::default_random_engine random_engine{seed};
        std::uniform_int_distribution<std::size_t> random_vertex{0, vertex_size - 1};

        auto verify = []<typename Graph>(const Graph &graph, const std::set<std::size_t> &delete_vertices) {
            const auto vertices_v = graph.vertices_view();
            for (auto vertex_desc = vertices_v.begin();
                 vertex_desc != vertices_v.end(); vertex_desc = vertices_v.next(vertex_desc)) {
                const auto &vertex = vertices_v[vertex_desc];

                BOOST_CHECK(not delete_vertices.contains(graph.vertex_index(vertex_desc)));

                const auto out_edges_v = graph.out_edges_view(vertex_desc);
                for (auto out_edge_desc = out_edges_v.begin();
                     out_edge_desc != out_edges_v.end(); out_edge_desc = out_edges_v.next(out_edge_desc)) {
                    const auto &edge_desc = out_edges_v[out_edge_desc];

                    if constexpr (Graph::is_undirected_v) {
                        BOOST_CHECK((std::addressof(vertices_v[graph.source(edge_desc)]) ==
                                    std::addressof(vertex)) ||
                                (std::addressof(vertices_v[graph.target(edge_desc)]) ==
                                    std::addressof(vertex)));
                    }

                    if constexpr (Graph::is_directional_v || Graph::is_bidirectional_v) {
                        BOOST_CHECK_EQUAL(std::addressof(vertices_v[graph.source(edge_desc)]),
                                std::addressof(vertex));
                    }

                }

                if constexpr (Graph::is_bidirectional_v) {
                    const auto in_edges_v = graph.in_edges_view(vertex_desc);
                    for (auto in_edge_desc = in_edges_v.begin();
                         in_edge_desc != in_edges_v.end(); in_edge_desc = in_edges_v.next(in_edge_desc)) {
                        BOOST_CHECK_EQUAL(std::addressof(vertices_v[graph.target(in_edges_v[in_edge_desc])]),
                                std::addressof(vertex));
                    }
                }

            }
        };

        auto test = [&]<typename Graph>() {
            Graph graph;

            using vertex_descriptor = typename Graph::vertex_descriptor;
            using edge_descriptor = typename Graph::edge_descriptor;

            std::vector<vertex_descriptor> vertices;
            vertices.reserve(vertex_size);
            for (std::size_t i = 0; i < vertex_size; ++i) {
                vertices.emplace_back(graph.add_vertex());
            }

            std::vector<edge_descriptor> edges;
            edges.reserve(edge_size);
            for (std::size_t i = 0; i < edge_size; ++i) {
                edges.emplace_back(graph.add_edge(vertices[random_vertex(random_engine)],
                        vertices[random_vertex(random_engine)]));
            }

            BOOST_CHECK_EQUAL(graph.vertices_view().size(), vertex_size);
            BOOST_CHECK_EQUAL(graph.edges_view().size(), edge_size);

            std::size_t vertex_deletes = 50;
            std::set<std::size_t> random_delete_vertices;
            while (random_delete_vertices.size() < vertex_deletes) {
                random_delete_vertices.emplace(random_vertex(random_engine));
            }

            for (std::size_t delete_vertex : random_delete_vertices) {
                vertex_descriptor &vertex_it = vertices.at(delete_vertex);
                graph.clear_vertex(vertex_it);
                graph.remove_vertex(vertex_it);
            }

            BOOST_CHECK_EQUAL(graph.vertices_view().size(), vertex_size - vertex_deletes);
            BOOST_CHECK_LE(graph.edges_view().size(), edge_size);

            verify(graph, random_delete_vertices);
        };

        BOOST_TEST_MESSAGE("undirected std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::undirected, void, void, std::list>>();

        BOOST_TEST_MESSAGE("directional std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::directional, void, void, std::list>>();

        BOOST_TEST_MESSAGE("bidirectional std::list");
        test.template operator()<map_matching_2::graph::adjacency_list<
            std::list, std::list, map_matching_2::graph::bidirectional, void, void, std::list>>();

    }

    BOOST_AUTO_TEST_CASE(mutable_directional_graph_csr_conversion_test) {

        auto test = [&]<typename AdjacencyList, typename CompressedSparseRow>() {
            AdjacencyList graph;

            {
                auto vertex_a = graph.add_vertex();
                auto vertex_b = graph.add_vertex();
                auto vertex_c = graph.add_vertex();

                auto edge_ab = graph.add_edge(vertex_a, vertex_b);
                auto edge_bc = graph.add_edge(vertex_b, vertex_c);
            }

            CompressedSparseRow csr;
            csr.from_adjacency_list(graph);

            auto vertex_a = csr.vertices_view().begin();
            auto vertex_b = csr.vertices_view().next(vertex_a);
            auto vertex_c = csr.vertices_view().next(vertex_a, 2);

            auto edge_ab = csr.edges_view().begin();
            auto edge_bc = csr.edges_view().next(edge_ab);

            BOOST_CHECK_EQUAL(csr.vertices_view().size(), 3);
            BOOST_CHECK_EQUAL(csr.out_edges_view(vertex_a).size(), 1);
            BOOST_CHECK_EQUAL(csr.out_edges_view(vertex_b).size(), 1);
            BOOST_CHECK_EQUAL(csr.edges_view().size(), 2);

            std::cout << "" << csr.out_edges_view(vertex_a).front().target << std::endl;

            BOOST_CHECK_EQUAL(std::addressof(csr.edges_view()[csr.out_edges_view(vertex_a).begin()]),
                    std::addressof(csr.edges_view()[edge_ab]));
            BOOST_CHECK_EQUAL(std::addressof(csr.vertices_view()[csr.source(edge_ab)]),
                    std::addressof(csr.vertices_view()[vertex_a]));
            BOOST_CHECK_EQUAL(std::addressof(csr.vertices_view()[csr.target(edge_ab)]),
                    std::addressof(csr.vertices_view()[vertex_b]));
            BOOST_CHECK_EQUAL(std::addressof(csr.edges_view()[csr.out_edges_view(vertex_b).begin()]),
                    std::addressof(csr.edges_view()[edge_bc]));
            BOOST_CHECK_EQUAL(std::addressof(csr.vertices_view()[csr.source(edge_bc)]),
                    std::addressof(csr.vertices_view()[vertex_b]));
            BOOST_CHECK_EQUAL(std::addressof(csr.vertices_view()[csr.target(edge_bc)]),
                    std::addressof(csr.vertices_view()[vertex_c]));
        };

        test.template operator()<map_matching_2::graph::adjacency_list<
                std::list, std::list, map_matching_2::graph::undirected, void, void, std::list>,
            map_matching_2::graph::compressed_sparse_row<
                std::vector, std::vector, map_matching_2::graph::directional, void, void>>();

        test.template operator()<map_matching_2::graph::adjacency_list<
                std::list, std::list, map_matching_2::graph::directional, void, void, std::list>,
            map_matching_2::graph::compressed_sparse_row<
                std::vector, std::vector, map_matching_2::graph::directional, void, void>>();

        test.template operator()<map_matching_2::graph::adjacency_list<
                std::list, std::list, map_matching_2::graph::bidirectional, void, void, std::list>,
            map_matching_2::graph::compressed_sparse_row<
                std::vector, std::vector, map_matching_2::graph::directional, void, void>>();

    }

    BOOST_AUTO_TEST_CASE(mmap_bdir_mutable_graph_construction_test) {

        using adjacency_list = map_matching_2::graph::adjacency_list<
            boost::interprocess::list,
            boost::interprocess::list,
            map_matching_2::graph::bidirectional,
            void,
            void,
            boost::interprocess::list,
            boost::interprocess::allocator<void, boost::interprocess::managed_mapped_file::segment_manager>>;

        using allocator_type = typename adjacency_list::allocator_type;

        const char *graph_id = "graph";
        const char *graph_file = "graph_construct.bin";
        std::size_t graph_size = 20 * 1024;

        boost::interprocess::file_mapping::remove(graph_file);

        {
            auto *graph_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_or_create, graph_file, graph_size
            };
            auto *graph_allocator = new allocator_type{graph_mmap_file->get_segment_manager()};
            auto *graph = graph_mmap_file->construct<adjacency_list>(graph_id)(*graph_allocator);

            {
                auto vertex_a = graph->add_vertex();
                auto vertex_b = graph->add_vertex();
                auto vertex_c = graph->add_vertex();

                auto edge_ab = graph->add_edge(vertex_a, vertex_b);
                auto edge_bc = graph->add_edge(vertex_b, vertex_c);

                BOOST_CHECK_EQUAL(graph->vertices_view().size(), 3);
                BOOST_CHECK_EQUAL(graph->edges_view().size(), 2);

                BOOST_CHECK_EQUAL(std::addressof(graph->edges_view()[graph->out_edges_view(vertex_a).front()]),
                        std::addressof(graph->edges_view()[edge_ab]));
                BOOST_CHECK_EQUAL(std::addressof(graph->edges_view()[graph->in_edges_view(vertex_b).front()]),
                        std::addressof(graph->edges_view()[edge_ab]));
                BOOST_CHECK_EQUAL(std::addressof(graph->vertices_view()[graph->source(edge_ab)]),
                        std::addressof(graph->vertices_view()[vertex_a]));
                BOOST_CHECK_EQUAL(std::addressof(graph->vertices_view()[graph->target(edge_ab)]),
                        std::addressof(graph->vertices_view()[vertex_b]));
                BOOST_CHECK_EQUAL(std::addressof(graph->edges_view()[graph->out_edges_view(vertex_b).front()]),
                        std::addressof(graph->edges_view()[edge_bc]));
                BOOST_CHECK_EQUAL(std::addressof(graph->edges_view()[graph->in_edges_view(vertex_c).front()]),
                        std::addressof(graph->edges_view()[edge_bc]));
                BOOST_CHECK_EQUAL(std::addressof(graph->vertices_view()[graph->source(edge_bc)]),
                        std::addressof(graph->vertices_view()[vertex_b]));
                BOOST_CHECK_EQUAL(std::addressof(graph->vertices_view()[graph->target(edge_bc)]),
                        std::addressof(graph->vertices_view()[vertex_c]));
            }

            delete graph_allocator;
            delete graph_mmap_file;
        }

        boost::interprocess::managed_mapped_file::shrink_to_fit(graph_file);

        {
            auto *graph_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_only, graph_file
            };
            auto *graph = graph_mmap_file->find<adjacency_list>(graph_id).first;

            {
                auto vertices_view = graph->vertices_view();
                auto vertex_a = vertices_view.begin();
                auto vertex_b = vertices_view.next(vertex_a);
                auto vertex_c = vertices_view.next(vertex_a, 2);

                auto edge_ab = vertices_view[vertex_a].out_edges_view().front();
                auto edge_bc = vertices_view[vertex_b].out_edges_view().front();

                BOOST_CHECK_EQUAL(graph->vertices_view().size(), 3);
                BOOST_CHECK_EQUAL(graph->edges_view().size(), 2);

                BOOST_CHECK_EQUAL(std::addressof(graph->edges_view()[graph->out_edges_view(vertex_a).front()]),
                        std::addressof(graph->edges_view()[edge_ab]));
                BOOST_CHECK_EQUAL(std::addressof(graph->edges_view()[graph->in_edges_view(vertex_b).front()]),
                        std::addressof(graph->edges_view()[edge_ab]));
                BOOST_CHECK_EQUAL(std::addressof(graph->vertices_view()[graph->source(edge_ab)]),
                        std::addressof(graph->vertices_view()[vertex_a]));
                BOOST_CHECK_EQUAL(std::addressof(graph->vertices_view()[graph->target(edge_ab)]),
                        std::addressof(graph->vertices_view()[vertex_b]));
                BOOST_CHECK_EQUAL(std::addressof(graph->edges_view()[graph->out_edges_view(vertex_b).front()]),
                        std::addressof(graph->edges_view()[edge_bc]));
                BOOST_CHECK_EQUAL(std::addressof(graph->edges_view()[graph->in_edges_view(vertex_c).front()]),
                        std::addressof(graph->edges_view()[edge_bc]));
                BOOST_CHECK_EQUAL(std::addressof(graph->vertices_view()[graph->source(edge_bc)]),
                        std::addressof(graph->vertices_view()[vertex_b]));
                BOOST_CHECK_EQUAL(std::addressof(graph->vertices_view()[graph->target(edge_bc)]),
                        std::addressof(graph->vertices_view()[vertex_c]));
            }

            delete graph_mmap_file;
        }
    }

    BOOST_AUTO_TEST_CASE(mmap_bdir_mutable_graph_resize_test) {

        using adjacency_list = map_matching_2::graph::adjacency_list<
            boost::interprocess::list,
            boost::interprocess::list,
            map_matching_2::graph::bidirectional,
            void,
            void,
            boost::interprocess::list,
            boost::interprocess::allocator<void, boost::interprocess::managed_mapped_file::segment_manager>>;

        using allocator_type = typename adjacency_list::allocator_type;
        using vertex_descriptor = typename adjacency_list::vertex_descriptor;
        using edge_descriptor = typename adjacency_list::edge_descriptor;

        const char *graph_id = "graph";
        const char *graph_file = "graph_resize.bin";
        std::size_t graph_size = 20 * 1024;

        std::size_t vertex_size = 1000;
        std::size_t edge_size = 3000;
        // std::size_t vertex_size = 100000;
        // std::size_t edge_size = 300000;
        std::size_t max_test = 100;

        std::random_device random_device;
        auto seed = random_device();
        BOOST_TEST_MESSAGE("Used seed: " + std::to_string(seed));
        std::default_random_engine random_engine{seed};
        std::uniform_int_distribution<std::size_t> random_index{0, vertex_size - 1};

        std::vector<std::pair<std::size_t, std::size_t>> random_edges;
        random_edges.reserve(edge_size);
        for (std::size_t i = 0; i < edge_size; ++i) {
            random_edges.emplace_back(random_index(random_engine), random_index(random_engine));
        }

        std::map<std::size_t, std::multiset<std::size_t>> random_out_edges;
        for (std::size_t i = 0; i < edge_size; ++i) {
            auto [source, target] = random_edges[i];
            if (not random_out_edges.contains(source)) {
                random_out_edges.emplace(source, std::multiset<std::size_t>{});
            }
            random_out_edges[source].emplace(target);
        }

        boost::interprocess::file_mapping::remove(graph_file);

        {
            auto *graph_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_or_create, graph_file, graph_size
            };
            auto *graph_allocator = new allocator_type{graph_mmap_file->get_segment_manager()};
            auto *graph = graph_mmap_file->construct<adjacency_list>(graph_id)(*graph_allocator);

            {
                for (std::size_t i = 0; i < vertex_size; ++i) {
                    map_matching_2::io::memory_mapped::retry_alloc([&]() {
                                graph->add_vertex();
                            }, [&](auto &ex) {
                                std::size_t previous_size = graph->vertices_view().size();
                                map_matching_2::io::memory_mapped::mmap_auto_grow(
                                        graph_mmap_file, graph_allocator, graph, graph_file, graph_size, graph_id);
                                if (graph->vertices_view().size() != previous_size) {
                                    std::rethrow_exception(std::current_exception());
                                }
                                BOOST_TEST_MESSAGE("Resized due to add_vertex(), new size: " << graph_size);
                            });
                }

                std::vector<vertex_descriptor> vertices;
                vertices.reserve(vertex_size);
                auto vertices_v = graph->vertices_view();
                for (auto it = vertices_v.begin(); it != vertices_v.end(); it = vertices_v.next(it)) {
                    vertices.emplace_back(it);
                }

                BOOST_CHECK_EQUAL(vertex_size, graph->vertices_view().size());
                BOOST_CHECK_EQUAL(vertices.size(), graph->vertices_view().size());

                for (std::size_t i = 0; i < edge_size; ++i) {
                    map_matching_2::io::memory_mapped::retry_alloc([&]() {
                                graph->add_edge(vertices.at(random_edges.at(i).first),
                                        vertices.at(random_edges.at(i).second));
                            }, [&](auto &ex) {
                                std::size_t previous_size = graph->edges_view().size();
                                // clear vertices because they will be invalidated
                                vertices.clear();
                                map_matching_2::io::memory_mapped::mmap_auto_grow(
                                        graph_mmap_file, graph_allocator, graph, graph_file, graph_size, graph_id);
                                if (graph->edges_view().size() != previous_size) {
                                    std::rethrow_exception(std::current_exception());
                                }
                                BOOST_TEST_MESSAGE("Resized due to add_edge(), new size: " << graph_size);
                                // reindex vertices after grow
                                auto vertices_v = graph->vertices_view();
                                for (auto it = vertices_v.begin(); it != vertices_v.end(); it = vertices_v.next(it)) {
                                    vertices.emplace_back(it);
                                }
                            });
                }

                BOOST_CHECK_EQUAL(edge_size, graph->edges_view().size());
            }

            delete graph_allocator;
            delete graph_mmap_file;
        }

        boost::interprocess::managed_mapped_file::shrink_to_fit(graph_file);

        {
            auto *graph_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_only, graph_file
            };
            auto *graph = graph_mmap_file->find<adjacency_list>(graph_id).first;

            BOOST_CHECK_EQUAL(vertex_size, graph->vertices_view().size());
            BOOST_CHECK_EQUAL(edge_size, graph->edges_view().size());

            auto vertices_v = graph->vertices_view();
            auto edges_v = graph->edges_view();
            for (auto it = vertices_v.begin();
                 it != vertices_v.end(); it = vertices_v.next(it, vertex_size / max_test)) {
                auto &source = vertices_v[it];
                auto out_edges_v = source.out_edges_view();

                BOOST_CHECK_EQUAL(out_edges_v.size(), random_out_edges[source.index].size());

                if (not out_edges_v.empty()) {

                    BOOST_CHECK(random_out_edges.contains(source.index));

                    for (auto oeit = out_edges_v.begin(); oeit != out_edges_v.end(); oeit = out_edges_v.next(oeit)) {
                        auto &edge = edges_v[out_edges_v[oeit]];

                        BOOST_CHECK(random_out_edges[source.index].contains(vertices_v[edge.target].index));
                    }
                }
            }

            delete graph_mmap_file;
        }
    }

    BOOST_AUTO_TEST_CASE(mmap_graph_conversion_test) {

        using adjacency_list = map_matching_2::graph::adjacency_list<
            boost::interprocess::list,
            boost::interprocess::list,
            map_matching_2::graph::bidirectional,
            void,
            void,
            boost::interprocess::list,
            boost::interprocess::allocator<void, boost::interprocess::managed_mapped_file::segment_manager>>;

        using compressed_sparse_row = map_matching_2::graph::compressed_sparse_row<
            boost_interprocess_vector,
            boost_interprocess_vector,
            map_matching_2::graph::directional,
            void,
            void,
            boost::interprocess::allocator<void, boost::interprocess::managed_mapped_file::segment_manager>>;

        using graph_allocator_type = typename adjacency_list::allocator_type;
        using csr_allocator_type = typename compressed_sparse_row::allocator_type;

        const char *graph_id = "graph";
        const char *graph_file = "graph_conversion_adj.bin";
        std::size_t graph_size = 20 * 1024;

        const char *csr_id = "graph";
        const char *csr_file = "graph_conversion_csr.bin";
        std::size_t csr_size = 20 * 1024;

        boost::interprocess::file_mapping::remove(graph_file);
        boost::interprocess::file_mapping::remove(csr_file);

        {
            auto *graph_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_or_create, graph_file, graph_size
            };
            auto *graph_allocator = new graph_allocator_type{graph_mmap_file->get_segment_manager()};
            auto *graph = graph_mmap_file->construct<adjacency_list>(graph_id)(*graph_allocator);

            {
                auto vertex_a = graph->add_vertex();
                auto vertex_b = graph->add_vertex();
                auto vertex_c = graph->add_vertex();

                auto edge_ab = graph->add_edge(vertex_a, vertex_b);
                auto edge_bc = graph->add_edge(vertex_b, vertex_c);
            }

            {
                auto *csr_mmap_file = new boost::interprocess::managed_mapped_file{
                        boost::interprocess::open_or_create, csr_file, csr_size
                };
                auto *csr_allocator = new csr_allocator_type{csr_mmap_file->get_segment_manager()};
                auto *csr = csr_mmap_file->construct<compressed_sparse_row>(csr_id)(*csr_allocator);

                csr->from_adjacency_list(*graph, *csr_allocator);

                delete csr_allocator;
                delete csr_mmap_file;
            }

            boost::interprocess::managed_mapped_file::shrink_to_fit(csr_file);

            delete graph_allocator;
            delete graph_mmap_file;
        }

        boost::interprocess::managed_mapped_file::shrink_to_fit(graph_file);

        {
            auto *csr_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_only, csr_file
            };
            auto *csr = csr_mmap_file->find<compressed_sparse_row>(csr_id).first;

            {
                auto vertex_a = csr->vertices_view().begin();
                auto vertex_b = csr->vertices_view().next(vertex_a);
                auto vertex_c = csr->vertices_view().next(vertex_a, 2);

                auto edge_ab = csr->edges_view().begin();
                auto edge_bc = csr->edges_view().next(edge_ab);

                BOOST_CHECK_EQUAL(csr->vertices_view().size(), 3);
                BOOST_CHECK_EQUAL(csr->edges_view().size(), 2);

                BOOST_CHECK_EQUAL(std::addressof(csr->edges_view()[csr->out_edges_view(vertex_a).begin()]),
                        std::addressof(csr->edges_view()[edge_ab]));
                BOOST_CHECK_EQUAL(std::addressof(csr->vertices_view()[csr->source(edge_ab)]),
                        std::addressof(csr->vertices_view()[vertex_a]));
                BOOST_CHECK_EQUAL(std::addressof(csr->vertices_view()[csr->target(edge_ab)]),
                        std::addressof(csr->vertices_view()[vertex_b]));
                BOOST_CHECK_EQUAL(std::addressof(csr->edges_view()[csr->out_edges_view(vertex_b).begin()]),
                        std::addressof(csr->edges_view()[edge_bc]));
                BOOST_CHECK_EQUAL(std::addressof(csr->vertices_view()[csr->source(edge_bc)]),
                        std::addressof(csr->vertices_view()[vertex_b]));
                BOOST_CHECK_EQUAL(std::addressof(csr->vertices_view()[csr->target(edge_bc)]),
                        std::addressof(csr->vertices_view()[vertex_c]));
            }

            delete csr_mmap_file;
        }
    }

    BOOST_AUTO_TEST_CASE(mmap_graph_conversion_resize_test) {

        using adjacency_list = map_matching_2::graph::adjacency_list<
            boost::interprocess::list,
            boost::interprocess::list,
            map_matching_2::graph::bidirectional,
            void,
            void,
            boost::interprocess::list,
            boost::interprocess::allocator<void, boost::interprocess::managed_mapped_file::segment_manager>>;

        using compressed_sparse_row = map_matching_2::graph::compressed_sparse_row<
            boost_interprocess_vector,
            boost_interprocess_vector,
            map_matching_2::graph::directional,
            void,
            void,
            boost::interprocess::allocator<void, boost::interprocess::managed_mapped_file::segment_manager>>;

        using graph_allocator_type = typename adjacency_list::allocator_type;
        using vertex_descriptor = typename adjacency_list::vertex_descriptor;
        using edge_descriptor = typename adjacency_list::edge_descriptor;

        using csr_allocator_type = typename compressed_sparse_row::allocator_type;

        const char *graph_id = "graph";
        const char *graph_file = "graph_resize_adj.bin";
        std::size_t graph_size = 20 * 1024;

        const char *csr_id = "graph";
        const char *csr_file = "graph_resize_csr.bin";
        std::size_t csr_size = 20 * 1024;

        std::size_t vertex_size = 1000;
        std::size_t edge_size = 3000;
        // std::size_t vertex_size = 100000;
        // std::size_t edge_size = 300000;
        std::size_t max_test = 100;

        std::random_device random_device;
        auto seed = random_device();
        BOOST_TEST_MESSAGE("Used seed: " + std::to_string(seed));
        std::default_random_engine random_engine{seed};
        std::uniform_int_distribution<std::size_t> random_index{0, vertex_size - 1};

        std::vector<std::pair<std::size_t, std::size_t>> random_edges;
        random_edges.reserve(edge_size);
        for (std::size_t i = 0; i < edge_size; ++i) {
            random_edges.emplace_back(random_index(random_engine), random_index(random_engine));
        }

        std::map<std::size_t, std::multiset<std::size_t>> random_out_edges;
        for (std::size_t i = 0; i < edge_size; ++i) {
            auto [source, target] = random_edges[i];
            if (not random_out_edges.contains(source)) {
                random_out_edges.emplace(source, std::multiset<std::size_t>{});
            }
            random_out_edges[source].emplace(target);
        }

        boost::interprocess::file_mapping::remove(graph_file);
        boost::interprocess::file_mapping::remove(csr_file);

        {
            auto *graph_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_or_create, graph_file, graph_size
            };
            auto *graph_allocator = new graph_allocator_type{graph_mmap_file->get_segment_manager()};
            auto *graph = graph_mmap_file->construct<adjacency_list>(graph_id)(*graph_allocator);

            {
                for (std::size_t i = 0; i < vertex_size; ++i) {
                    map_matching_2::io::memory_mapped::retry_alloc([&]() {
                                graph->add_vertex();
                            }, [&](auto &ex) {
                                std::size_t previous_size = graph->vertices_view().size();
                                map_matching_2::io::memory_mapped::mmap_auto_grow(
                                        graph_mmap_file, graph_allocator, graph, graph_file, graph_size, graph_id);
                                if (graph->vertices_view().size() != previous_size) {
                                    std::rethrow_exception(std::current_exception());
                                }
                                BOOST_TEST_MESSAGE("Resized due to add_vertex(), new size: " << graph_size);
                            });
                }

                std::vector<vertex_descriptor> vertices;
                vertices.reserve(vertex_size);
                auto vertices_v = graph->vertices_view();
                for (auto it = vertices_v.begin(); it != vertices_v.end(); it = vertices_v.next(it)) {
                    vertices.emplace_back(it);
                }

                BOOST_CHECK_EQUAL(vertex_size, graph->vertices_view().size());
                BOOST_CHECK_EQUAL(vertices.size(), graph->vertices_view().size());

                for (std::size_t i = 0; i < edge_size; ++i) {
                    map_matching_2::io::memory_mapped::retry_alloc([&]() {
                                graph->add_edge(vertices.at(random_edges.at(i).first),
                                        vertices.at(random_edges.at(i).second));
                            }, [&](auto &ex) {
                                std::size_t previous_size = graph->edges_view().size();
                                // clear vertices because they will be invalidated
                                vertices.clear();
                                map_matching_2::io::memory_mapped::mmap_auto_grow(
                                        graph_mmap_file, graph_allocator, graph, graph_file, graph_size, graph_id);
                                if (graph->edges_view().size() != previous_size) {
                                    std::rethrow_exception(std::current_exception());
                                }
                                BOOST_TEST_MESSAGE("Resized due to add_edge(), new size: " << graph_size);
                                // reindex vertices after grow
                                auto vertices_v = graph->vertices_view();
                                for (auto it = vertices_v.begin(); it != vertices_v.end(); it = vertices_v.next(it)) {
                                    vertices.emplace_back(it);
                                }
                            });
                }

                BOOST_CHECK_EQUAL(edge_size, graph->edges_view().size());
            }

            {
                auto *csr_mmap_file = new boost::interprocess::managed_mapped_file{
                        boost::interprocess::open_or_create, csr_file, csr_size
                };
                auto *csr_allocator = new csr_allocator_type{csr_mmap_file->get_segment_manager()};
                auto *csr = csr_mmap_file->construct<compressed_sparse_row>(csr_id)(*csr_allocator);

                std::size_t csr_max_size = 1 * 1024 * 1024 * 1024; // 1 GB
                auto csr_retries = static_cast<std::size_t>(std::log2(csr_max_size / csr_size));

                map_matching_2::io::memory_mapped::retry_alloc([&]() {
                            csr->from_adjacency_list(*graph, *csr_allocator);
                        }, [&](auto &ex) {
                            map_matching_2::io::memory_mapped::mmap_auto_grow(
                                    csr_mmap_file, csr_allocator, csr, csr_file, csr_size, csr_id);
                            BOOST_TEST_MESSAGE("Resized due to from_adjacency_list() reserve, new size: " << csr_size);
                        }, csr_retries);

                delete csr_allocator;
                delete csr_mmap_file;
            }

            boost::interprocess::managed_mapped_file::shrink_to_fit(csr_file);

            delete graph_allocator;
            delete graph_mmap_file;
        }

        boost::interprocess::managed_mapped_file::shrink_to_fit(graph_file);

        {
            auto *csr_mmap_file = new boost::interprocess::managed_mapped_file{
                    boost::interprocess::open_only, csr_file
            };
            auto *csr = csr_mmap_file->find<compressed_sparse_row>(csr_id).first;

            BOOST_CHECK_EQUAL(vertex_size, csr->vertices_view().size());
            BOOST_CHECK_EQUAL(edge_size, csr->edges_view().size());

            auto vertices_v = csr->vertices_view();
            auto edges_v = csr->edges_view();
            for (auto it = vertices_v.begin();
                 it != vertices_v.end(); it = vertices_v.next(it, vertex_size / max_test)) {
                auto &source = vertices_v[it];
                auto out_edges_v = csr->out_edges_view(it);

                BOOST_CHECK_EQUAL(out_edges_v.size(), random_out_edges[it].size());

                if (not out_edges_v.empty()) {

                    BOOST_CHECK(random_out_edges.contains(it));

                    for (auto oeit = out_edges_v.begin(); oeit != out_edges_v.end(); oeit = out_edges_v.next(oeit)) {
                        auto &edge = edges_v[oeit];

                        BOOST_CHECK_EQUAL(it, edge.source);
                        BOOST_CHECK(random_out_edges[it].contains(edge.target));
                    }
                }
            }

            delete csr_mmap_file;
        }
    }

BOOST_AUTO_TEST_SUITE_END()
