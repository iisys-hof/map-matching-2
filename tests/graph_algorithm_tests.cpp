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
#include <format>

#include <boost/test/unit_test.hpp>
#include <boost/unordered/unordered_flat_set.hpp>

#include "graph/adjacency_list.hpp"
#include "graph/compressed_sparse_row.hpp"
#include "graph/algorithm/breadth_first_search.hpp"
#include "graph/algorithm/depth_first_search.hpp"
#include "graph/algorithm/dijkstra_shortest_path.hpp"

struct edge {
    double weight;
};

template<typename Type>
struct priority_type_comparator {
    bool operator()(const Type &left, const Type &right) const {
        return left.first > right.first;
    }
};

template<typename Edge>
using adjacency_list_type = map_matching_2::graph::adjacency_list<
    std::list, std::list, map_matching_2::graph::directional, void, Edge, std::list>;
template<typename Edge>
using compressed_sparse_row_type = map_matching_2::graph::compressed_sparse_row<
    std::vector, std::vector, map_matching_2::graph::directional, void, Edge>;

template<typename Graph = adjacency_list_type<void>>
std::pair<Graph, std::vector<typename Graph::vertex_descriptor>> create_tree() {
    Graph graph;

    using vertex_descriptor = typename Graph::vertex_descriptor;
    using edge_descriptor = typename Graph::edge_descriptor;

    std::vector<vertex_descriptor> vertices;
    vertices.reserve(10);
    for (std::size_t i = 0; i < 10; ++i) {
        vertices.emplace_back(graph.add_vertex());
    }

    /*
     *     <- 0 ->  <--
     *   /    |    \    \
     * 1      2      3   \
     * | \    |      |    |
     * 4  5   6      7    |
     *    |              /
     *    8 ------------
     *    |
     *    9
     */

    graph.add_edge(vertices[0], vertices[1]);
    graph.add_edge(vertices[0], vertices[2]);
    graph.add_edge(vertices[0], vertices[3]);
    graph.add_edge(vertices[1], vertices[4]);
    graph.add_edge(vertices[1], vertices[5]);
    graph.add_edge(vertices[2], vertices[6]);
    graph.add_edge(vertices[3], vertices[7]);
    graph.add_edge(vertices[5], vertices[8]);
    graph.add_edge(vertices[8], vertices[9]);
    graph.add_edge(vertices[8], vertices[0]);

    return std::pair{std::move(graph), std::move(vertices)};
}

template<typename Graph = adjacency_list_type<edge>>
std::pair<Graph, std::vector<typename Graph::vertex_descriptor>> create_weighted_graph() {
    Graph graph;

    using vertex_descriptor = typename Graph::vertex_descriptor;
    using edge_descriptor = typename Graph::edge_descriptor;

    std::vector<vertex_descriptor> vertices;
    vertices.reserve(10);
    for (std::size_t i = 0; i < 10; ++i) {
        vertices.emplace_back(graph.add_vertex());
    }

    /*
     * 3 -- 4 -- 6 -- 9
     * |    |    |    |
     * 1 -- 2 -- 5 -- 8
     * |    |   /    /
     * 0 -- 7 ------
     */

    graph.add_edge(vertices[0], vertices[1], edge{2.0});
    graph.add_edge(vertices[0], vertices[7], edge{3.0});
    graph.add_edge(vertices[1], vertices[0], edge{2.0});
    graph.add_edge(vertices[1], vertices[2], edge{5.0});
    graph.add_edge(vertices[1], vertices[3], edge{4.0});
    graph.add_edge(vertices[2], vertices[1], edge{5.0});
    graph.add_edge(vertices[2], vertices[4], edge{2.0});
    graph.add_edge(vertices[2], vertices[5], edge{3.0});
    graph.add_edge(vertices[2], vertices[7], edge{1.0});
    graph.add_edge(vertices[3], vertices[1], edge{4.0});
    graph.add_edge(vertices[3], vertices[4], edge{3.0});
    graph.add_edge(vertices[4], vertices[2], edge{2.0});
    graph.add_edge(vertices[4], vertices[3], edge{3.0});
    graph.add_edge(vertices[4], vertices[6], edge{1.0});
    graph.add_edge(vertices[5], vertices[2], edge{3.0});
    graph.add_edge(vertices[5], vertices[6], edge{2.0});
    graph.add_edge(vertices[5], vertices[7], edge{3.0});
    graph.add_edge(vertices[5], vertices[8], edge{3.0});
    graph.add_edge(vertices[6], vertices[4], edge{1.0});
    graph.add_edge(vertices[6], vertices[5], edge{2.0});
    graph.add_edge(vertices[6], vertices[9], edge{2.0});
    graph.add_edge(vertices[7], vertices[0], edge{3.0});
    graph.add_edge(vertices[7], vertices[2], edge{1.0});
    graph.add_edge(vertices[7], vertices[5], edge{3.0});
    graph.add_edge(vertices[7], vertices[8], edge{2.0});
    graph.add_edge(vertices[8], vertices[5], edge{2.0});
    graph.add_edge(vertices[8], vertices[7], edge{2.0});
    graph.add_edge(vertices[8], vertices[9], edge{3.0});
    graph.add_edge(vertices[9], vertices[6], edge{2.0});
    graph.add_edge(vertices[9], vertices[8], edge{3.0});

    return std::pair{std::move(graph), std::move(vertices)};
}

template<typename InGraph, typename OutGraph>
std::vector<typename OutGraph::vertex_descriptor> convert_vertex_descriptors(
        const InGraph &in_graph,
        const std::vector<typename InGraph::vertex_descriptor> &in_vertices) {
    using in_vertex_descriptor = typename InGraph::vertex_descriptor;
    using out_vertex_descriptor = typename OutGraph::vertex_descriptor;

    std::vector<out_vertex_descriptor> out_vertices;
    out_vertices.reserve(in_vertices.size());
    for (const in_vertex_descriptor &in_vertex_desc : in_vertices) {
        out_vertices.emplace_back(in_graph.vertex_index(in_vertex_desc));
    }

    return out_vertices;
}

struct bfs_visitor : map_matching_2::graph::visitor {

    bfs_visitor(const std::vector<std::size_t> &indices_order)
        : indices_order{indices_order} {}

    template<typename Graph>
    void discover_vertex(const Graph &graph, typename Graph::vertex_descriptor vertex_desc) {
        BOOST_CHECK_EQUAL(indices_order[index], graph.vertex_index(vertex_desc));
        index++;
    }

    std::size_t index = 0;
    const std::vector<std::size_t> &indices_order;

};

struct dfs_visitor : map_matching_2::graph::visitor {

    dfs_visitor(const std::vector<std::size_t> &indices_order)
        : indices_order{indices_order} {}

    template<typename Graph>
    void examine_vertex(const Graph &graph, typename Graph::vertex_descriptor vertex_desc) {
        BOOST_CHECK_EQUAL(indices_order[index], graph.vertex_index(vertex_desc));
        index++;
    }

    std::size_t index = 0;
    const std::vector<std::size_t> &indices_order;

};

BOOST_AUTO_TEST_SUITE(graph_algorithm_tests)

    BOOST_AUTO_TEST_CASE(adjacency_list_graph_bfs_test) {

        auto test_bfs = []<typename Graph>(const Graph &graph, const auto &vertices) {
            using vertex_descriptor = typename Graph::vertex_descriptor;

            std::vector<std::size_t> indices_order{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

            boost::unordered_flat_set<std::size_t> visited;
            map_matching_2::graph::queue_type<vertex_descriptor> queue;
            bfs_visitor visitor{indices_order};

            map_matching_2::graph::breadth_first_search(graph, vertices[0], visited, queue, visitor);
        };

        auto [graph, vertices] = create_tree();
        test_bfs(graph, vertices);

        compressed_sparse_row_type<void> csr;
        csr.from_adjacency_list(graph);
        auto csr_vertices = convert_vertex_descriptors<decltype(graph), decltype(csr)>(graph, vertices);
        test_bfs(csr, csr_vertices);
    }

    BOOST_AUTO_TEST_CASE(adjacency_list_graph_dfs_test) {

        auto test_dfs = []<typename Graph>(const Graph &graph, const auto &vertices) {
            using vertex_descriptor = typename Graph::vertex_descriptor;

            std::vector<std::size_t> indices_order{0, 1, 4, 5, 8, 9, 2, 6, 3, 7};

            boost::unordered_flat_set<std::size_t> visited;
            map_matching_2::graph::stack_type<vertex_descriptor> stack;
            dfs_visitor visitor{indices_order};

            std::size_t index = 0;
            map_matching_2::graph::depth_first_search(graph, vertices[0], visited, stack, visitor);
        };

        auto [graph, vertices] = create_tree();
        test_dfs(graph, vertices);

        compressed_sparse_row_type<void> csr;
        csr.from_adjacency_list(graph);
        auto csr_vertices = convert_vertex_descriptors<decltype(graph), decltype(csr)>(graph, vertices);
        test_dfs(csr, csr_vertices);

    }

    BOOST_AUTO_TEST_CASE(adjacency_list_graph_dijkstra_test) {

        auto test_dijkstra = []<typename Graph>(const Graph &graph, const auto &vertices) {
            using vertex_descriptor = typename Graph::vertex_descriptor;

            std::vector<std::size_t> shortest_path_0_9{0, 7, 8, 9};
            std::vector<std::size_t> shortest_path_3_8{3, 4, 2, 7, 8};

            //        auto vertices_v = graph.vertices_view();
            //        std::vector<vertex_descriptor> index_map(vertices_v.size());
            //        for (auto it = vertices_v.begin(); it != vertices_v.end(); it = vertices_v.next(it)) {
            //            index_map[graph.vertex_index(it)] = it;
            //        }

            using weight_type = decltype(edge::weight);
            constexpr auto weight_func = [](const edge &data) constexpr {
                return data.weight;
            };

            using priority_type = map_matching_2::graph::priority_type<weight_type, vertex_descriptor>;
            map_matching_2::graph::predecessors_type predecessors;
            map_matching_2::graph::distances_type<weight_type> distances;
            map_matching_2::graph::colors_type colors;
            map_matching_2::graph::priority_queue_type<priority_type> queue;
            map_matching_2::graph::visitor visitor;

            map_matching_2::graph::dijkstra_shortest_path(graph, vertices[0],
                    predecessors, distances, colors, queue, visitor, weight_func);

            for (std::size_t i = shortest_path_0_9.size() - 1; i > 0; --i) {
                BOOST_CHECK_EQUAL(predecessors[shortest_path_0_9[i]], shortest_path_0_9[i - 1]);
            }

            for (std::size_t i = 0; i < vertices.size(); ++i) {
                std::cout << std::format("Predecessor {}: {}; ", i, predecessors[i])
                        << std::format("Distance to {}: {}", i, distances[i]) << std::endl;
            }

            predecessors.clear();
            distances.clear();
            colors.clear();

            map_matching_2::graph::dijkstra_shortest_path(
                    graph, vertices[3], predecessors, distances, colors, queue, visitor, [](const edge &data) {
                        return data.weight;
                    });

            for (std::size_t i = shortest_path_3_8.size() - 1; i > 0; --i) {
                BOOST_CHECK_EQUAL(predecessors[shortest_path_3_8[i]], shortest_path_3_8[i - 1]);
            }

            for (std::size_t i = 0; i < vertices.size(); ++i) {
                std::cout << std::format("Predecessor {}: {}; ", i, predecessors[i])
                        << std::format("Distance to {}: {}", i, distances[i]) << std::endl;
            }

            predecessors.clear();
            distances.clear();
            colors.clear();
        };

        auto [graph, vertices] = create_weighted_graph();
        test_dijkstra(graph, vertices);

        compressed_sparse_row_type<edge> csr;
        csr.from_adjacency_list(graph);
        auto csr_vertices = convert_vertex_descriptors<decltype(graph), decltype(csr)>(graph, vertices);
        test_dijkstra(csr, csr_vertices);

    }

BOOST_AUTO_TEST_SUITE_END()
