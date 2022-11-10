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

#include <random>

#include <boost/test/unit_test.hpp>

#include <geometry/network/graph.hpp>
#include <io/memory_mapped/memory_mapped_file.hpp>

BOOST_AUTO_TEST_SUITE(graph_storage_tests)

    BOOST_AUTO_TEST_CASE(graph_storage_bundled_vector_test) {
        struct node_type {
            std::size_t index;
            std::string name;
        };

        struct edge_type {
            std::size_t index;
            std::string name;
        };

        using graph_type = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                node_type, edge_type>;
        using graph_memory_type = map_matching_2::geometry::network::graph_memory<graph_type>;

        graph_memory_type graph_storage;
        const auto a = graph_storage.add_vertex(node_type{0, "a"});
        const auto b = graph_storage.add_vertex(node_type{0, "b"});
        const auto c = graph_storage.add_vertex(node_type{0, "c"});
        const auto d = graph_storage.add_vertex(node_type{0, "d"});

        const auto a_b = graph_storage.add_edge(a, b, edge_type{0, "a-b"});
        const auto b_c = graph_storage.add_edge(b, c, edge_type{0, "b-c"});
        const auto a_d = graph_storage.add_edge(a, d, edge_type{0, "a-d"});

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 3);

        const auto &vertex_index_map = graph_storage.vertex_index_map();
        const auto &edge_index_map = graph_storage.edge_index_map();

        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, a), 0);
        BOOST_CHECK_EQUAL(graph_storage.vertex(a).index, 0);
        BOOST_CHECK_EQUAL(graph_storage.vertex(a).name, "a");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, b), 1);
        BOOST_CHECK_EQUAL(graph_storage.vertex(b).index, 1);
        BOOST_CHECK_EQUAL(graph_storage.vertex(b).name, "b");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, c), 2);
        BOOST_CHECK_EQUAL(graph_storage.vertex(c).index, 2);
        BOOST_CHECK_EQUAL(graph_storage.vertex(c).name, "c");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, d), 3);
        BOOST_CHECK_EQUAL(graph_storage.vertex(d).index, 3);
        BOOST_CHECK_EQUAL(graph_storage.vertex(d).name, "d");

        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_b.first), 0);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_b.first).index, 0);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_b.first).name, "a-b");
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, b_c.first), 1);
        BOOST_CHECK_EQUAL(graph_storage.edge(b_c.first).index, 1);
        BOOST_CHECK_EQUAL(graph_storage.edge(b_c.first).name, "b-c");
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_d.first), 2);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_d.first).index, 2);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_d.first).name, "a-d");

        graph_storage.clear_vertex(a);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 1);

        graph_storage.remove_vertex(a);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 3);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 1);
    }

    BOOST_AUTO_TEST_CASE(graph_storage_bundled_list_test) {
        struct node_type {
            std::size_t index;
            std::string name;
        };

        struct edge_type {
            std::size_t index;
            std::string name;
        };

        using graph_type = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
                node_type, edge_type>;
        using graph_memory_type = map_matching_2::geometry::network::graph_memory<graph_type>;

        graph_memory_type graph_storage;
        const auto a = graph_storage.add_vertex(node_type{0, "a"});
        const auto b = graph_storage.add_vertex(node_type{0, "b"});
        const auto c = graph_storage.add_vertex(node_type{0, "c"});
        const auto d = graph_storage.add_vertex(node_type{0, "d"});

        const auto a_b = graph_storage.add_edge(a, b, edge_type{0, "a-b"});
        const auto b_c = graph_storage.add_edge(b, c, edge_type{0, "b-c"});
        const auto a_d = graph_storage.add_edge(a, d, edge_type{0, "a-d"});

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 3);

        const auto &vertex_index_map = graph_storage.vertex_index_map();
        const auto &edge_index_map = graph_storage.edge_index_map();

        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, a), 0);
        BOOST_CHECK_EQUAL(graph_storage.vertex(a).index, 0);
        BOOST_CHECK_EQUAL(graph_storage.vertex(a).name, "a");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, b), 1);
        BOOST_CHECK_EQUAL(graph_storage.vertex(b).index, 1);
        BOOST_CHECK_EQUAL(graph_storage.vertex(b).name, "b");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, c), 2);
        BOOST_CHECK_EQUAL(graph_storage.vertex(c).index, 2);
        BOOST_CHECK_EQUAL(graph_storage.vertex(c).name, "c");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, d), 3);
        BOOST_CHECK_EQUAL(graph_storage.vertex(d).index, 3);
        BOOST_CHECK_EQUAL(graph_storage.vertex(d).name, "d");

        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_b.first), 0);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_b.first).index, 0);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_b.first).name, "a-b");
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, b_c.first), 1);
        BOOST_CHECK_EQUAL(graph_storage.edge(b_c.first).index, 1);
        BOOST_CHECK_EQUAL(graph_storage.edge(b_c.first).name, "b-c");
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_d.first), 2);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_d.first).index, 2);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_d.first).name, "a-d");

        graph_storage.clear_vertex(a);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 1);

        graph_storage.remove_vertex(a);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 3);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 1);
    }

    BOOST_AUTO_TEST_CASE(graph_storage_bundled_copy_test) {
        struct node_type {
            std::size_t index;
            std::string name;
        };

        struct edge_type {
            std::size_t index;
            std::string name;
        };

        using graph_type_a = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
                node_type, edge_type>;
        using graph_type_b = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                node_type, edge_type>;
        using graph_memory_type_a = map_matching_2::geometry::network::graph_memory<graph_type_a>;
        using graph_memory_type_b = map_matching_2::geometry::network::graph_memory<graph_type_b>;

        graph_memory_type_a graph_storage_a;
        const auto a = graph_storage_a.add_vertex(node_type{0, "a"});
        const auto b = graph_storage_a.add_vertex(node_type{0, "b"});
        const auto c = graph_storage_a.add_vertex(node_type{0, "c"});
        const auto d = graph_storage_a.add_vertex(node_type{0, "d"});

        const auto a_b = graph_storage_a.add_edge(a, b, edge_type{0, "a-b"});
        const auto b_c = graph_storage_a.add_edge(b, c, edge_type{0, "b-c"});
        const auto a_d = graph_storage_a.add_edge(a, d, edge_type{0, "a-d"});

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage_a.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage_a.graph()), 3);

        graph_memory_type_b graph_storage_b{graph_storage_a};

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage_b.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage_b.graph()), 3);

        const auto &vertex_index_map = graph_storage_b.vertex_index_map();
        const auto &edge_index_map = graph_storage_b.edge_index_map();

        const auto a2 = boost::vertex(0, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, a2), 0);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(a2).index, 0);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(a2).name, "a");
        const auto b2 = boost::vertex(1, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, b2), 1);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(b2).index, 1);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(b2).name, "b");
        const auto c2 = boost::vertex(2, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, c2), 2);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(c2).index, 2);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(c2).name, "c");
        const auto d2 = boost::vertex(3, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, d2), 3);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(d2).index, 3);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(d2).name, "d");

        const auto a_b2 = boost::edge(a2, b2, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_b2.first), 0);
        BOOST_CHECK_EQUAL(graph_storage_b.edge(a_b2.first).index, 0);
        BOOST_CHECK_EQUAL(graph_storage_b.edge(a_b2.first).name, "a-b");
        const auto b_c2 = boost::edge(b2, c2, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, b_c2.first), 1);
        BOOST_CHECK_EQUAL(graph_storage_b.edge(b_c2.first).index, 1);
        BOOST_CHECK_EQUAL(graph_storage_b.edge(b_c2.first).name, "b-c");
        const auto a_d2 = boost::edge(a2, d2, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_d2.first), 2);
        BOOST_CHECK_EQUAL(graph_storage_b.edge(a_d2.first).index, 2);
        BOOST_CHECK_EQUAL(graph_storage_b.edge(a_d2.first).name, "a-d");
    }

    BOOST_AUTO_TEST_CASE(graph_storage_external_vector_test) {
        struct node_type {
            std::string name;
        };

        struct edge_type {
            std::string name;
        };

        using graph_type = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                boost::no_property, boost::property<boost::edge_index_t, std::size_t>>;
        using graph_memory_type = map_matching_2::geometry::network::graph_memory_external<
                graph_type, node_type, edge_type>;

        std::vector<node_type> nodes;
        std::vector<edge_type> edges;

        graph_memory_type graph_storage{&nodes, &edges};
        const auto a = graph_storage.add_vertex(node_type{"a"});
        const auto b = graph_storage.add_vertex(node_type{"b"});
        const auto c = graph_storage.add_vertex(node_type{"c"});
        const auto d = graph_storage.add_vertex(node_type{"d"});

        const auto a_b = graph_storage.add_edge(a, b, edge_type{"a-b"});
        const auto b_c = graph_storage.add_edge(b, c, edge_type{"b-c"});
        const auto a_d = graph_storage.add_edge(a, d, edge_type{"a-d"});

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 3);

        const auto &vertex_index_map = graph_storage.vertex_index_map();
        const auto &edge_index_map = graph_storage.edge_index_map();

        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, a), 0);
        BOOST_CHECK_EQUAL(graph_storage.vertex(a).name, "a");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, b), 1);
        BOOST_CHECK_EQUAL(graph_storage.vertex(b).name, "b");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, c), 2);
        BOOST_CHECK_EQUAL(graph_storage.vertex(c).name, "c");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, d), 3);
        BOOST_CHECK_EQUAL(graph_storage.vertex(d).name, "d");

        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_b.first), 0);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_b.first).name, "a-b");
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, b_c.first), 1);
        BOOST_CHECK_EQUAL(graph_storage.edge(b_c.first).name, "b-c");
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_d.first), 2);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_d.first).name, "a-d");

        graph_storage.clear_vertex(a);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 1);

        graph_storage.remove_vertex(a);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 3);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 1);
    }

    BOOST_AUTO_TEST_CASE(graph_storage_external_list_test) {
        struct node_type {
            std::string name;
        };

        struct edge_type {
            std::string name;
        };

        using graph_type = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
                boost::property<boost::vertex_index_t, std::size_t>, boost::property<boost::edge_index_t, std::size_t>>;
        using graph_memory_type = map_matching_2::geometry::network::graph_memory_external<
                graph_type, node_type, edge_type>;

        std::vector<node_type> nodes;
        std::vector<edge_type> edges;

        graph_memory_type graph_storage{&nodes, &edges};
        const auto a = graph_storage.add_vertex(node_type{"a"});
        const auto b = graph_storage.add_vertex(node_type{"b"});
        const auto c = graph_storage.add_vertex(node_type{"c"});
        const auto d = graph_storage.add_vertex(node_type{"d"});

        const auto a_b = graph_storage.add_edge(a, b, edge_type{"a-b"});
        const auto b_c = graph_storage.add_edge(b, c, edge_type{"b-c"});
        const auto a_d = graph_storage.add_edge(a, d, edge_type{"a-d"});

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 3);

        const auto &vertex_index_map = graph_storage.vertex_index_map();
        const auto &edge_index_map = graph_storage.edge_index_map();

        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, a), 0);
        BOOST_CHECK_EQUAL(graph_storage.vertex(a).name, "a");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, b), 1);
        BOOST_CHECK_EQUAL(graph_storage.vertex(b).name, "b");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, c), 2);
        BOOST_CHECK_EQUAL(graph_storage.vertex(c).name, "c");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, d), 3);
        BOOST_CHECK_EQUAL(graph_storage.vertex(d).name, "d");

        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_b.first), 0);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_b.first).name, "a-b");
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, b_c.first), 1);
        BOOST_CHECK_EQUAL(graph_storage.edge(b_c.first).name, "b-c");
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_d.first), 2);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_d.first).name, "a-d");

        graph_storage.clear_vertex(a);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 1);

        graph_storage.remove_vertex(a);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 3);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 1);
    }

    BOOST_AUTO_TEST_CASE(graph_storage_external_copy_test) {
        struct node_type {
            std::string name;
        };

        struct edge_type {
            std::string name;
        };

        using graph_type_a = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
                boost::property<boost::vertex_index_t, std::size_t>, boost::property<boost::edge_index_t, std::size_t>>;
        using graph_type_b = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                boost::no_property, boost::property<boost::edge_index_t, std::size_t>>;
        using graph_memory_type_a = map_matching_2::geometry::network::graph_memory_external<
                graph_type_a, node_type, edge_type>;
        using graph_memory_type_b = map_matching_2::geometry::network::graph_memory_external<
                graph_type_b, node_type, edge_type>;

        std::vector<node_type> nodes_a;
        std::vector<edge_type> edges_a;

        graph_memory_type_a graph_storage_a{&nodes_a, &edges_a};
        const auto a = graph_storage_a.add_vertex(node_type{"a"});
        const auto b = graph_storage_a.add_vertex(node_type{"b"});
        const auto c = graph_storage_a.add_vertex(node_type{"c"});
        const auto d = graph_storage_a.add_vertex(node_type{"d"});

        const auto a_b = graph_storage_a.add_edge(a, b, edge_type{"a-b"});
        const auto b_c = graph_storage_a.add_edge(b, c, edge_type{"b-c"});
        const auto a_d = graph_storage_a.add_edge(a, d, edge_type{"a-d"});

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage_a.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage_a.graph()), 3);

        std::vector<node_type> nodes_b;
        std::vector<edge_type> edges_b;
        graph_memory_type_b graph_storage_b{&nodes_b, &edges_b};
        graph_storage_b = graph_storage_a;

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage_b.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage_b.graph()), 3);

        const auto &vertex_index_map = graph_storage_b.vertex_index_map();
        const auto &edge_index_map = graph_storage_b.edge_index_map();

        const auto a2 = boost::vertex(0, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, a2), 0);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(a2).name, "a");
        const auto b2 = boost::vertex(1, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, b2), 1);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(b2).name, "b");
        const auto c2 = boost::vertex(2, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, c2), 2);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(c2).name, "c");
        const auto d2 = boost::vertex(3, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, d2), 3);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(d2).name, "d");

        const auto a_b2 = boost::edge(a2, b2, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_b2.first), 0);
        BOOST_CHECK_EQUAL(graph_storage_b.edge(a_b2.first).name, "a-b");
        const auto b_c2 = boost::edge(b2, c2, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, b_c2.first), 1);
        BOOST_CHECK_EQUAL(graph_storage_b.edge(b_c2.first).name, "b-c");
        const auto a_d2 = boost::edge(a2, d2, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_d2.first), 2);
        BOOST_CHECK_EQUAL(graph_storage_b.edge(a_d2.first).name, "a-d");
    }

    BOOST_AUTO_TEST_CASE(graph_storage_memory_mapped_vector_test) {
        struct node_type {
            std::string name;
        };

        struct edge_type {
            std::string name;
        };

        using nodes_allocator_type = boost::interprocess::private_adaptive_pool<node_type,
                boost::interprocess::managed_mapped_file::segment_manager>;
        using edges_allocator_type = boost::interprocess::private_adaptive_pool<edge_type,
                boost::interprocess::managed_mapped_file::segment_manager>;

        using nodes_type = boost::interprocess::vector<node_type, nodes_allocator_type>;
        using edges_type = boost::interprocess::vector<edge_type, edges_allocator_type>;

        using graph_type = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                boost::no_property, boost::property<boost::edge_index_t, std::size_t>>;
        using vertex_index_map_type = typename boost::property_map<graph_type, boost::vertex_index_t>::type;
        using edge_index_map_type = typename boost::property_map<graph_type, boost::edge_index_t>::type;
        using graph_memory_type = map_matching_2::geometry::network::graph_memory_external<
                graph_type, node_type, edge_type, vertex_index_map_type, edge_index_map_type,
                nodes_allocator_type, edges_allocator_type, nodes_type, edges_type>;

        const char *nodes_file = "graph_storage_nodes.bin";
        const char *edges_file = "graph_storage_edges.bin";
        map_matching_2::io::memory_mapped::mmap_remover mmap_nodes_remover{nodes_file};
        map_matching_2::io::memory_mapped::mmap_remover mmap_edges_remover{edges_file};

        std::size_t nodes_file_size = 10 * 1024;
        std::size_t edges_file_size = 10 * 1024;
        auto *nodes_mmap_file = new boost::interprocess::managed_mapped_file{
                boost::interprocess::open_or_create, nodes_file, nodes_file_size};
        auto *nodes_allocator = new nodes_allocator_type{nodes_mmap_file->get_segment_manager()};
        auto *nodes = nodes_mmap_file->construct<nodes_type>(nodes_file)(*nodes_allocator);

        auto *edges_mmap_file = new boost::interprocess::managed_mapped_file{
                boost::interprocess::open_or_create, edges_file, edges_file_size};
        auto *edges_allocator = new edges_allocator_type{edges_mmap_file->get_segment_manager()};
        auto *edges = edges_mmap_file->construct<edges_type>(edges_file)(*edges_allocator);

        graph_memory_type graph_storage{nodes, edges};
        const auto a = graph_storage.add_vertex(node_type{"a"});
        const auto b = graph_storage.add_vertex(node_type{"b"});
        const auto c = graph_storage.add_vertex(node_type{"c"});
        const auto d = graph_storage.add_vertex(node_type{"d"});

        const auto a_b = graph_storage.add_edge(a, b, edge_type{"a-b"});
        const auto b_c = graph_storage.add_edge(b, c, edge_type{"b-c"});
        const auto a_d = graph_storage.add_edge(a, d, edge_type{"a-d"});

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 3);

        const auto &vertex_index_map = graph_storage.vertex_index_map();
        const auto &edge_index_map = graph_storage.edge_index_map();

        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, a), 0);
        BOOST_CHECK_EQUAL(graph_storage.vertex(a).name, "a");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, b), 1);
        BOOST_CHECK_EQUAL(graph_storage.vertex(b).name, "b");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, c), 2);
        BOOST_CHECK_EQUAL(graph_storage.vertex(c).name, "c");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, d), 3);
        BOOST_CHECK_EQUAL(graph_storage.vertex(d).name, "d");

        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_b.first), 0);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_b.first).name, "a-b");
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, b_c.first), 1);
        BOOST_CHECK_EQUAL(graph_storage.edge(b_c.first).name, "b-c");
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_d.first), 2);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_d.first).name, "a-d");

        graph_storage.clear_vertex(a);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 1);

        graph_storage.remove_vertex(a);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 3);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 1);

        delete edges_allocator;
        delete edges_mmap_file;
        delete nodes_allocator;
        delete nodes_mmap_file;
    }

    BOOST_AUTO_TEST_CASE(graph_storage_memory_mapped_list_test) {
        struct node_type {
            std::string name;
        };

        struct edge_type {
            std::string name;
        };

        using nodes_allocator_type = boost::interprocess::private_adaptive_pool<node_type,
                boost::interprocess::managed_mapped_file::segment_manager>;
        using edges_allocator_type = boost::interprocess::private_adaptive_pool<edge_type,
                boost::interprocess::managed_mapped_file::segment_manager>;

        using nodes_type = boost::interprocess::vector<node_type, nodes_allocator_type>;
        using edges_type = boost::interprocess::vector<edge_type, edges_allocator_type>;

        using graph_type = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
                boost::property<boost::vertex_index_t, std::size_t>, boost::property<boost::edge_index_t, std::size_t>>;
        using vertex_index_map_type = typename boost::property_map<graph_type, boost::vertex_index_t>::type;
        using edge_index_map_type = typename boost::property_map<graph_type, boost::edge_index_t>::type;
        using graph_memory_type = map_matching_2::geometry::network::graph_memory_external<
                graph_type, node_type, edge_type, vertex_index_map_type, edge_index_map_type,
                nodes_allocator_type, edges_allocator_type, nodes_type, edges_type>;

        const char *nodes_file = "graph_storage_nodes.bin";
        const char *edges_file = "graph_storage_edges.bin";
        map_matching_2::io::memory_mapped::mmap_remover mmap_nodes_remover{nodes_file};
        map_matching_2::io::memory_mapped::mmap_remover mmap_edges_remover{edges_file};

        std::size_t nodes_file_size = 10 * 1024;
        std::size_t edges_file_size = 10 * 1024;
        auto *nodes_mmap_file = new boost::interprocess::managed_mapped_file{
                boost::interprocess::open_or_create, nodes_file, nodes_file_size};
        auto *nodes_allocator = new nodes_allocator_type{nodes_mmap_file->get_segment_manager()};
        auto *nodes = nodes_mmap_file->construct<nodes_type>(nodes_file)(*nodes_allocator);

        auto *edges_mmap_file = new boost::interprocess::managed_mapped_file{
                boost::interprocess::open_or_create, edges_file, edges_file_size};
        auto *edges_allocator = new edges_allocator_type{edges_mmap_file->get_segment_manager()};
        auto *edges = edges_mmap_file->construct<edges_type>(edges_file)(*edges_allocator);

        graph_memory_type graph_storage{nodes, edges};
        const auto a = graph_storage.add_vertex(node_type{"a"});
        const auto b = graph_storage.add_vertex(node_type{"b"});
        const auto c = graph_storage.add_vertex(node_type{"c"});
        const auto d = graph_storage.add_vertex(node_type{"d"});

        const auto a_b = graph_storage.add_edge(a, b, edge_type{"a-b"});
        const auto b_c = graph_storage.add_edge(b, c, edge_type{"b-c"});
        const auto a_d = graph_storage.add_edge(a, d, edge_type{"a-d"});

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 3);

        const auto &vertex_index_map = graph_storage.vertex_index_map();
        const auto &edge_index_map = graph_storage.edge_index_map();

        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, a), 0);
        BOOST_CHECK_EQUAL(graph_storage.vertex(a).name, "a");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, b), 1);
        BOOST_CHECK_EQUAL(graph_storage.vertex(b).name, "b");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, c), 2);
        BOOST_CHECK_EQUAL(graph_storage.vertex(c).name, "c");
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, d), 3);
        BOOST_CHECK_EQUAL(graph_storage.vertex(d).name, "d");

        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_b.first), 0);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_b.first).name, "a-b");
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, b_c.first), 1);
        BOOST_CHECK_EQUAL(graph_storage.edge(b_c.first).name, "b-c");
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_d.first), 2);
        BOOST_CHECK_EQUAL(graph_storage.edge(a_d.first).name, "a-d");

        graph_storage.clear_vertex(a);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 1);

        graph_storage.remove_vertex(a);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), 3);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), 1);

        delete edges_allocator;
        delete edges_mmap_file;
        delete nodes_allocator;
        delete nodes_mmap_file;
    }

    BOOST_AUTO_TEST_CASE(graph_storage_memory_mapped_copy_test) {
        struct node_type {
            std::string name;
        };

        struct edge_type {
            std::string name;
        };

        using nodes_allocator_type = boost::interprocess::private_adaptive_pool<node_type,
                boost::interprocess::managed_mapped_file::segment_manager>;
        using edges_allocator_type = boost::interprocess::private_adaptive_pool<edge_type,
                boost::interprocess::managed_mapped_file::segment_manager>;

        using nodes_type = boost::interprocess::vector<node_type, nodes_allocator_type>;
        using edges_type = boost::interprocess::vector<edge_type, edges_allocator_type>;

        using graph_type_a = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
                boost::property<boost::vertex_index_t, std::size_t>, boost::property<boost::edge_index_t, std::size_t>>;
        using vertex_index_map_type_a = typename boost::property_map<graph_type_a, boost::vertex_index_t>::type;
        using edge_index_map_type_a = typename boost::property_map<graph_type_a, boost::edge_index_t>::type;
        using graph_memory_type_a = map_matching_2::geometry::network::graph_memory_external<
                graph_type_a, node_type, edge_type, vertex_index_map_type_a, edge_index_map_type_a,
                nodes_allocator_type, edges_allocator_type, nodes_type, edges_type>;

        const char *nodes_file_a = "graph_storage_nodes_a.bin";
        const char *edges_file_a = "graph_storage_edges_a.bin";
        map_matching_2::io::memory_mapped::mmap_remover mmap_nodes_remover_a{nodes_file_a};
        map_matching_2::io::memory_mapped::mmap_remover mmap_edges_remover_a{edges_file_a};

        std::size_t nodes_file_size = 10 * 1024;
        std::size_t edges_file_size = 10 * 1024;
        auto *nodes_mmap_file_a = new boost::interprocess::managed_mapped_file{
                boost::interprocess::open_or_create, nodes_file_a, nodes_file_size};
        auto *nodes_allocator_a = new nodes_allocator_type{nodes_mmap_file_a->get_segment_manager()};
        auto *nodes_a = nodes_mmap_file_a->construct<nodes_type>(nodes_file_a)(*nodes_allocator_a);

        auto *edges_mmap_file_a = new boost::interprocess::managed_mapped_file{
                boost::interprocess::open_or_create, edges_file_a, edges_file_size};
        auto *edges_allocator_a = new edges_allocator_type{edges_mmap_file_a->get_segment_manager()};
        auto *edges_a = edges_mmap_file_a->construct<edges_type>(edges_file_a)(*edges_allocator_a);

        graph_memory_type_a graph_storage_a{nodes_a, edges_a};
        const auto a = graph_storage_a.add_vertex(node_type{"a"});
        const auto b = graph_storage_a.add_vertex(node_type{"b"});
        const auto c = graph_storage_a.add_vertex(node_type{"c"});
        const auto d = graph_storage_a.add_vertex(node_type{"d"});

        const auto a_b = graph_storage_a.add_edge(a, b, edge_type{"a-b"});
        const auto b_c = graph_storage_a.add_edge(b, c, edge_type{"b-c"});
        const auto a_d = graph_storage_a.add_edge(a, d, edge_type{"a-d"});

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage_a.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage_a.graph()), 3);

        using graph_type_b = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                boost::no_property, boost::property<boost::edge_index_t, std::size_t>>;
        using vertex_index_map_type_b = typename boost::property_map<graph_type_b, boost::vertex_index_t>::type;
        using edge_index_map_type_b = typename boost::property_map<graph_type_b, boost::edge_index_t>::type;
        using graph_memory_type_b = map_matching_2::geometry::network::graph_memory_external<
                graph_type_b, node_type, edge_type, vertex_index_map_type_b, edge_index_map_type_b,
                nodes_allocator_type, edges_allocator_type, nodes_type, edges_type>;

        const char *nodes_file_b = "graph_storage_nodes_b.bin";
        const char *edges_file_b = "graph_storage_edges_b.bin";
        map_matching_2::io::memory_mapped::mmap_remover mmap_nodes_remover_b{nodes_file_b};
        map_matching_2::io::memory_mapped::mmap_remover mmap_edges_remover_b{edges_file_b};

        auto *nodes_mmap_file_b = new boost::interprocess::managed_mapped_file{
                boost::interprocess::open_or_create, nodes_file_b, nodes_file_size};
        auto *nodes_allocator_b = new nodes_allocator_type{nodes_mmap_file_b->get_segment_manager()};
        auto *nodes_b = nodes_mmap_file_b->construct<nodes_type>(nodes_file_b)(*nodes_allocator_b);

        auto *edges_mmap_file_b = new boost::interprocess::managed_mapped_file{
                boost::interprocess::open_or_create, edges_file_b, edges_file_size};
        auto *edges_allocator_b = new edges_allocator_type{edges_mmap_file_b->get_segment_manager()};
        auto *edges_b = edges_mmap_file_a->construct<edges_type>(edges_file_b)(*edges_allocator_b);

        graph_memory_type_b graph_storage_b{nodes_b, edges_b};
        graph_storage_b = graph_storage_a;

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage_b.graph()), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage_b.graph()), 3);

        const auto &vertex_index_map = graph_storage_b.vertex_index_map();
        const auto &edge_index_map = graph_storage_b.edge_index_map();

        const auto a2 = boost::vertex(0, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, a2), 0);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(a2).name, "a");
        const auto b2 = boost::vertex(1, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, b2), 1);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(b2).name, "b");
        const auto c2 = boost::vertex(2, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, c2), 2);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(c2).name, "c");
        const auto d2 = boost::vertex(3, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(vertex_index_map, d2), 3);
        BOOST_CHECK_EQUAL(graph_storage_b.vertex(d2).name, "d");

        const auto a_b2 = boost::edge(a2, b2, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_b2.first), 0);
        BOOST_CHECK_EQUAL(graph_storage_b.edge(a_b2.first).name, "a-b");
        const auto b_c2 = boost::edge(b2, c2, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, b_c2.first), 1);
        BOOST_CHECK_EQUAL(graph_storage_b.edge(b_c2.first).name, "b-c");
        const auto a_d2 = boost::edge(a2, d2, graph_storage_b.graph());
        BOOST_CHECK_EQUAL(boost::get(edge_index_map, a_d2.first), 2);
        BOOST_CHECK_EQUAL(graph_storage_b.edge(a_d2.first).name, "a-d");

        delete edges_allocator_a;
        delete edges_mmap_file_a;
        delete nodes_allocator_a;
        delete nodes_mmap_file_a;

        delete edges_allocator_b;
        delete edges_mmap_file_b;
        delete nodes_allocator_b;
        delete nodes_mmap_file_b;
    }

    BOOST_AUTO_TEST_CASE(graph_storage_memory_mapped_vector_resize_test) {
        struct node_type {
            std::string name;
        };

        struct edge_type {
            std::string name;
        };

        using nodes_allocator_type = boost::interprocess::private_adaptive_pool<node_type,
                boost::interprocess::managed_mapped_file::segment_manager>;
        using edges_allocator_type = boost::interprocess::private_adaptive_pool<edge_type,
                boost::interprocess::managed_mapped_file::segment_manager>;

        using nodes_type = boost::interprocess::vector<node_type, nodes_allocator_type>;
        using edges_type = boost::interprocess::vector<edge_type, edges_allocator_type>;

        using graph_type = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                boost::no_property, boost::property<boost::edge_index_t, std::size_t>>;
        using vertex_index_map_type = typename boost::property_map<graph_type, boost::vertex_index_t>::type;
        using edge_index_map_type = typename boost::property_map<graph_type, boost::edge_index_t>::type;
        using graph_memory_type = map_matching_2::geometry::network::graph_memory_external<
                graph_type, node_type, edge_type, vertex_index_map_type, edge_index_map_type,
                nodes_allocator_type, edges_allocator_type, nodes_type, edges_type>;

        const char *nodes_file = "graph_storage_nodes.bin";
        const char *edges_file = "graph_storage_edges.bin";
        map_matching_2::io::memory_mapped::mmap_start_remover mmap_nodes_remover{nodes_file};
        map_matching_2::io::memory_mapped::mmap_start_remover mmap_edges_remover{edges_file};

        std::size_t nodes_file_size = 10 * 1024;
        std::size_t edges_file_size = 10 * 1024;
        const char *nodes_identifier = "nodes";
        const char *edges_identifier = "edges";
        auto *nodes_mmap_file = new boost::interprocess::managed_mapped_file{
                boost::interprocess::open_or_create, nodes_file, nodes_file_size};
        auto *nodes_allocator = new nodes_allocator_type{nodes_mmap_file->get_segment_manager()};
        auto *nodes = nodes_mmap_file->construct<nodes_type>(nodes_identifier)(*nodes_allocator);

        auto *edges_mmap_file = new boost::interprocess::managed_mapped_file{
                boost::interprocess::open_or_create, edges_file, edges_file_size};
        auto *edges_allocator = new edges_allocator_type{edges_mmap_file->get_segment_manager()};
        auto *edges = edges_mmap_file->construct<edges_type>(edges_identifier)(*edges_allocator);

        std::size_t vertices_size = 1000;
        std::size_t edges_size = 1000;

        std::random_device random_device;
        std::default_random_engine random_engine{random_device()};
        std::uniform_int_distribution<std::size_t> random_index{0, vertices_size - 1};

        graph_memory_type graph_storage{nodes, edges};

        std::vector<typename graph_type::vertex_descriptor> vertex_descriptors;
        vertex_descriptors.reserve(vertices_size);
        for (std::size_t i = 0; i < vertices_size; ++i) {
            vertex_descriptors.emplace_back(graph_storage.add_vertex([&](nodes_type *&node_storage) -> std::size_t {
                std::size_t index = node_storage->size();
                map_matching_2::io::memory_mapped::emplace_back_auto_grow(
                        nodes_mmap_file, nodes_allocator, node_storage, nodes_file, nodes_file_size, nodes_identifier,
                        node_type{std::to_string(i)});
                return index;
            }));
        }

        std::vector<typename graph_type::edge_descriptor> edge_descriptors;
        edge_descriptors.reserve(edges_size);
        for (std::size_t i = 0; i < edges_size; ++i) {
            edge_descriptors.emplace_back(graph_storage.add_edge(
                    vertex_descriptors[random_index(random_engine)], vertex_descriptors[random_index(random_engine)],
                    [&](edges_type *&edge_storage) -> std::size_t {
                        std::size_t index = edge_storage->size();
                        map_matching_2::io::memory_mapped::emplace_back_auto_grow(
                                edges_mmap_file, edges_allocator, edge_storage, edges_file, edges_file_size,
                                edges_identifier, edge_type{std::to_string(i)});
                        return index;
                    }).first);
        }

        BOOST_CHECK_EQUAL(boost::num_vertices(graph_storage.graph()), vertices_size);
        BOOST_CHECK_EQUAL(boost::num_edges(graph_storage.graph()), edges_size);

        const auto &vertex_index_map = graph_storage.vertex_index_map();
        const auto &edge_index_map = graph_storage.edge_index_map();

        delete edges_allocator;
        delete edges_mmap_file;
        delete nodes_allocator;
        delete nodes_mmap_file;

        boost::interprocess::managed_mapped_file::shrink_to_fit(nodes_file);
        boost::interprocess::managed_mapped_file::shrink_to_fit(edges_file);
    }

BOOST_AUTO_TEST_SUITE_END()
