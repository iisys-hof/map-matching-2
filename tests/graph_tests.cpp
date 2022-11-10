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

#include <boost/graph/adjacency_list.hpp>

BOOST_AUTO_TEST_SUITE(graph_tests)

    BOOST_AUTO_TEST_CASE(adjacency_list_bundled_test) {
        struct vertex_type {
            std::size_t index;
            std::string name;
        };

        struct edge_type {
            std::size_t index;
            std::string name;
        };

        using graph_type = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
                vertex_type, edge_type, boost::no_property>;
        using graph_traits = boost::graph_traits<graph_type>;
        using vertex_descriptor = typename graph_traits::vertex_descriptor;
        using edge_descriptor = typename graph_traits::edge_descriptor;

        graph_type graph;

        vertex_type a{0, "A"};
        vertex_type b{1, "B"};
        vertex_type c{2, "C"};
        vertex_type d{3, "D"};

        vertex_descriptor _a = boost::add_vertex(a, graph);
        vertex_descriptor _b = boost::add_vertex(b, graph);
        vertex_descriptor _c = boost::add_vertex(c, graph);
        vertex_descriptor _d = boost::add_vertex(d, graph);

        edge_type a_b{0, "A-B"};
        edge_type a_c{1, "A-C"};
        edge_type c_d{2, "C-D"};

        edge_descriptor _a_b = boost::add_edge(_a, _b, a_b, graph).first;
        edge_descriptor _a_c = boost::add_edge(_a, _c, a_c, graph).first;
        edge_descriptor _c_d = boost::add_edge(_c, _d, c_d, graph).first;

        BOOST_CHECK_EQUAL(boost::num_vertices(graph), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph), 3);
        BOOST_CHECK_EQUAL(graph[_a].index, 0);
        BOOST_CHECK_EQUAL(graph[_a].name, "A");
        BOOST_CHECK_EQUAL(graph[_b].index, 1);
        BOOST_CHECK_EQUAL(graph[_b].name, "B");
        BOOST_CHECK_EQUAL(graph[_c].index, 2);
        BOOST_CHECK_EQUAL(graph[_c].name, "C");
        BOOST_CHECK_EQUAL(graph[_d].index, 3);
        BOOST_CHECK_EQUAL(graph[_d].name, "D");
        BOOST_CHECK_EQUAL(graph[_a_b].index, 0);
        BOOST_CHECK_EQUAL(graph[_a_b].name, "A-B");
        BOOST_CHECK_EQUAL(graph[_a_c].index, 1);
        BOOST_CHECK_EQUAL(graph[_a_c].name, "A-C");
        BOOST_CHECK_EQUAL(graph[_c_d].index, 2);
        BOOST_CHECK_EQUAL(graph[_c_d].name, "C-D");
    }

    BOOST_AUTO_TEST_CASE(adjacency_list_internal_test) {
        using graph_type = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
                boost::property<boost::vertex_index_t, std::size_t, boost::property<boost::vertex_name_t, std::string>>,
                boost::property<boost::edge_index_t, std::size_t, boost::property<boost::edge_name_t, std::string>>,
                boost::no_property>;
        using graph_traits = boost::graph_traits<graph_type>;
        using vertex_descriptor = typename graph_traits::vertex_descriptor;
        using edge_descriptor = typename graph_traits::edge_descriptor;

        graph_type graph;

        boost::property_map<graph_type, boost::vertex_index_t>::type vertex_index =
                boost::get(boost::vertex_index, graph);
        boost::property_map<graph_type, boost::vertex_name_t>::type vertex_name =
                boost::get(boost::vertex_name, graph);

        vertex_descriptor _a = boost::add_vertex(graph);
        vertex_descriptor _b = boost::add_vertex(graph);
        vertex_descriptor _c = boost::add_vertex(graph);
        vertex_descriptor _d = boost::add_vertex(graph);

        boost::put(vertex_index, _a, 0);
        boost::put(vertex_name, _a, "A");
        boost::put(vertex_index, _b, 1);
        boost::put(vertex_name, _b, "B");
        boost::put(vertex_index, _c, 2);
        boost::put(vertex_name, _c, "C");
        boost::put(vertex_index, _d, 3);
        boost::put(vertex_name, _d, "D");

        boost::property_map<graph_type, boost::edge_index_t>::type edge_index =
                boost::get(boost::edge_index, graph);
        boost::property_map<graph_type, boost::edge_name_t>::type edge_name =
                boost::get(boost::edge_name, graph);

        edge_descriptor _a_b = boost::add_edge(_a, _b, graph).first;
        edge_descriptor _a_c = boost::add_edge(_a, _c, graph).first;
        edge_descriptor _c_d = boost::add_edge(_c, _d, graph).first;

        boost::put(edge_index, _a_b, 0);
        boost::put(edge_name, _a_b, "A-B");
        boost::put(edge_index, _a_c, 1);
        boost::put(edge_name, _a_c, "A-C");
        boost::put(edge_index, _c_d, 2);
        boost::put(edge_name, _c_d, "C-D");

        BOOST_CHECK_EQUAL(boost::num_vertices(graph), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph), 3);
        BOOST_CHECK_EQUAL(boost::get(vertex_index, _a), 0);
        BOOST_CHECK_EQUAL(boost::get(vertex_name, _a), "A");
        BOOST_CHECK_EQUAL(boost::get(vertex_index, _b), 1);
        BOOST_CHECK_EQUAL(boost::get(vertex_name, _b), "B");
        BOOST_CHECK_EQUAL(boost::get(vertex_index, _c), 2);
        BOOST_CHECK_EQUAL(boost::get(vertex_name, _c), "C");
        BOOST_CHECK_EQUAL(boost::get(vertex_index, _d), 3);
        BOOST_CHECK_EQUAL(boost::get(vertex_name, _d), "D");
        BOOST_CHECK_EQUAL(boost::get(edge_index, _a_b), 0);
        BOOST_CHECK_EQUAL(boost::get(edge_name, _a_b), "A-B");
        BOOST_CHECK_EQUAL(boost::get(edge_index, _a_c), 1);
        BOOST_CHECK_EQUAL(boost::get(edge_name, _a_c), "A-C");
        BOOST_CHECK_EQUAL(boost::get(edge_index, _c_d), 2);
        BOOST_CHECK_EQUAL(boost::get(edge_name, _c_d), "C-D");
    }

    BOOST_AUTO_TEST_CASE(adjacency_list_external_test) {
        struct vertex_type {
            std::string name;
        };

        struct edge_type {
            std::string name;
        };

        using graph_type = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                boost::no_property, boost::property<boost::edge_index_t, std::size_t>, boost::no_property>;
        using graph_traits = boost::graph_traits<graph_type>;
        using vertex_descriptor = typename graph_traits::vertex_descriptor;
        using edge_descriptor = typename graph_traits::edge_descriptor;

        graph_type graph;

        std::vector<vertex_type> vertices{{"A"},
                                          {"B"},
                                          {"C"},
                                          {"E"}};

        using vertex_index_map_type = boost::property_map<graph_type, boost::vertex_index_t>::type;
        vertex_index_map_type vertex_index = boost::get(boost::vertex_index, graph);

        using vertex_map_type = boost::iterator_property_map<typename std::vector<vertex_type>::iterator, vertex_index_map_type>;
        vertex_map_type vertex_map = boost::make_iterator_property_map(vertices.begin(), vertex_index);

        vertex_descriptor _a = boost::add_vertex(graph);
        vertex_descriptor _b = boost::add_vertex(graph);
        vertex_descriptor _c = boost::add_vertex(graph);
        vertex_descriptor _d = boost::add_vertex(graph);

        BOOST_CHECK_EQUAL(boost::get(vertex_map, _d).name, "E");
        boost::put(vertex_map, _d, vertex_type{"F"});
        BOOST_CHECK_EQUAL(boost::get(vertex_map, _d).name, "F");
        boost::get(vertex_map, _d).name = "D";
        BOOST_CHECK_EQUAL(boost::get(vertex_map, _d).name, "D");

        std::vector<edge_type> edges{{"A-B"},
                                     {"A-C"},
                                     {"D-E"}};

        using edge_index_map_type = boost::property_map<graph_type, boost::edge_index_t>::type;
        edge_index_map_type edge_index = boost::get(boost::edge_index, graph);

        using edge_map_type = boost::iterator_property_map<typename std::vector<edge_type>::iterator, edge_index_map_type>;
        edge_map_type edge_map = boost::make_iterator_property_map(edges.begin(), edge_index);

        edge_descriptor _a_b = boost::add_edge(_a, _b, 0, graph).first;
        edge_descriptor _a_c = boost::add_edge(_a, _c, 1, graph).first;
        edge_descriptor _c_d = boost::add_edge(_c, _d, 2, graph).first;

        BOOST_CHECK_EQUAL(boost::get(edge_map, _c_d).name, "D-E");
        boost::put(edge_map, _c_d, edge_type{"E-F"});
        BOOST_CHECK_EQUAL(boost::get(edge_map, _c_d).name, "E-F");
        boost::get(edge_map, _c_d).name = "C-D";
        BOOST_CHECK_EQUAL(boost::get(edge_map, _c_d).name, "C-D");

        BOOST_CHECK_EQUAL(boost::num_vertices(graph), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph), 3);
        BOOST_CHECK_EQUAL(boost::get(vertex_index, _a), 0);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, _a).name, "A");
        BOOST_CHECK_EQUAL(boost::get(vertex_index, _b), 1);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, _b).name, "B");
        BOOST_CHECK_EQUAL(boost::get(vertex_index, _c), 2);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, _c).name, "C");
        BOOST_CHECK_EQUAL(boost::get(vertex_index, _d), 3);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, _d).name, "D");
        BOOST_CHECK_EQUAL(boost::get(edge_index, _a_b), 0);
        BOOST_CHECK_EQUAL(boost::get(edge_map, _a_b).name, "A-B");
        BOOST_CHECK_EQUAL(boost::get(edge_index, _a_c), 1);
        BOOST_CHECK_EQUAL(boost::get(edge_map, _a_c).name, "A-C");
        BOOST_CHECK_EQUAL(boost::get(edge_index, _c_d), 2);
        BOOST_CHECK_EQUAL(boost::get(edge_map, _c_d).name, "C-D");
    }

    BOOST_AUTO_TEST_CASE(adjacency_list_external_pointing_test) {
        using point_type = std::pair<double, double>;
        using segment_type = std::pair<point_type *, point_type *>;

        struct vertex_type {
            point_type *point;
        };

        struct edge_type {
            std::vector<segment_type> segments;
        };

        using graph_type = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                boost::no_property, boost::property<boost::edge_index_t, std::size_t>, boost::no_property>;
        using graph_traits = boost::graph_traits<graph_type>;
        using vertex_descriptor = typename graph_traits::vertex_descriptor;
        using edge_descriptor = typename graph_traits::edge_descriptor;

        graph_type graph;

        std::vector<point_type> points{{0, 0},
                                       {0, 1},
                                       {0, 2},
                                       {0, 3},
                                       {1, 1},
                                       {1, 2}};

        std::vector<vertex_type> vertices{{&points[0]},
                                          {&points[1]},
                                          {&points[3]},
                                          {&points[5]}};

        using vertex_index_map_type = boost::property_map<graph_type, boost::vertex_index_t>::type;
        vertex_index_map_type vertex_index = boost::get(boost::vertex_index, graph);

        using vertex_map_type = boost::iterator_property_map<typename std::vector<vertex_type>::iterator, vertex_index_map_type>;
        vertex_map_type vertex_map = boost::make_iterator_property_map(vertices.begin(), vertex_index);

        std::vector<vertex_descriptor> vertex_descriptors;
        vertex_descriptors.reserve(vertices.size());
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            vertex_descriptors.emplace_back(boost::add_vertex(graph));
        }

        std::vector<edge_type> edges{{{{&points[0], &points[1]}}},
                                     {{{&points[1], &points[0]}}},
                                     {{{&points[1], &points[2]}, {&points[2], &points[3]}}},
                                     {{{&points[3], &points[2]}, {&points[2], &points[1]}}},
                                     {{{&points[5], &points[4]}, {&points[4], &points[1]}}}};

        using edge_index_map_type = boost::property_map<graph_type, boost::edge_index_t>::type;
        edge_index_map_type edge_index = boost::get(boost::edge_index, graph);

        using edge_map_type = boost::iterator_property_map<typename std::vector<edge_type>::iterator, edge_index_map_type>;
        edge_map_type edge_map = boost::make_iterator_property_map(edges.begin(), edge_index);

        std::vector<edge_descriptor> edge_descriptors;
        edge_descriptors.reserve(edges.size());
        edge_descriptors.emplace_back(boost::add_edge(vertex_descriptors[0], vertex_descriptors[1], 0, graph).first);
        edge_descriptors.emplace_back(boost::add_edge(vertex_descriptors[1], vertex_descriptors[0], 1, graph).first);
        edge_descriptors.emplace_back(boost::add_edge(vertex_descriptors[1], vertex_descriptors[2], 2, graph).first);
        edge_descriptors.emplace_back(boost::add_edge(vertex_descriptors[2], vertex_descriptors[1], 3, graph).first);
        edge_descriptors.emplace_back(boost::add_edge(vertex_descriptors[3], vertex_descriptors[1], 4, graph).first);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph), 5);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[0]).point->first, points[0].first);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[0]).point->second, points[0].second);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[0]).point, &points[0]);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[1]).point->first, points[1].first);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[1]).point->second, points[1].second);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[1]).point, &points[1]);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[2]).point->first, points[3].first);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[2]).point->second, points[3].second);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[2]).point, &points[3]);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[3]).point->first, points[5].first);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[3]).point->second, points[5].second);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[3]).point, &points[5]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[0]).segments[0].first->first, points[0].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[0]).segments[0].first->second, points[0].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[0]).segments[0].first, &points[0]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[0]).segments[0].second->first, points[1].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[0]).segments[0].second->second, points[1].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[0]).segments[0].second, &points[1]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[1]).segments[0].first->first, points[1].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[1]).segments[0].first->second, points[1].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[1]).segments[0].first, &points[1]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[1]).segments[0].second->first, points[0].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[1]).segments[0].second->second, points[0].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[1]).segments[0].second, &points[0]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[0].first->first, points[1].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[0].first->second, points[1].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[0].first, &points[1]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[0].second->first, points[2].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[0].second->second, points[2].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[0].second, &points[2]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[1].first->first, points[2].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[1].first->second, points[2].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[1].first, &points[2]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[1].second->first, points[3].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[1].second->second, points[3].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[1].second, &points[3]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[0].first->first, points[3].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[0].first->second, points[3].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[0].first, &points[3]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[0].second->first, points[2].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[0].second->second, points[2].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[0].second, &points[2]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[1].first->first, points[2].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[1].first->second, points[2].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[1].first, &points[2]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[1].second->first, points[1].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[1].second->second, points[1].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[1].second, &points[1]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[0].first->first, points[5].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[0].first->second, points[5].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[0].first, &points[5]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[0].second->first, points[4].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[0].second->second, points[4].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[0].second, &points[4]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[1].first->first, points[4].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[1].first->second, points[4].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[1].first, &points[4]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[1].second->first, points[1].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[1].second->second, points[1].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[1].second, &points[1]);
    }

    BOOST_AUTO_TEST_CASE(adjacency_list_completely_external_test) {
        using point_type = std::pair<double, double>;
        using segment_type = std::pair<point_type *, point_type *>;

        struct vertex_type {
            point_type *point;
        };

        struct edge_type {
            std::vector<segment_type> segments;
        };

        using graph_type = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS>;
        using graph_traits = boost::graph_traits<graph_type>;
        using vertex_descriptor = typename graph_traits::vertex_descriptor;
        using edge_descriptor = typename graph_traits::edge_descriptor;

        graph_type graph;

        std::vector<point_type> points{{0, 0},
                                       {0, 1},
                                       {0, 2},
                                       {0, 3},
                                       {1, 1},
                                       {1, 2}};

        std::vector<vertex_type> vertices{{&points[0]},
                                          {&points[1]},
                                          {&points[3]},
                                          {&points[5]}};

        using vertex_descriptor_map_type = std::map<vertex_descriptor, std::size_t>;
        vertex_descriptor_map_type vertex_index;

        using vertex_index_map_type = boost::associative_property_map<vertex_descriptor_map_type>;
        vertex_index_map_type vertex_index_map = boost::make_assoc_property_map(vertex_index);

        using vertex_map_type = boost::iterator_property_map<typename std::vector<vertex_type>::iterator, vertex_index_map_type>;
        vertex_map_type vertex_map = boost::make_iterator_property_map(vertices.begin(), vertex_index_map);

        std::vector<vertex_descriptor> vertex_descriptors;
        vertex_descriptors.reserve(vertices.size());
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            vertex_descriptors.emplace_back(boost::add_vertex(graph));
            boost::put(vertex_index_map, vertex_descriptors.back(), i);
        }

        std::vector<edge_type> edges{{{{&points[0], &points[1]}}},
                                     {{{&points[1], &points[0]}}},
                                     {{{&points[1], &points[2]}, {&points[2], &points[3]}}},
                                     {{{&points[3], &points[2]}, {&points[2], &points[1]}}},
                                     {{{&points[5], &points[4]}, {&points[4], &points[1]}}}};

        using edge_descriptor_map_type = std::map<edge_descriptor, std::size_t>;
        edge_descriptor_map_type edge_index;

        using edge_index_map_type = boost::associative_property_map<edge_descriptor_map_type>;
        edge_index_map_type edge_index_map = boost::make_assoc_property_map(edge_index);

        using edge_map_type = boost::iterator_property_map<typename std::vector<edge_type>::iterator, edge_index_map_type>;
        edge_map_type edge_map = boost::make_iterator_property_map(edges.begin(), edge_index_map);

        std::vector<edge_descriptor> edge_descriptors;
        edge_descriptors.reserve(edges.size());
        edge_descriptors.emplace_back(boost::add_edge(vertex_descriptors[0], vertex_descriptors[1], graph).first);
        boost::put(edge_index_map, edge_descriptors.back(), 0);
        edge_descriptors.emplace_back(boost::add_edge(vertex_descriptors[1], vertex_descriptors[0], graph).first);
        boost::put(edge_index_map, edge_descriptors.back(), 1);
        edge_descriptors.emplace_back(boost::add_edge(vertex_descriptors[1], vertex_descriptors[2], graph).first);
        boost::put(edge_index_map, edge_descriptors.back(), 2);
        edge_descriptors.emplace_back(boost::add_edge(vertex_descriptors[2], vertex_descriptors[1], graph).first);
        boost::put(edge_index_map, edge_descriptors.back(), 3);
        edge_descriptors.emplace_back(boost::add_edge(vertex_descriptors[3], vertex_descriptors[1], graph).first);
        boost::put(edge_index_map, edge_descriptors.back(), 4);

        BOOST_CHECK_EQUAL(boost::num_vertices(graph), 4);
        BOOST_CHECK_EQUAL(boost::num_edges(graph), 5);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[0]).point->first, points[0].first);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[0]).point->second, points[0].second);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[0]).point, &points[0]);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[1]).point->first, points[1].first);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[1]).point->second, points[1].second);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[1]).point, &points[1]);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[2]).point->first, points[3].first);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[2]).point->second, points[3].second);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[2]).point, &points[3]);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[3]).point->first, points[5].first);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[3]).point->second, points[5].second);
        BOOST_CHECK_EQUAL(boost::get(vertex_map, vertex_descriptors[3]).point, &points[5]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[0]).segments[0].first->first, points[0].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[0]).segments[0].first->second, points[0].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[0]).segments[0].first, &points[0]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[0]).segments[0].second->first, points[1].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[0]).segments[0].second->second, points[1].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[0]).segments[0].second, &points[1]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[1]).segments[0].first->first, points[1].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[1]).segments[0].first->second, points[1].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[1]).segments[0].first, &points[1]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[1]).segments[0].second->first, points[0].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[1]).segments[0].second->second, points[0].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[1]).segments[0].second, &points[0]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[0].first->first, points[1].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[0].first->second, points[1].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[0].first, &points[1]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[0].second->first, points[2].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[0].second->second, points[2].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[0].second, &points[2]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[1].first->first, points[2].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[1].first->second, points[2].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[1].first, &points[2]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[1].second->first, points[3].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[1].second->second, points[3].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[2]).segments[1].second, &points[3]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[0].first->first, points[3].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[0].first->second, points[3].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[0].first, &points[3]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[0].second->first, points[2].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[0].second->second, points[2].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[0].second, &points[2]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[1].first->first, points[2].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[1].first->second, points[2].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[1].first, &points[2]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[1].second->first, points[1].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[1].second->second, points[1].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[3]).segments[1].second, &points[1]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[0].first->first, points[5].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[0].first->second, points[5].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[0].first, &points[5]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[0].second->first, points[4].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[0].second->second, points[4].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[0].second, &points[4]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[1].first->first, points[4].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[1].first->second, points[4].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[1].first, &points[4]);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[1].second->first, points[1].first);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[1].second->second, points[1].second);
        BOOST_CHECK_EQUAL(boost::get(edge_map, edge_descriptors[4]).segments[1].second, &points[1]);
    }

BOOST_AUTO_TEST_SUITE_END()
