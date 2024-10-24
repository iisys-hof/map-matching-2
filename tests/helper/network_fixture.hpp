// Copyright (C) 2021-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TESTS_HELPER_NETWORK_FIXTURE_HPP
#define MAP_MATCHING_2_TESTS_HELPER_NETWORK_FIXTURE_HPP

#include <vector>
#include <initializer_list>

#include <boost/unordered/unordered_flat_map.hpp>

#include "types/geometry/network/network.hpp"
#include "types/geometry/network/network_import.hpp"

#include "geometry/network/traits/network.hpp"

#include "io/allocator/counting_allocator.hpp"

#include "geometry_helper.hpp"

using network_import_geographic = MM2_NETWORK_IMPORT(
        MM2_GRAPH_HELPER_IMPORT(MM2_GEOGRAPHIC, MM2_MEMORY_TYPES, MM2_TAG_HELPER(MM2_MEMORY_TYPES)));
using network_import_metric = MM2_NETWORK_IMPORT(
        MM2_GRAPH_HELPER_IMPORT(MM2_CARTESIAN, MM2_MEMORY_TYPES, MM2_TAG_HELPER(MM2_MEMORY_TYPES)));

using network_geographic = MM2_NETWORK(
        MM2_GRAPH_HELPER(MM2_GEOGRAPHIC, MM2_MEMORY_TYPES, MM2_TAG_HELPER(MM2_MEMORY_TYPES)),
        MM2_INDEX_HELPER(MM2_GEOGRAPHIC, MM2_MEMORY_TYPES, MM2_MEMORY_TYPES));
using network_metric = MM2_NETWORK(
        MM2_GRAPH_HELPER(MM2_CARTESIAN, MM2_MEMORY_TYPES, MM2_TAG_HELPER(MM2_MEMORY_TYPES)),
        MM2_INDEX_HELPER(MM2_CARTESIAN, MM2_MEMORY_TYPES, MM2_MEMORY_TYPES));

template<typename Network>
struct network_fixture {

    using point_type = typename map_matching_2::geometry::network::network_traits<Network>::point_type;
    using line_type = typename map_matching_2::geometry::network::network_traits<Network>::line_type;
    using rich_line_type = typename map_matching_2::geometry::network::network_traits<Network>::rich_line_type;
    using node_type = typename map_matching_2::geometry::network::network_traits<Network>::vertex_data_type;
    using edge_type = typename map_matching_2::geometry::network::network_traits<Network>::edge_data_type;
    using vertex_descriptor = typename map_matching_2::geometry::network::network_traits<Network>::vertex_descriptor;
    using edge_descriptor = typename map_matching_2::geometry::network::network_traits<Network>::edge_descriptor;

    map_matching_2::io::allocator::counting_allocator<std::byte,
        map_matching_2::io::allocator::allocation_counter> allocator;

    boost::unordered::unordered_flat_map<std::uint64_t, vertex_descriptor> nodes_map;
    boost::unordered::unordered_flat_map<std::uint64_t, edge_descriptor> edges_map;

    map_matching_2::io::helper::tag_helper_type<> tag_helper;
    using tag_vector_type = map_matching_2::io::helper::tag_helper_type<>::tag_vector_type;

    node_type create_node(std::uint64_t id, point_type point,
            std::initializer_list<std::pair<std::string, std::string>> tags = {}) {
        tag_vector_type tag_ids(allocator);
        tag_ids.reserve(tags.size());
        for (const auto &tag_pair : tags) {
            tag_ids.emplace_back(tag_helper.id(tag_pair.first, tag_pair.second));
        }
        tag_helper.set_node_tags(id, std::move(tag_ids));

        return node_type{id, std::move(point)};
    }

    edge_type create_edge(std::uint64_t id, const line_type &line,
            std::initializer_list<std::pair<std::string, std::string>> tags = {}) {
        tag_vector_type tag_ids(allocator);
        tag_ids.reserve(tags.size());
        for (const auto &tag_pair : tags) {
            tag_ids.emplace_back(tag_helper.id(tag_pair.first, tag_pair.second));
        }
        tag_helper.set_edge_tags(id, std::move(tag_ids));

        return edge_type{id, rich_line_type{line}};
    }

    vertex_descriptor add_node(Network &network, const std::uint64_t node_id, node_type node) {
        auto vertex_index = network.graph_helper().add_vertex_with_index_mapping(node);
        auto vertex = network.graph_helper().get_vertex_descriptor_for_index(vertex_index);
        nodes_map.emplace(node_id, vertex);
        return vertex;
    }

    vertex_descriptor add_vertex(Network &network, const std::uint64_t node_id, const point_type point) {
        return add_node(network, node_id, create_node(node_id, point));
    }


    edge_descriptor add_edge(Network &network, const std::uint64_t edge_id,
            const std::initializer_list<const std::pair<std::uint64_t, std::uint64_t>> node_pairs) {
        line_type line;
        line.reserve(node_pairs.size() * 2 - 1);

        vertex_descriptor start, end;
        for (auto it = node_pairs.begin(); it != node_pairs.end(); ++it) {
            const auto &node_pair = *it;

            const auto vertex_a = nodes_map.at(node_pair.first);
            const auto vertex_b = nodes_map.at(node_pair.second);
            const auto &node_a = network.vertex(vertex_a);
            const auto &node_b = network.vertex(vertex_b);

            if (line.empty()) {
                line.emplace_back(node_a.point);
            }
            line.emplace_back(node_b.point);

            if (it == node_pairs.begin()) {
                start = vertex_a;
            }
            if (std::next(it) == node_pairs.end()) {
                end = vertex_b;
            }
        }

        edge_type edge{edge_id, rich_line_type{std::move(line)}};
        auto edge_descriptor = network.graph_helper().add_edge(
                network.graph().vertex_index(start),
                network.graph().vertex_index(end),
                std::move(edge));
        edges_map.emplace(edge_id, edge_descriptor);
        return edge_descriptor;
    }

    edge_descriptor add_edge(Network &network, const std::uint64_t edge_id,
            const std::pair<std::uint64_t, std::uint64_t> node_pair) {
        return add_edge(network, edge_id, {node_pair});
    }

    Network generate_simplification_network() {
        Network network;

        add_vertex(network, 1, {0, 0});

        add_vertex(network, 2, {-3, 0});
        add_vertex(network, 3, {-2, 0});
        add_vertex(network, 4, {-1, 0});
        add_vertex(network, 5, {1, 0});
        add_vertex(network, 6, {2, 0});
        add_vertex(network, 7, {3, 0});
        add_edge(network, 1, {2, 3});
        add_edge(network, 1, {3, 2});
        add_edge(network, 1, {3, 4});
        add_edge(network, 1, {4, 3});
        add_edge(network, 1, {4, 1});
        add_edge(network, 1, {1, 4});
        add_edge(network, 1, {1, 5});
        add_edge(network, 1, {5, 1});
        add_edge(network, 1, {5, 6});
        add_edge(network, 1, {6, 5});
        add_edge(network, 1, {6, 7});
        add_edge(network, 1, {7, 6});

        add_vertex(network, 8, {0, -2});
        add_vertex(network, 9, {0, -1});
        add_vertex(network, 10, {0, 1});
        add_vertex(network, 11, {0, 2});
        add_edge(network, 2, {8, 9});
        add_edge(network, 2, {9, 8});
        add_edge(network, 2, {9, 1});
        add_edge(network, 2, {1, 9});
        add_edge(network, 2, {1, 10});
        add_edge(network, 2, {10, 1});
        add_edge(network, 2, {10, 11});
        add_edge(network, 2, {11, 10});

        add_vertex(network, 12, {-2, -2});
        add_vertex(network, 13, {-2, -1});
        add_edge(network, 3, {12, 13});
        add_edge(network, 3, {13, 4});

        add_vertex(network, 14, {4, -1});
        add_vertex(network, 15, {5, 0});
        add_vertex(network, 16, {4, 1});
        add_vertex(network, 17, {6, 0});
        add_vertex(network, 18, {4, 2});
        add_edge(network, 4, {7, 14});
        add_edge(network, 4, {14, 15});
        add_edge(network, 4, {15, 16});
        add_edge(network, 4, {16, 7});
        add_edge(network, 5, {17, 15});
        add_edge(network, 6, {16, 18});

        return network;
    }

    Network generate_grid_network() {
        Network network;

        for (std::int32_t x = 0; x <= 9; ++x) {
            for (std::int32_t y = 0; y <= 9; ++y) {
                add_vertex(network, x * 10 + y, {(double) x, (double) y});
            }
        }

        std::size_t edge_id = 0;
        for (std::int32_t x = 0; x <= 9; ++x) {
            for (std::int32_t y = 0; y < 9; ++y) {
                add_edge(network, edge_id, {x * 10 + y, x * 10 + (y + 1)});
                add_edge(network, edge_id, {x * 10 + (y + 1), x * 10 + y});
            }
            edge_id++;
        }

        for (std::int32_t y = 0; y <= 9; ++y) {
            for (std::int32_t x = 0; x < 9; ++x) {
                add_edge(network, edge_id, {x * 10 + y, (x + 1) * 10 + y});
                add_edge(network, edge_id, {(x + 1) * 10 + y, x * 10 + y});
            }
            edge_id++;
        }

        return network;
    }

};

#endif //MAP_MATCHING_2_TESTS_HELPER_NETWORK_FIXTURE_HPP
