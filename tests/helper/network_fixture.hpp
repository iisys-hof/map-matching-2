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

#ifndef MAP_MATCHING_2_NETWORK_FIXTURE_HPP
#define MAP_MATCHING_2_NETWORK_FIXTURE_HPP

#include <vector>
#include <initializer_list>

#include <absl/container/flat_hash_map.h>

#include <geometry/types.hpp>
#include <geometry/network/types.hpp>

#include "geometry_helper.hpp"

using network_import_geographic = map_matching_2::geometry::network::types_geographic::internal_network_import;
using network_import_metric = map_matching_2::geometry::network::types_cartesian::internal_network_import;

using network_geographic = map_matching_2::geometry::network::types_geographic::internal_network;
using network_metric = map_matching_2::geometry::network::types_cartesian::internal_network;

template<typename Network>
struct network_fixture {

    using point_type = typename Network::point_type;
    using line_type = typename Network::line_type;
    using node_type = typename Network::node_type;
    using edge_type = typename Network::edge_type;
    using vertex_descriptor = typename Network::vertex_descriptor;
    using edge_descriptor = typename Network::edge_descriptor;

    absl::flat_hash_map<osmium::object_id_type, vertex_descriptor> nodes_map;
    absl::flat_hash_map<osmium::object_id_type, edge_descriptor> edges_map;

    map_matching_2::geometry::network::tag_helper tag_helper;

    node_type create_node(osmium::object_id_type id, point_type point,
                          std::initializer_list<std::pair<std::string, std::string>> tags = {}) {
        std::vector<std::uint64_t> tag_ids;
        tag_ids.reserve(tags.size());
        for (const auto &tag_pair: tags) {
            tag_ids.emplace_back(tag_helper.id(tag_pair.first, tag_pair.second));
        }

        return node_type{id, std::move(point), std::move(tag_ids)};
    }

    edge_type create_edge(osmium::object_id_type id, const line_type &line,
                          std::initializer_list<std::pair<std::string, std::string>> tags = {}) {
        std::vector<std::uint64_t> tag_ids;
        tag_ids.reserve(tags.size());
        for (const auto &tag_pair: tags) {
            tag_ids.emplace_back(tag_helper.id(tag_pair.first, tag_pair.second));
        }

        return edge_type{id, line, std::move(tag_ids)};
    }

    vertex_descriptor add_vertex(Network &network, const osmium::object_id_type node_id, const point_type point) {
        auto node = create_node(node_id, point);
        auto vertex = network.add_vertex(node);
        nodes_map.emplace(node_id, vertex);
        return vertex;
    }

    edge_descriptor add_edge(Network &network, const osmium::object_id_type edge_id,
                             const std::initializer_list<const std::pair<osmium::object_id_type, osmium::object_id_type>> node_pairs) {
        using rich_line_type = typename edge_type::rich_line_type;
        using rich_segment_type = typename rich_line_type::rich_segment_type;
        using segment_type = typename rich_segment_type::segment_type;

        std::vector<rich_segment_type> rich_segments;
        rich_segments.reserve(node_pairs.size() * 2 - 1);

        vertex_descriptor start, end;
        for (auto it = node_pairs.begin(); it != node_pairs.end(); ++it) {
            const auto &node_pair = *it;

            const auto vertex_a = nodes_map.at(node_pair.first);
            const auto vertex_b = nodes_map.at(node_pair.second);
            const auto &node_a = network.vertex(vertex_a);
            const auto &node_b = network.vertex(vertex_b);

            rich_segments.emplace_back(rich_segment_type{segment_type{node_a.point, node_b.point}});

            if (it == node_pairs.begin()) {
                start = vertex_a;
            }
            if (std::next(it) == node_pairs.end()) {
                end = vertex_b;
            }
        }

        edge_type edge{edge_id, rich_line_type{std::move(rich_segments)}};
        auto edge_descriptor = network.add_edge(start, end, edge).first;
        edges_map.emplace(edge_id, edge_descriptor);
        return edge_descriptor;
    }

    edge_descriptor add_edge(Network &network, const osmium::object_id_type edge_id,
                             const std::pair<osmium::object_id_type, osmium::object_id_type> node_pair) {
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

#endif //MAP_MATCHING_2_NETWORK_FIXTURE_HPP
