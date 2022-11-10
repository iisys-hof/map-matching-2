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

#ifndef MAP_MATCHING_2_ARC_NODE_IMPORTER_HPP
#define MAP_MATCHING_2_ARC_NODE_IMPORTER_HPP

#include <fstream>
#include <string>
#include <vector>

#include <absl/container/flat_hash_map.h>

#include <boost/algorithm/string.hpp>

#include <osmium/osm.hpp>

#include "../importer.hpp"

namespace map_matching_2::io::network {

    template<typename Network>
    class arc_node_importer : public importer {

    private:
        Network &_network;
        const std::string _nodes;

        std::vector<typename Network::node_type> _node_map;
        absl::flat_hash_map<std::size_t, typename Network::vertex_descriptor> _vertex_map;

    public:
        arc_node_importer(std::string arcs, std::string nodes, Network &network)
                : importer{std::move(arcs)}, _nodes{std::move(nodes)}, _network{network} {}

        void read() override {
            using node_type = typename Network::node_type;
            using edge_type = typename Network::edge_type;
            using point_type = typename node_type::point_type;

            std::ifstream nodes;
            nodes.open(this->nodes());

            std::string line;
            osmium::object_id_type id = 0;
            while (std::getline(nodes, line)) {
                std::vector<std::string> parts;
                boost::split(parts, line, boost::is_any_of("\t;,| "));
                point_type point{std::stod(parts[0]), std::stod(parts[1])};
                node_type node{id++, std::move(point)};
                const auto vertex_descriptor = _network.add_vertex(node);
                _node_map.emplace_back(std::move(node));
                _vertex_map.emplace(_node_map.size() - 1, vertex_descriptor);
            }

            std::ifstream arcs;
            arcs.open(this->arcs());

            id = 0;
            while (std::getline(arcs, line)) {
                std::vector<std::string> parts;
                boost::split(parts, line, boost::is_any_of("\t;,| "));
                const std::size_t index_a = std::stoul(parts[0]);
                const std::size_t index_b = std::stoul(parts[1]);
                const node_type &node_a = _node_map[index_a];
                const node_type &node_b = _node_map[index_b];
                const auto vertex_a = _vertex_map[index_a];
                const auto vertex_b = _vertex_map[index_b];
                edge_type edge{id++, {node_a.point, node_b.point}};
                _network.add_edge(vertex_a, vertex_b, edge);
            }
        }

        [[nodiscard]] const std::string &arcs() const {
            return this->filename();
        }

        [[nodiscard]] const std::string &nodes() const {
            return _nodes;
        }

    };

}

#endif //MAP_MATCHING_2_ARC_NODE_IMPORTER_HPP
