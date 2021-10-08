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

#include "arc_node_importer.hpp"

#include <fstream>
#include <string>

#include <boost/algorithm/string.hpp>

#include <osmium/osm.hpp>

#include <geometry/network/types.hpp>

namespace map_matching_2::io::network {

    template<typename Network>
    arc_node_importer<Network>::arc_node_importer(std::string arcs, std::string nodes, Network &network)
            : importer{std::move(arcs)}, _nodes{std::move(nodes)}, _network{network} {}

    template<typename Network>
    void arc_node_importer<Network>::read() {
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
            edge_type edge{id++, {node_a.id, node_b.id}, {node_a.point, node_b.point}};
            _network.add_edge(vertex_a, vertex_b, edge);
        }
    }

    template<typename Network>
    const std::string &arc_node_importer<Network>::arcs() const {
        return this->filename();
    }

    template<typename Network>
    const std::string &arc_node_importer<Network>::nodes() const {
        return _nodes;
    }

    template
    class arc_node_importer<geometry::network::types_geographic::network_modifiable>;

    template
    class arc_node_importer<geometry::network::types_cartesian::network_modifiable>;

}