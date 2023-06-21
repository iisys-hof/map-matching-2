// Copyright (C) 2021-2023 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_MELBOURNE_IMPORTER_HPP
#define MAP_MATCHING_2_MELBOURNE_IMPORTER_HPP

#include <fstream>
#include <string>
#include <vector>

#include <absl/container/flat_hash_map.h>

#include <boost/algorithm/string.hpp>

#include <osmium/osm.hpp>

#include "../importer.hpp"

namespace map_matching_2::io::network {

    template<typename Network>
    class melbourne_importer : public importer {

    private:
        Network &_network;
        const std::string _vertex, _streets;

        absl::flat_hash_map<osmium::object_id_type, typename Network::vertex_descriptor> _vertex_map;

    public:
        melbourne_importer(std::string edges, std::string vertex, std::string streets, Network &network)
                : importer{std::move(edges)}, _vertex{std::move(vertex)}, _streets{std::move(streets)},
                  _network{network} {}

        void read() override {
            using node_type = typename Network::node_type;
            using edge_type = typename Network::edge_type;
            using point_type = typename node_type::point_type;

            std::ifstream vertices;
            vertices.open(this->vertex());

            std::vector<std::string> parts;

            std::string line;

            // skip header
            std::getline(vertices, line);
            // read body
            while (std::getline(vertices, line)) {
                boost::split(parts, line, boost::is_any_of(" "));

                osmium::object_id_type node_id = std::stol(parts[0]);
                point_type point{std::stod(parts[2]), std::stod(parts[3])};

                node_type node{node_id, std::move(point)};
                const auto vertex = _network.add_vertex(node);
                _vertex_map.emplace(node_id, vertex);

                parts.clear();
            }

            std::ifstream edges;
            edges.open(this->edges());

            // skip header
            std::getline(edges, line);
            std::getline(edges, line);
            // read body
            while (std::getline(edges, line)) {
                boost::split(parts, line, boost::is_any_of(" "));

                osmium::object_id_type edge_id = std::stol(parts[0]);
                osmium::object_id_type from_node_id = std::stol(parts[1]);
                osmium::object_id_type to_node_id = std::stol(parts[2]);

                const auto from_vertex = _vertex_map[from_node_id];
                const auto to_vertex = _vertex_map[to_node_id];
                const auto &from_node = _network.vertex(from_vertex);
                const auto &to_node = _network.vertex(to_vertex);

                edge_type edge{edge_id, {from_node.point, to_node.point}};
                _network.add_edge(from_vertex, to_vertex, edge);

                parts.clear();
            }

            std::ifstream streets;
            streets.open(this->streets());

            // can be skipped

        }

        [[nodiscard]] const std::string &edges() const {
            return this->filename();
        }

        [[nodiscard]] const std::string &vertex() const {
            return _vertex;
        }

        [[nodiscard]] const std::string &streets() const {
            return _streets;
        }

    };

}

#endif //MAP_MATCHING_2_MELBOURNE_IMPORTER_HPP
