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

#ifndef MAP_MATCHING_2_SEATTLE_IMPORTER_HPP
#define MAP_MATCHING_2_SEATTLE_IMPORTER_HPP

#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include <absl/container/flat_hash_map.h>

#include <boost/algorithm/string.hpp>

#include <osmium/osm.hpp>

#include "../importer.hpp"

namespace map_matching_2::io::network {

    template<typename Network>
    class seattle_importer : public importer {

    private:
        bool _seattle_fix_connections;
        Network &_network;

        absl::flat_hash_map<std::pair<double, double>, typename Network::vertex_descriptor> _point_map;
        absl::flat_hash_map<osmium::object_id_type, typename Network::vertex_descriptor> _vertex_map;

    public:
        seattle_importer(std::string seattle, bool seattle_fix_connections, Network &network)
                : importer{std::move(seattle)}, _seattle_fix_connections{seattle_fix_connections}, _network{network} {}

        void read() override {
            using node_type = typename Network::node_type;
            using edge_type = typename Network::edge_type;
            using point_type = typename node_type::point_type;
            using line_type = typename edge_type::line_type;

            std::ifstream seattle;
            seattle.open(this->seattle());

            std::vector<std::string> parts;

            std::string line;
            // skip header
            std::getline(seattle, line);
            // read body
            while (std::getline(seattle, line)) {
                boost::split(parts, line, boost::is_any_of("\t"));

                osmium::object_id_type edge_id = std::stol(parts[0]);
                osmium::object_id_type from_node_id = std::stol(parts[1]);
                osmium::object_id_type to_node_id = std::stol(parts[2]);
                bool two_way = std::stoi(parts[3]) == 1;
                std::string wkt_line = parts[6];

                boost::trim(wkt_line);

                line_type edge_line;
                boost::geometry::read_wkt(wkt_line, edge_line);

                typename Network::vertex_descriptor from_vertex;
                if (not _vertex_map.contains(from_node_id)) {
                    const auto &from_point = edge_line.front();
                    const auto from_point_pair = std::pair{
                            boost::geometry::get<0>(from_point), boost::geometry::get<1>(from_point)};
                    if (not _seattle_fix_connections or not _point_map.contains(from_point_pair)) {
                        auto from_node = node_type{from_node_id, from_point};
                        from_vertex = _network.add_vertex(from_node);
                        _vertex_map.emplace(from_node_id, from_vertex);
                        if (_seattle_fix_connections) {
                            _point_map.emplace(from_point_pair, from_vertex);
                        }
                    } else {
                        from_vertex = _point_map.at(from_point_pair);
                    }
                } else {
                    from_vertex = _vertex_map.at(from_node_id);
                }

                typename Network::vertex_descriptor to_vertex;
                if (not _vertex_map.contains(to_node_id)) {
                    const auto &to_point = edge_line.back();
                    const auto to_point_pair = std::pair{
                            boost::geometry::get<0>(to_point), boost::geometry::get<1>(to_point)};
                    if (not _seattle_fix_connections or not _point_map.contains(to_point_pair)) {
                        auto to_node = node_type{to_node_id, to_point};
                        to_vertex = _network.add_vertex(to_node);
                        _vertex_map.emplace(to_node_id, to_vertex);
                        if (_seattle_fix_connections) {
                            _point_map.emplace(to_point_pair, to_vertex);
                        }
                    } else {
                        to_vertex = _point_map.at(to_point_pair);
                    }
                } else {
                    to_vertex = _vertex_map.at(to_node_id);
                }

                edge_type edge_a{edge_id, edge_line};
                _network.add_edge(from_vertex, to_vertex, edge_a);

                if (two_way) {
                    std::reverse(edge_line.begin(), edge_line.end());
                    edge_type edge_b{edge_id, edge_line};
                    _network.add_edge(to_vertex, from_vertex, edge_b);
                }

                parts.clear();
            }
        }

        [[nodiscard]] const std::string &seattle() const {
            return this->filename();
        }

    };

}

#endif //MAP_MATCHING_2_SEATTLE_IMPORTER_HPP
