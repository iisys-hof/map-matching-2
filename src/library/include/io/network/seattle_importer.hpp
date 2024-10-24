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

#ifndef MAP_MATCHING_2_IO_NETWORK_SEATTLE_IMPORTER_HPP
#define MAP_MATCHING_2_IO_NETWORK_SEATTLE_IMPORTER_HPP

#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include <boost/unordered/unordered_flat_map.hpp>

#include <boost/algorithm/string.hpp>

#include <osmium/osm.hpp>

#include "geometry/network/traits/network.hpp"
#include "types/geometry/reprojector.hpp"

#include "../importer.hpp"

namespace map_matching_2::io::network {

    template<typename GraphHelper>
    class seattle_importer : public importer {

    public:
        using graph_helper_type = GraphHelper;
        using point_type = typename graph_helper_type::vertex_data_type::point_type;
        using coordinate_type = typename geometry::data<point_type>::coordinate_type;
        using vertex_descriptor = typename graph_helper_type::vertex_descriptor;
        using vertex_size_type = typename graph_helper_type::vertex_size_type;
        using vertex_data_type = geometry::network::node<point_type>;
        using rich_line_type = geometry::rich_line<point_type>; // use in-memory type for transportation
        using line_type = typename geometry::rich_line_traits<rich_line_type>::line_type;
        using edge_data_type = geometry::network::edge<rich_line_type>;

        seattle_importer(std::vector<std::string> filenames, graph_helper_type &graph_helper,
                geometry::point_reprojector_variant reprojector_variant)
            : importer{std::move(filenames)}, _graph_helper{graph_helper},
            _reprojector_variant{std::move(reprojector_variant)} {}

        void read() override {
            const auto &filenames = this->filenames();
            for (std::size_t i = 0; i < filenames.size(); ++i) {
                // road network
                if (not std::filesystem::exists(filenames[i])) {
                    throw std::runtime_error{std::format("file '{}' does not exist", filenames[i])};
                }
                std::ifstream seattle;
                seattle.open(filenames[i]);

                std::vector<std::string> parts;

                std::string line;
                // skip header
                std::getline(seattle, line);
                // read body
                while (std::getline(seattle, line)) {
                    boost::split(parts, line, boost::is_any_of("\t"));

                    vertex_size_type edge_id = std::stol(parts.at(0));
                    vertex_size_type from_node_id = std::stol(parts.at(1));
                    vertex_size_type to_node_id = std::stol(parts.at(2));
                    bool two_way = std::stoi(parts.at(3)) == 1;
                    std::string wkt_line = parts.at(6);

                    boost::trim(wkt_line);

                    line_type edge_line;
                    boost::geometry::read_wkt(wkt_line, edge_line);

                    vertex_size_type from_vertex;
                    if (not _vertex_map.contains(from_node_id)) {
                        const point_type &from_point = edge_line.front();
                        from_vertex = _graph_helper.add_vertex_with_index_mapping(
                                vertex_data_type{
                                        from_node_id, from_point, _reprojector_variant
                                });
                        _vertex_map.emplace(from_node_id, from_vertex);
                    } else {
                        from_vertex = _vertex_map.at(from_node_id);
                    }

                    vertex_size_type to_vertex;
                    if (not _vertex_map.contains(to_node_id)) {
                        const point_type &to_point = edge_line.back();
                        to_vertex = _graph_helper.add_vertex_with_index_mapping(
                                vertex_data_type{
                                        to_node_id, to_point, _reprojector_variant
                                });
                        _vertex_map.emplace(to_node_id, to_vertex);
                    } else {
                        to_vertex = _vertex_map.at(to_node_id);
                    }

                    _graph_helper.add_edge(
                            from_vertex, to_vertex, edge_data_type{
                                    edge_id, rich_line_type{edge_line}
                            });

                    if (two_way) {
                        std::reverse(edge_line.begin(), edge_line.end());
                        _graph_helper.add_edge(
                                to_vertex, from_vertex, edge_data_type{
                                        edge_id, rich_line_type{edge_line}
                                });
                    }
                }
            }
        }

    private:
        graph_helper_type &_graph_helper;
        geometry::point_reprojector_variant _reprojector_variant;
        boost::unordered_flat_map<vertex_size_type, vertex_size_type> _vertex_map{};

    };

}

#endif //MAP_MATCHING_2_IO_NETWORK_SEATTLE_IMPORTER_HPP
