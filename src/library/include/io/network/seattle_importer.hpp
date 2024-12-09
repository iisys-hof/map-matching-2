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

#include "types/geometry/reprojector.hpp"

#include "geometry/network/traits/network.hpp"

#include "geometry/algorithm/srs_dispatch.hpp"

#include "../importer.hpp"

namespace map_matching_2::io::network {

    template<typename GraphHelper>
    class seattle_importer : public importer {

    public:
        using graph_helper_type = GraphHelper;

        using vertex_data_type = typename graph_helper_type::vertex_data_type;
        using vertex_size_type = typename graph_helper_type::vertex_size_type;
        using graph_point_type = typename vertex_data_type::point_type;
        using graph_cs_type = typename geometry::data<graph_point_type>::coordinate_system_type;

        seattle_importer(std::vector<std::string> filenames, graph_helper_type &graph_helper,
                const geometry::srs_transform &srs_transform,
                const geometry::point_reprojector_variant &reprojector_variant)
            : importer{std::move(filenames)}, _graph_helper{graph_helper},
            _srs_transform{srs_transform}, _reprojector_variant{reprojector_variant} {}

        void read() override {
            geometry::srs_dispatch(_srs_transform,
                    [this]<typename InputCS, typename OutputCS>() {
                        if constexpr (std::same_as<OutputCS, graph_cs_type>) {
                            using point_type_in = geometry::point_type<InputCS>;
                            using point_type_out = geometry::point_type<OutputCS>;

                            using node_type = geometry::network::node_type<point_type_out>;

                            using line_type_in = typename geometry::models<point_type_in>::template line_type<>;

                            using edge_type = geometry::network::import_edge_type<point_type_out>;
                            using rich_line_type = typename edge_type::rich_line_type;
                            using line_type = typename geometry::rich_line_traits<rich_line_type>::line_type;

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

                                    line_type_in edge_line_in;
                                    boost::geometry::read_wkt(wkt_line, edge_line_in);

                                    vertex_size_type from_vertex;
                                    if (not _vertex_map.contains(from_node_id)) {
                                        const point_type_in &from_point = edge_line_in.front();
                                        from_vertex = _graph_helper.add_vertex_with_index_mapping(
                                                node_type{
                                                        from_node_id, from_point, _reprojector_variant
                                                });
                                        _vertex_map.emplace(from_node_id, from_vertex);
                                    } else {
                                        from_vertex = _vertex_map.at(from_node_id);
                                    }

                                    vertex_size_type to_vertex;
                                    if (not _vertex_map.contains(to_node_id)) {
                                        const point_type_in &to_point = edge_line_in.back();
                                        to_vertex = _graph_helper.add_vertex_with_index_mapping(
                                                node_type{
                                                        to_node_id, to_point, _reprojector_variant
                                                });
                                        _vertex_map.emplace(to_node_id, to_vertex);
                                    } else {
                                        to_vertex = _vertex_map.at(to_node_id);
                                    }

                                    line_type edge_line;
                                    geometry::reproject_line(edge_line_in, edge_line, _reprojector_variant);

                                    _graph_helper.add_edge(
                                            from_vertex, to_vertex, edge_type{
                                                    edge_id, rich_line_type{edge_line}
                                            });

                                    if (two_way) {
                                        std::reverse(edge_line.begin(), edge_line.end());
                                        _graph_helper.add_edge(
                                                to_vertex, from_vertex, edge_type{
                                                        edge_id, rich_line_type{edge_line}
                                                });
                                    }
                                }
                            }
                        } else {
                            throw std::invalid_argument{
                                    "output coordinate system does not match graph coordinate system"
                            };
                        }
                    });
        }

    private:
        graph_helper_type &_graph_helper;
        const geometry::srs_transform &_srs_transform;
        const geometry::point_reprojector_variant &_reprojector_variant;

        boost::unordered_flat_map<vertex_size_type, vertex_size_type> _vertex_map{};

    };

}

#endif //MAP_MATCHING_2_IO_NETWORK_SEATTLE_IMPORTER_HPP
