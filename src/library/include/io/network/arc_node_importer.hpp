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

#ifndef MAP_MATCHING_2_IO_NETWORK_ARC_NODE_IMPORTER_HPP
#define MAP_MATCHING_2_IO_NETWORK_ARC_NODE_IMPORTER_HPP

#include <fstream>
#include <filesystem>
#include <string>
#include <format>
#include <vector>

#include <boost/unordered/unordered_flat_map.hpp>

#include <boost/algorithm/string.hpp>

#include <osmium/osm.hpp>

#include "types/geometry/rich_type/rich_line.hpp"
#include "types/geometry/reprojector.hpp"
#include "types/geometry/network/node.hpp"
#include "types/geometry/network/edge.hpp"

#include "geometry/network/traits/network.hpp"
#include "geometry/rich_type/traits/rich_line.hpp"

#include "../importer.hpp"

namespace map_matching_2::io::network {

    template<typename GraphHelper>
    class arc_node_importer : public importer {

    public:
        using graph_helper_type = GraphHelper;
        using point_type = typename graph_helper_type::vertex_data_type::point_type;
        using vertex_descriptor = typename graph_helper_type::vertex_descriptor;
        using vertex_size_type = typename graph_helper_type::vertex_size_type;
        using vertex_data_type = geometry::network::node_type<point_type>;
        using edge_data_type = geometry::network::edge_type<point_type>; // in-memory type for transportation
        using rich_line_type = typename edge_data_type::rich_line_type;
        using line_type = typename geometry::rich_line_traits<rich_line_type>::line_type;

        arc_node_importer(std::vector<std::string> filenames, graph_helper_type &graph_helper,
                geometry::point_reprojector_variant reprojector_variant)
            : importer{std::move(filenames)}, _graph_helper{graph_helper},
            _reprojector_variant{std::move(reprojector_variant)} {}

        void read() override {
            const auto &filenames = this->filenames();
            for (std::size_t i = 0; i + 1 < filenames.size(); i += 2) {
                // nodes
                if (not std::filesystem::exists(filenames[i])) {
                    throw std::runtime_error{std::format("file '{}' does not exist", filenames[i])};
                }
                std::ifstream nodes;
                nodes.open(filenames[i]);

                std::vector<std::string> parts;

                std::string line;
                std::uint64_t id = 0;
                while (std::getline(nodes, line)) {
                    boost::split(parts, line, boost::is_any_of("\t;,| "));
                    point_type point{std::stod(parts.at(0)), std::stod(parts.at(1))};
                    const vertex_size_type vertex_index = _graph_helper.add_vertex_with_index_mapping(
                            vertex_data_type{id++, std::move(point), _reprojector_variant});
                    _node_map.emplace_back(vertex_index);
                }

                // arcs
                if (not std::filesystem::exists(filenames[i + 1])) {
                    throw std::runtime_error{std::format("file '{}' does not exist", filenames[i + 1])};
                }
                std::ifstream arcs;
                arcs.open(filenames[i + 1]);

                id = 0;
                while (std::getline(arcs, line)) {
                    boost::split(parts, line, boost::is_any_of("\t;,| "));
                    const std::size_t index_a = std::stoul(parts.at(0));
                    const std::size_t index_b = std::stoul(parts.at(1));
                    const vertex_size_type &vertex_index_a = _node_map.at(index_a);
                    const vertex_size_type &vertex_index_b = _node_map.at(index_b);
                    const auto &node_a = _graph_helper.graph().get_vertex_data(
                            _graph_helper.get_vertex_descriptor_for_index(vertex_index_a));
                    const auto &node_b = _graph_helper.graph().get_vertex_data(
                            _graph_helper.get_vertex_descriptor_for_index(vertex_index_b));
                    _graph_helper.add_edge(vertex_index_a, vertex_index_b, edge_data_type{
                            id++, rich_line_type{line_type{node_a.point, node_b.point}}
                    });
                }

                _node_map.clear();
            }
        }

    private:
        graph_helper_type &_graph_helper;
        geometry::point_reprojector_variant _reprojector_variant;
        std::vector<vertex_size_type> _node_map{};

    };

}

#endif //MAP_MATCHING_2_IO_NETWORK_ARC_NODE_IMPORTER_HPP
