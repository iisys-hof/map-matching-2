// Copyright (C) 2024-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TRAITS_NETWORK_HPP
#define MAP_MATCHING_2_TRAITS_NETWORK_HPP

#include "io/helper/graph_helper.hpp"

#include "geometry/rich_type/traits/rich_line.hpp"

#include "graph/traits/graph.hpp"

namespace map_matching_2::geometry::network {

    template<typename Network>
    concept is_network_base = requires {
        typename Network::graph_helper_type;
        typename Network::tag_helper_type;
    };

    template<typename Network>
    concept is_memory_network_graph = is_network_base<Network> and
            io::helper::is_memory_graph_helper<typename Network::graph_helper_type>;

    template<typename Network>
    concept is_mmap_network_graph = is_network_base<Network> and
            io::helper::is_mmap_graph_helper<typename Network::graph_helper_type>;

    template<typename Network>
    concept is_network_import = is_network_base<Network> and requires(Network &network) {
        { network.simplify() } -> std::same_as<void>;
    };

    template<typename Network>
    concept is_network = is_network_base<Network> and requires(Network &network) {
        typename Network::rtree_points_type;
        typename Network::rtree_segments_type;
        typename Network::index_helper_type;
        typename Network::index_build_helper_type;
        { network.build_spatial_indices() } -> std::same_as<void>;
    };

    template<is_network_base Network>
    struct network_traits {
        using network_type = Network;
        using graph_helper_type = typename network_type::graph_helper_type;
        using tag_helper_type = typename network_type::tag_helper_type;

        using graph_type = typename graph_helper_type::graph_type;
        using vertex_data_type = typename graph::graph_traits<graph_type>::vertex_data_type;
        using edge_data_type = typename graph::graph_traits<graph_type>::edge_data_type;
        using vertex_descriptor = typename graph::graph_traits<graph_type>::vertex_descriptor;
        using vertex_size_type = typename graph::graph_traits<graph_type>::vertex_size_type;
        using edge_descriptor = typename graph::graph_traits<graph_type>::edge_descriptor;
        using edge_size_type = typename graph::graph_traits<graph_type>::edge_size_type;

        using rich_line_type = typename edge_data_type::rich_line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using coordinate_system_type = typename rich_line_traits<rich_line_type>::coordinate_system_type;
        using coordinate_type = typename rich_line_traits<rich_line_type>::coordinate_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
    };

}

#endif //MAP_MATCHING_2_TRAITS_NETWORK_HPP
