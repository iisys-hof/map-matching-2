// Copyright (C) 2020-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_NETWORK_TYPES_HPP
#define MAP_MATCHING_2_NETWORK_TYPES_HPP

#include "../types.hpp"
#include "network.hpp"

namespace map_matching_2::geometry::network {

    template<typename CoordinateSystem>
    struct types {
        using point_type = typename geometry::types<CoordinateSystem>::point_type;

        using internal_node_type = geometry::network::internal_node<point_type>;
        using internal_edge_type = geometry::network::internal_edge_type<point_type>;
        using eager_internal_edge_type = geometry::network::eager_internal_edge_type<point_type>;
        using lazy_internal_edge_type = geometry::network::lazy_internal_edge_type<point_type>;

        using external_node_type = geometry::network::node<point_type>;
        using external_edge_type = geometry::network::edge_type<point_type>;
        using eager_external_edge_type = geometry::network::eager_edge_type<point_type>;
        using lazy_external_edge_type = geometry::network::lazy_edge_type<point_type>;

        using internal_graph_import = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
                internal_node_type, internal_edge_type,
                boost::no_property, boost::listS>;
        using internal_graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                internal_node_type, eager_internal_edge_type,
                boost::no_property, boost::vecS>;

        using external_graph_import = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
                boost::property<boost::vertex_index_t, std::size_t>, boost::property<boost::edge_index_t, std::size_t>,
                boost::no_property, boost::listS>;
        using external_graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                boost::no_property, boost::property<boost::edge_index_t, std::size_t>,
                boost::no_property, boost::vecS>;

        using internal_graph_storage_import = graph_memory<internal_graph_import>;
        using internal_graph_storage = graph_memory<internal_graph>;

        using external_graph_storage_import = graph_memory_external<external_graph_import, external_node_type, external_edge_type>;
        using external_graph_storage = graph_memory_external<external_graph, external_node_type, eager_external_edge_type>;

        using internal_network_import = class network_import<internal_graph_storage_import>;
        using internal_network = class network<internal_graph_storage>;

        using external_network_import = class network_import<external_graph_storage_import>;
        using external_network = class network<external_graph_storage>;
    };

    using types_geographic = types<geometry::cs_geographic>;
    using types_spherical_equatorial = types<geometry::cs_spherical_equatorial>;
    using types_cartesian = types<geometry::cs_cartesian>;

}

#endif //MAP_MATCHING_2_NETWORK_TYPES_HPP
