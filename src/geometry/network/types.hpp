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
#include "node.hpp"
#include "edge.hpp"
#include "route.hpp"
#include "network.hpp"

namespace map_matching_2::geometry::network {

    template<typename CoordinateSystem>
    struct types {
        using node_type = node<typename geometry::types<CoordinateSystem>::point_type>;
        using edge_type = edge<typename geometry::types<CoordinateSystem>::line_type>;

        using graph_modifiable = boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
                node_type, edge_type, boost::no_property, boost::listS>;
        using graph_static = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                node_type, edge_type, boost::no_property, boost::vecS>;

        using network_modifiable = network<graph_modifiable>;
        using network_static = network<graph_static>;
    };

    using types_geographic = types<geometry::cs_geographic>;
    using types_cartesian = types<geometry::cs_cartesian>;

}

#endif //MAP_MATCHING_2_NETWORK_TYPES_HPP
