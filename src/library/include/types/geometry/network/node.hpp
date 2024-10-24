// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_NETWORK_NODE_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_NETWORK_NODE_HPP

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/types.hpp"

#include "geometry/network/node.hpp"

#define MM2_NODE(CS) map_matching_2::geometry::network::node<MM2_POINT(CS)>

#ifdef EXPLICIT_TEMPLATES

#define MM2_NODE_TEMPLATE_CS(CS) \
    MM2_EXTERN template class MM2_NODE(CS);

MM2_NODE_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_NODE_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_NODE_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::geometry::network {

    template<is_point Point>
    using node_type = node<Point>;

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_NETWORK_NODE_HPP
