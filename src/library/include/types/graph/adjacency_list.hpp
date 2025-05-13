// Copyright (C) 2020-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TYPES_GRAPH_ADJACENCY_LIST_HPP
#define MAP_MATCHING_2_TYPES_GRAPH_ADJACENCY_LIST_HPP

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/types.hpp"

#include "types/geometry/network/node.hpp"
#include "types/geometry/network/edge.hpp"

#include "graph/adjacency_list.hpp"

#define MM2_ADJACENCY_LIST(CS, TYPES) map_matching_2::graph::adjacency_list< \
    MM2_LIST(TYPES), MM2_LIST(TYPES), map_matching_2::graph::bidirectional, \
    MM2_NODE(CS), MM2_EDGE(MM2_IMPORT_RICH_LINE, CS, TYPES), \
    MM2_LIST(TYPES), typename MM2_FAST_COUNTING_ALLOCATOR(TYPES)<void>>

#ifdef EXPLICIT_TEMPLATES

#define MM2_ADJACENCY_LIST_TEMPLATE(CS, TYPES) \
    MM2_EXTERN template class MM2_ADJACENCY_LIST(CS, TYPES);

#define MM2_ADJACENCY_LIST_TEMPLATE_CS(CS) \
    MM2_ADJACENCY_LIST_TEMPLATE(CS, MM2_MEMORY_TYPES) \
    MM2_ADJACENCY_LIST_TEMPLATE(CS, MM2_MMAP_TYPES)

MM2_ADJACENCY_LIST_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_ADJACENCY_LIST_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_ADJACENCY_LIST_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::graph {

    template<geometry::is_point Point, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using network_adjacency_list_type = adjacency_list<
        Types::template list_type,
        Types::template list_type,
        bidirectional,
        geometry::network::node_type<Point>,
        geometry::network::import_edge_type<Point, Types>,
        Types::template list_type,
        typename Types::template fast_counting_allocator_type<void>>;

}

#endif //MAP_MATCHING_2_TYPES_GRAPH_ADJACENCY_LIST_HPP
