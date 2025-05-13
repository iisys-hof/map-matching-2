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

#ifndef MAP_MATCHING_2_TYPES_GRAPH_COMPRESSED_SPARSE_ROW_HPP
#define MAP_MATCHING_2_TYPES_GRAPH_COMPRESSED_SPARSE_ROW_HPP

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/types.hpp"

#include "types/geometry/network/node.hpp"
#include "types/geometry/network/edge.hpp"

#include "graph/compressed_sparse_row.hpp"

#define MM2_EAGER_COMPRESSED_SPARSE_ROW(CS, TYPES) map_matching_2::graph::compressed_sparse_row< \
    MM2_VECTOR(TYPES), MM2_VECTOR(TYPES), map_matching_2::graph::directional, \
    MM2_NODE(CS), MM2_EDGE(MM2_EAGER_RICH_LINE, CS, TYPES), \
    typename MM2_ALLOCATOR(TYPES)<void>>

#ifdef EXPLICIT_TEMPLATES

#define MM2_EAGER_COMPRESSED_SPARSE_ROW_TEMPLATE(CS, TYPES) \
    MM2_EXTERN template class MM2_EAGER_COMPRESSED_SPARSE_ROW(CS, TYPES);

#define MM2_EAGER_COMPRESSED_SPARSE_ROW_TEMPLATE_CS(CS) \
    MM2_EAGER_COMPRESSED_SPARSE_ROW_TEMPLATE(CS, MM2_MEMORY_TYPES) \
    MM2_EAGER_COMPRESSED_SPARSE_ROW_TEMPLATE(CS, MM2_MMAP_TYPES)

MM2_EAGER_COMPRESSED_SPARSE_ROW_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_EAGER_COMPRESSED_SPARSE_ROW_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_EAGER_COMPRESSED_SPARSE_ROW_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::graph {

    template<geometry::is_point Point, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using network_compressed_sparse_row_type = compressed_sparse_row<
        Types::template vector_type,
        Types::template vector_type,
        directional,
        geometry::network::node_type<Point>,
        geometry::network::eager_edge_type<Point, Types>,
        typename Types::template allocator_type<void>>;

}

#endif //MAP_MATCHING_2_TYPES_GRAPH_COMPRESSED_SPARSE_ROW_HPP
