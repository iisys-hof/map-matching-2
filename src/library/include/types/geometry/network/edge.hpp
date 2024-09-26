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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_NETWORK_EDGE_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_NETWORK_EDGE_HPP

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/types.hpp"

#include "types/geometry/rich_type/rich_line.hpp"
#include "types/geometry/rich_type/eager_rich_line.hpp"
#include "types/geometry/rich_type/lazy_rich_line.hpp"

#include "geometry/network/edge.hpp"

#define MM2_EDGE(RICH_LINE_TYPE, CS, TYPES) map_matching_2::geometry::network::edge< \
    RICH_LINE_TYPE(CS, TYPES), MM2_VECTOR(TYPES), MM2_ALLOCATOR(TYPES)>

#define MM2_IMPORT_EDGE(RICH_LINE_TYPE, CS, TYPES) map_matching_2::geometry::network::edge< \
    RICH_LINE_TYPE(CS, TYPES), MM2_VECTOR(TYPES), MM2_FAST_COUNTING_ALLOCATOR(TYPES)>

#ifdef EXPLICIT_TEMPLATES

#define MM2_EDGE_TEMPLATE(EDGE_TYPE, RICH_LINE_TYPE, CS, TYPES) \
    MM2_EXTERN template class EDGE_TYPE(RICH_LINE_TYPE, CS, TYPES);

#define MM2_EDGE_TEMPLATE_RICH_LINE_TYPE(CS, TYPES) \
    MM2_EDGE_TEMPLATE(MM2_EDGE, MM2_RICH_LINE, CS, TYPES) \
    MM2_EDGE_TEMPLATE(MM2_EDGE, MM2_EAGER_RICH_LINE, CS, TYPES) \
    MM2_EDGE_TEMPLATE(MM2_EDGE, MM2_LAZY_RICH_LINE, CS, TYPES) \
    MM2_EDGE_TEMPLATE(MM2_IMPORT_EDGE, MM2_IMPORT_RICH_LINE, CS, TYPES) \
    MM2_EDGE_TEMPLATE(MM2_IMPORT_EDGE, MM2_IMPORT_EAGER_RICH_LINE, CS, TYPES) \
    MM2_EDGE_TEMPLATE(MM2_IMPORT_EDGE, MM2_IMPORT_LAZY_RICH_LINE, CS, TYPES)

#define MM2_EDGE_TEMPLATE_RICH_LINE_TYPE_CS(CS) \
    MM2_EDGE_TEMPLATE_RICH_LINE_TYPE(CS, MM2_MEMORY_TYPES) \
    MM2_EDGE_TEMPLATE_RICH_LINE_TYPE(CS, MM2_MMAP_TYPES)

MM2_EDGE_TEMPLATE_RICH_LINE_TYPE_CS(MM2_GEOGRAPHIC)
MM2_EDGE_TEMPLATE_RICH_LINE_TYPE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_EDGE_TEMPLATE_RICH_LINE_TYPE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::geometry::network {

    template<is_point Point, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using edge_type = edge<rich_line_type<Point, Types>,
        Types::template vector_type, Types::template allocator_type>;

    template<is_point Point, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using import_edge_type = edge<import_rich_line_type<Point, Types>,
        Types::template vector_type, Types::template fast_counting_allocator_type>;

    template<is_point Point, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using eager_edge_type = edge<eager_rich_line_type<Point, Types>,
        Types::template vector_type, Types::template allocator_type>;

    template<is_point Point, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using import_eager_edge_type = edge<import_eager_rich_line_type<Point, Types>,
        Types::template vector_type, Types::template fast_counting_allocator_type>;

    template<is_point Point, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using lazy_edge_type = edge<lazy_rich_line_type<Point, Types>,
        Types::template vector_type, Types::template allocator_type>;

    template<is_point Point, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using import_lazy_edge_type = edge<import_lazy_rich_line_type<Point, Types>,
        Types::template vector_type, Types::template fast_counting_allocator_type>;

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_NETWORK_EDGE_HPP
