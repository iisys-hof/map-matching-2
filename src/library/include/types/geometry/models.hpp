// Copyright (C) 2023-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_MODELS_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_MODELS_HPP

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/types.hpp"
#include "types/geometry/point.hpp"

#include "io/memory_mapped/concepts.hpp"

#include "geometry/models.hpp"

#define MM2_LINESTRING(CS, TYPES) map_matching_2::geometry::linestring< \
    MM2_POINT(CS), MM2_VECTOR(TYPES), MM2_ALLOCATOR(TYPES)>

#define MM2_TIME_LINESTRING(CS, TYPES) map_matching_2::geometry::linestring< \
    MM2_TIME_POINT(CS), MM2_VECTOR(TYPES), MM2_ALLOCATOR(TYPES)>

#define MM2_IMPORT_LINESTRING(CS, TYPES) map_matching_2::geometry::linestring< \
    MM2_POINT(CS), MM2_VECTOR(TYPES), MM2_FAST_COUNTING_ALLOCATOR(TYPES)>

#define MM2_MULTI_LINESTRING(CS, TYPES) map_matching_2::geometry::multi_linestring< \
    map_matching_2::geometry::linestring<MM2_POINT(CS), MM2_VECTOR(TYPES), MM2_ALLOCATOR(TYPES)>, \
    MM2_VECTOR(TYPES), MM2_ALLOCATOR(TYPES)>

#define MM2_TIME_MULTI_LINESTRING(CS, TYPES) map_matching_2::geometry::multi_linestring< \
    map_matching_2::geometry::linestring<MM2_TIME_POINT(CS), MM2_VECTOR(TYPES), MM2_ALLOCATOR(TYPES)>, \
    MM2_VECTOR(TYPES), MM2_ALLOCATOR(TYPES)>

#define MM2_IMPORT_MULTI_LINESTRING(CS, TYPES) map_matching_2::geometry::multi_linestring< \
    map_matching_2::geometry::linestring<MM2_POINT(CS), MM2_VECTOR(TYPES), MM2_FAST_COUNTING_ALLOCATOR(TYPES)>, \
    MM2_VECTOR(TYPES), MM2_FAST_COUNTING_ALLOCATOR(TYPES)>

#ifdef EXPLICIT_TEMPLATES

#define MM2_LINESTRING_TEMPLATE(CS, TYPES) \
    MM2_EXTERN template class MM2_LINESTRING(CS, TYPES); \
    MM2_EXTERN template class MM2_TIME_LINESTRING(CS, TYPES); \
    MM2_EXTERN template class MM2_IMPORT_LINESTRING(CS, TYPES);

#define MM2_LINESTRING_TEMPLATE_CS(CS) \
    MM2_LINESTRING_TEMPLATE(CS, MM2_MEMORY_TYPES) \
    MM2_LINESTRING_TEMPLATE(CS, MM2_MMAP_TYPES)

MM2_LINESTRING_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_LINESTRING_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_LINESTRING_TEMPLATE_CS(MM2_CARTESIAN)

#define MM2_MULTI_LINESTRING_TEMPLATE(CS, TYPES) \
    MM2_EXTERN template class MM2_MULTI_LINESTRING(CS, TYPES); \
    MM2_EXTERN template class MM2_TIME_MULTI_LINESTRING(CS, TYPES); \
    MM2_EXTERN template class MM2_IMPORT_MULTI_LINESTRING(CS, TYPES);

#define MM2_MULTI_LINESTRING_TEMPLATE_CS(CS) \
    MM2_MULTI_LINESTRING_TEMPLATE(CS, MM2_MEMORY_TYPES) \
    MM2_MULTI_LINESTRING_TEMPLATE(CS, MM2_MMAP_TYPES)

MM2_MULTI_LINESTRING_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_MULTI_LINESTRING_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_MULTI_LINESTRING_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::geometry {

    template<is_point Point, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using linestring_type = linestring<Point, Types::template vector_type, Types::template allocator_type>;

    template<is_point Point, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using import_linestring_type = linestring<Point, Types::template vector_type,
        Types::template fast_counting_allocator_type>;

    template<is_line Line, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using multi_linestring_type = multi_linestring<Line, Types::template vector_type, Types::template allocator_type>;

    template<is_line Line, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using import_multi_linestring_type = multi_linestring<Line, Types::template vector_type,
        Types::template fast_counting_allocator_type>;

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_MODELS_HPP
