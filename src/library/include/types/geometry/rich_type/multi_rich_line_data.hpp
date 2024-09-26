// Copyright (C) 2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_RICH_TYPE_MULTI_RICH_LINE_DATA_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_RICH_TYPE_MULTI_RICH_LINE_DATA_HPP

#include "types/extern_define.hpp"

#include "types/io/memory_mapped/types.hpp"

#include "geometry/rich_type/multi_rich_line_data.hpp"

#define MM2_MULTI_RICH_LINE_DATA(LENGTH, ANGLE, TYPES) map_matching_2::geometry::multi_rich_line_data< \
    LENGTH, ANGLE, MM2_VECTOR(TYPES), MM2_ALLOCATOR(TYPES)>

#define MM2_IMPORT_MULTI_RICH_LINE_DATA(LENGTH, ANGLE, TYPES) map_matching_2::geometry::multi_rich_line_data< \
    LENGTH, ANGLE, MM2_VECTOR(TYPES), MM2_FAST_COUNTING_ALLOCATOR(TYPES)>

#ifdef EXPLICIT_TEMPLATES

#define MM2_MULTI_RICH_LINE_DATA_TEMPLATE(LENGTH, ANGLE, TYPES) \
    MM2_EXTERN template class MM2_MULTI_RICH_LINE_DATA(LENGTH, ANGLE, TYPES); \
    MM2_EXTERN template class MM2_IMPORT_MULTI_RICH_LINE_DATA(LENGTH, ANGLE, TYPES);

#define MM2_MULTI_RICH_LINE_DATA_TEMPLATE_LENGTH_ANGLE(LENGTH, ANGLE) \
    MM2_MULTI_RICH_LINE_DATA_TEMPLATE(LENGTH, ANGLE, MM2_MEMORY_TYPES) \
    MM2_MULTI_RICH_LINE_DATA_TEMPLATE(LENGTH, ANGLE, MM2_MMAP_TYPES)

#define MM2_MULTI_RICH_LINE_DATA_TEMPLATE_LENGTH(LENGTH) \
    MM2_MULTI_RICH_LINE_DATA_TEMPLATE_LENGTH_ANGLE(LENGTH, float) \
    MM2_MULTI_RICH_LINE_DATA_TEMPLATE_LENGTH_ANGLE(LENGTH, double) \
    MM2_MULTI_RICH_LINE_DATA_TEMPLATE_LENGTH_ANGLE(LENGTH, long double)

MM2_MULTI_RICH_LINE_DATA_TEMPLATE_LENGTH(float)
MM2_MULTI_RICH_LINE_DATA_TEMPLATE_LENGTH(double)
MM2_MULTI_RICH_LINE_DATA_TEMPLATE_LENGTH(long double)

#endif

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_RICH_TYPE_MULTI_RICH_LINE_DATA_HPP
