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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_COMPARE_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_COMPARE_HPP

#include <list>

#include "types/extern_define.hpp"

#include "types/geometry/rich_type/rich_segment.hpp"
#include "types/geometry/rich_type/eager_rich_segment.hpp"
#include "types/geometry/rich_type/lazy_rich_segment.hpp"

#include "geometry/rich_type/traits/rich_segment.hpp"

#include "index.hpp"

#include "util/macros.hpp"

namespace map_matching_2::geometry {

    template<is_rich_segment RichSegment>
    using compare_rtree_segments_value_type = std::pair<typename rich_segment_traits<RichSegment>::segment_type,
        typename std::list<RichSegment>::iterator>;

}

#define MM2_COMPARE_RTREE_POINTS(CS) MM2_RTREE((MM2_POINT(CS)), MM2_MEMORY_TYPES)

#define MM2_COMPARE_RTREE_SEGMENTS(RICH_SEGMENT_TYPE) \
    MM2_RTREE((map_matching_2::geometry::compare_rtree_segments_value_type<UNPACK RICH_SEGMENT_TYPE>), MM2_MEMORY_TYPES)

#ifdef EXPLICIT_TEMPLATES

#define MM2_COMPARE_RTREE_POINTS_TEMPLATE(CS) \
    MM2_EXTERN template class MM2_COMPARE_RTREE_POINTS(CS);

MM2_COMPARE_RTREE_POINTS_TEMPLATE(MM2_GEOGRAPHIC)
MM2_COMPARE_RTREE_POINTS_TEMPLATE(MM2_SPHERICAL_EQUATORIAL)
MM2_COMPARE_RTREE_POINTS_TEMPLATE(MM2_CARTESIAN)

#define MM2_COMPARE_RTREE_SEGMENTS_TEMPLATE(RICH_SEGMENT_TYPE) \
    MM2_EXTERN template class MM2_COMPARE_RTREE_SEGMENTS(RICH_SEGMENT_TYPE);

#define MM2_COMPARE_RTREE_SEGMENTS_TEMPLATE_CS(CS) \
    MM2_COMPARE_RTREE_SEGMENTS_TEMPLATE((MM2_RICH_SEGMENT(CS, MM2_SEGMENT))) \
    MM2_COMPARE_RTREE_SEGMENTS_TEMPLATE((MM2_EAGER_RICH_SEGMENT(CS, MM2_SEGMENT))) \
    MM2_COMPARE_RTREE_SEGMENTS_TEMPLATE((MM2_LAZY_RICH_SEGMENT(CS, MM2_SEGMENT)))

MM2_COMPARE_RTREE_SEGMENTS_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_COMPARE_RTREE_SEGMENTS_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_COMPARE_RTREE_SEGMENTS_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::geometry {

    template<is_point Point>
    using compare_rtree_points_type = rtree_type<Point, io::memory_mapped::memory_types>;

    template<is_rich_segment RichSegment>
    using compare_rtree_segments_type = rtree_type<
        compare_rtree_segments_value_type<RichSegment>,
        io::memory_mapped::memory_types>;

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_COMPARE_HPP
