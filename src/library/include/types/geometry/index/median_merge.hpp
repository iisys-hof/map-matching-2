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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_MEDIAN_MERGE_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_MEDIAN_MERGE_HPP

#include <list>

#include "types/extern_define.hpp"

#include "types/geometry/point.hpp"

#include "index.hpp"

namespace map_matching_2::geometry {

    template<is_point Point>
    using median_merge_rtree_value_type = std::pair<Point, typename std::list<Point>::iterator>;

}

#define MM2_MEDIAN_MERGE_RTREE(CS) \
    MM2_RTREE((map_matching_2::geometry::median_merge_rtree_value_type<MM2_POINT(CS)>), MM2_MEMORY_TYPES)

#ifdef EXPLICIT_TEMPLATES

#define MM2_MEDIAN_MERGE_RTREE_TEMPLATE(CS) \
    MM2_EXTERN template class MM2_MEDIAN_MERGE_RTREE(CS);

MM2_MEDIAN_MERGE_RTREE_TEMPLATE(MM2_GEOGRAPHIC)
MM2_MEDIAN_MERGE_RTREE_TEMPLATE(MM2_SPHERICAL_EQUATORIAL)
MM2_MEDIAN_MERGE_RTREE_TEMPLATE(MM2_CARTESIAN)

#endif

namespace map_matching_2::geometry {

    template<is_point Point>
    using median_merge_rtree_type = rtree_type<median_merge_rtree_value_type<Point>, io::memory_mapped::memory_types>;

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_MEDIAN_MERGE_HPP
