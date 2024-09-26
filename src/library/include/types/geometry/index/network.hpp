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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_NETWORK_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_NETWORK_HPP

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL
#define BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL
#endif

#include <cstddef>
#include <utility>

#include "types/extern_define.hpp"

#include "types/geometry/point.hpp"

#include "geometry/types.hpp"

#include "index.hpp"

namespace map_matching_2::geometry {

    template<is_point Point>
    using rtree_points_value_type = std::pair<Point, std::size_t>;

    template<typename Segment>
    using rtree_segments_value_type = std::pair<Segment, std::pair<std::size_t, std::size_t>>;

}

#define MM2_RTREE_POINTS(CS, TYPES) \
    MM2_RTREE((map_matching_2::geometry::rtree_points_value_type<MM2_POINT(CS)>), TYPES)

#define MM2_RTREE_SEGMENTS(CS, TYPES) \
    MM2_RTREE((map_matching_2::geometry::rtree_segments_value_type< \
        typename map_matching_2::geometry::models<MM2_POINT(CS)>::segment_type>), TYPES)

#ifdef EXPLICIT_TEMPLATES

#define MM2_RTREE_POINTS_TEMPLATE(CS, TYPES) \
    MM2_EXTERN template class MM2_RTREE_POINTS(CS, TYPES);

#define MM2_RTREE_POINTS_TEMPLATE_CS(CS) \
    MM2_RTREE_POINTS_TEMPLATE(CS, MM2_MEMORY_TYPES) /* \
    // bug, see: https://github.com/boostorg/geometry/issues/1276
    MM2_RTREE_POINTS_TEMPLATE(CS, MM2_MMAP_TYPES) */

MM2_RTREE_POINTS_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_RTREE_POINTS_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_RTREE_POINTS_TEMPLATE_CS(MM2_CARTESIAN)

#define MM2_RTREE_SEGMENTS_TEMPLATE(CS, TYPES) \
    MM2_EXTERN template class MM2_RTREE_SEGMENTS(CS, TYPES);

#define MM2_RTREE_SEGMENTS_TEMPLATE_CS(CS) \
    MM2_RTREE_SEGMENTS_TEMPLATE(CS, MM2_MEMORY_TYPES) /* \
    // bug, see: https://github.com/boostorg/geometry/issues/1276
    MM2_RTREE_SEGMENTS_TEMPLATE(CS, MM2_MMAP_TYPES) */

MM2_RTREE_SEGMENTS_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_RTREE_SEGMENTS_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_RTREE_SEGMENTS_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::geometry {

    template<is_point Point, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using rtree_points_type = rtree_type<rtree_points_value_type<Point>, Types>;

    template<typename Segment, io::memory_mapped::is_types Types = io::memory_mapped::memory_types>
    using rtree_segments_type = rtree_type<rtree_segments_value_type<Segment>, Types>;

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_INDEX_NETWORK_HPP
