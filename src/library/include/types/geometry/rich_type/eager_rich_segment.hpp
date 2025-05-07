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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_RICH_TYPE_EAGER_RICH_SEGMENT_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_RICH_TYPE_EAGER_RICH_SEGMENT_HPP

#include "types/extern_define.hpp"

#include "rich_segment_data.hpp"
#include "rich_segment.hpp"

#include "geometry/rich_type/eager_rich_segment.hpp"

#define MM2_EAGER_RICH_SEGMENT(CS, SEGMENT_TYPE) map_matching_2::geometry::eager_rich_segment< \
    typename map_matching_2::geometry::models<MM2_POINT(CS)>::SEGMENT_TYPE>

#define MM2_EAGER_RICH_TIME_SEGMENT(CS, SEGMENT_TYPE) map_matching_2::geometry::eager_rich_segment< \
    typename map_matching_2::geometry::models<MM2_TIME_POINT(CS)>::SEGMENT_TYPE>

#ifdef EXPLICIT_TEMPLATES

#define MM2_EAGER_RICH_SEGMENT_TEMPLATE(CS, SEGMENT_TYPE) \
    MM2_EXTERN template class MM2_EAGER_RICH_SEGMENT(CS, SEGMENT_TYPE); \
    MM2_EXTERN template class MM2_EAGER_RICH_TIME_SEGMENT(CS, SEGMENT_TYPE);

#define MM2_EAGER_RICH_SEGMENT_TEMPLATE_CS(CS) \
    MM2_EAGER_RICH_SEGMENT_TEMPLATE(CS, MM2_SEGMENT) \
    MM2_EAGER_RICH_SEGMENT_TEMPLATE(CS, MM2_POINTING_SEGMENT) \
    MM2_EAGER_RICH_SEGMENT_TEMPLATE(CS, MM2_REFERRING_SEGMENT)

MM2_EAGER_RICH_SEGMENT_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_EAGER_RICH_SEGMENT_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_EAGER_RICH_SEGMENT_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::geometry {

    template<is_point Point>
    using eager_rich_segment_type = eager_rich_segment<typename models<Point>::segment_type>;

    template<is_point Point>
    using eager_rich_referring_segment_type = eager_rich_segment<typename models<Point>::referring_segment_type>;

    template<is_point Point>
    using eager_rich_pointing_segment_type = eager_rich_segment<typename models<Point>::pointing_segment_type>;

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_RICH_TYPE_EAGER_RICH_SEGMENT_HPP
