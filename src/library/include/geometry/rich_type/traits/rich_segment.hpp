// Copyright (C) 2024-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TRAITS_RICH_SEGMENT_HPP
#define MAP_MATCHING_2_TRAITS_RICH_SEGMENT_HPP

#include "geometry/point.hpp"

#include "../concepts.hpp"

namespace map_matching_2::geometry {

    template<is_rich_segment RichSegment>
    struct rich_segment_traits {
        using rich_segment_type = RichSegment;
        using segment_type = typename rich_segment_type::segment_type;
        using point_type = point_type_t<segment_type>;
        using coordinate_system_type = typename data<point_type>::coordinate_system_type;
        using coordinate_type = typename data<point_type>::coordinate_type;
        using distance_type = typename data<point_type>::distance_type;
        using length_type = typename data<point_type>::length_type;
        using angle_type = typename data<point_type>::angle_type;
    };

}

#endif //MAP_MATCHING_2_TRAITS_RICH_SEGMENT_HPP
