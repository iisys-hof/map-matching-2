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

#ifndef MAP_MATCHING_2_TRAITS_RICH_LINE_HPP
#define MAP_MATCHING_2_TRAITS_RICH_LINE_HPP

#include "geometry/types.hpp"

#include "../concepts.hpp"

namespace map_matching_2::geometry {

    template<is_rich_line RichLine>
    struct rich_line_traits {
        using rich_line_type = RichLine;
        using point_type = typename rich_line_type::point_type;
        using line_type = typename rich_line_type::line_type;
        using rich_segment_type = typename rich_line_type::rich_segment_type;
        using rich_referring_segment_type = typename rich_line_type::rich_referring_segment_type;
        using segment_type = typename models<point_type>::segment_type;
        using referring_segment_type = typename models<point_type>::referring_segment_type;
        using coordinate_system_type = typename data<line_type>::coordinate_system_type;
        using memory_rich_line_type = typename rich_line_type::memory_rich_line_type;
        using memory_line_type = typename rich_line_type::memory_line_type;
        using coordinate_type = typename data<line_type>::coordinate_type;
        using distance_type = typename data<line_type>::distance_type;
        using length_type = typename data<line_type>::length_type;
        using angle_type = typename data<line_type>::angle_type;
    };

}

#endif //MAP_MATCHING_2_TRAITS_RICH_LINE_HPP
