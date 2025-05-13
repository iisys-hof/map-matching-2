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

#ifndef MAP_MATCHING_2_GEOMETRY_CONCEPTS_HPP
#define MAP_MATCHING_2_GEOMETRY_CONCEPTS_HPP

#include <type_traits>

#include "types.hpp"

namespace map_matching_2::geometry {

    template<typename Segment>
    concept is_segment = std::is_same_v<Segment,
        typename models<point_type_t<Segment>>::segment_type>;

    template<typename Segment>
    concept is_referring_segment = std::is_same_v<Segment,
        typename models<point_type_t<Segment>>::referring_segment_type>;

    template<typename Segment>
    concept is_pointing_segment = std::is_same_v<Segment,
        typename models<point_type_t<Segment>>::pointing_segment_type>;

}

#endif //MAP_MATCHING_2_GEOMETRY_CONCEPTS_HPP
