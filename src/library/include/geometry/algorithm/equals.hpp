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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_EQUALS_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_EQUALS_HPP

#include "util/concepts.hpp"

#include "geometry/concepts.hpp"
#include "geometry/traits.hpp"

namespace map_matching_2::geometry {

    template<typename GeometryA, typename GeometryB>
    [[nodiscard]] bool equals(const GeometryA &a, const GeometryB &b) {
        return boost::geometry::equals(a, b);
    }

    template<typename Point>
    [[nodiscard]] constexpr bool equals_points(const Point &a, const Point &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));

        if constexpr (boost::geometry::dimension<Point>::value == 1) {
            return boost::geometry::get<0>(a) == boost::geometry::get<0>(b);
        } else if constexpr (boost::geometry::dimension<Point>::value == 2) {
            return boost::geometry::get<0>(a) == boost::geometry::get<0>(b) and
                    boost::geometry::get<1>(a) == boost::geometry::get<1>(b);
        } else {
            return boost::geometry::get<0>(a) == boost::geometry::get<0>(b) and
                    boost::geometry::get<1>(a) == boost::geometry::get<1>(b) and
                    boost::geometry::get<2>(a) == boost::geometry::get<2>(b);
        }
    }

    template<typename Segment>
    [[nodiscard]] typename data<Segment>::length_type
    equals_points_segment(const Segment &segment) {
        if constexpr (geometry::is_segment<Segment> or geometry::is_referring_segment<Segment>) {
            return geometry::equals_points(segment.first, segment.second);
        } else if constexpr (geometry::is_pointing_segment<Segment>) {
            return geometry::equals_points(*(segment.first), *(segment.second));
        } else {
            static_assert(util::dependent_false_v<Segment>, "invalid segment type");
            throw std::invalid_argument{"invalid segment type"};
        }
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_EQUALS_HPP
