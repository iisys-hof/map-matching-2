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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_LENGTH_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_LENGTH_HPP

#include "util/concepts.hpp"

#include "distance.hpp"
#include "geometry/traits.hpp"

namespace map_matching_2::geometry {

    template<typename Geometry>
    [[nodiscard]] typename data<Geometry>::length_type
    length(const Geometry &geometry) {
        using coordinate_system_type = typename boost::geometry::coordinate_system<Geometry>::type;
        if constexpr (std::same_as<coordinate_system_type, cs_spherical_equatorial>) {
            return boost::geometry::length(geometry,
                    boost::geometry::strategy::distance::haversine(EARTH_RADIUS_METER));
        } else {
            return boost::geometry::length(geometry);
        }
    }

    template<typename Segment>
    [[nodiscard]] typename data<Segment>::length_type
    length_segment(const Segment &segment) {
        if constexpr (geometry::is_segment<Segment> or geometry::is_referring_segment<Segment>) {
            return geometry::point_distance(segment.first, segment.second);
        } else if constexpr (geometry::is_pointing_segment<Segment>) {
            return geometry::point_distance(*(segment.first), *(segment.second));
        } else {
            static_assert(util::dependent_false_v<Segment>, "invalid segment type");
            throw std::invalid_argument{"invalid segment type"};
        }
    }

    template<template<typename, typename> typename Container,
        typename Segment,
        template<typename> typename Allocator>
    [[nodiscard]] Container<typename data<Segment>::length_type,
        Allocator<typename data<Segment>::length_type>>
    segments_lengths(const Container<Segment, Allocator<Segment>> &segments) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Segment<Segment>));

        using length_type = typename data<Segment>::length_type;
        using allocator_type = Allocator<length_type>;

        Container<length_type, allocator_type> lengths;
        if constexpr (util::has_reserve<Container<length_type, allocator_type>>) {
            lengths.reserve(segments.size());
        }
        for (const auto &segment : segments) {
            lengths.emplace_back(geometry::length_segment(segment));
        }
        return lengths;
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_LENGTH_HPP
