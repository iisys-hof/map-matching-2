// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_TYPES_HPP
#define MAP_MATCHING_2_GEOMETRY_TYPES_HPP

#include "point.hpp"
#include "models.hpp"

namespace map_matching_2::geometry {

    template<is_point Point>
    struct models {
        using point_type = Point;
        template<template<typename, typename> typename Container = std::vector,
            template<typename> typename Allocator = std::allocator>
        using multi_point_type = boost::geometry::model::multi_point<point_type, Container, Allocator>;
        template<template<typename, typename> typename Container = std::vector,
            template<typename> typename Allocator = std::allocator>
        using line_type = linestring<point_type, Container, Allocator>;
        template<typename Line,
            template<typename, typename> typename Container = std::vector,
            template<typename> typename Allocator = std::allocator>
        using multi_line_type = multi_linestring<Line, Container, Allocator>;
        using segment_type = boost::geometry::model::segment<point_type>;
        using referring_segment_type = boost::geometry::model::referring_segment<const point_type>;
        using pointing_segment_type = boost::geometry::model::pointing_segment<const point_type>;
        using box_type = boost::geometry::model::box<point_type>;
        template<bool ClockWise = true,
            bool Closed = true,
            template<typename, typename> typename PointContainer = std::vector,
            template<typename, typename> typename RingContainer = std::vector,
            template<typename> typename PointAllocator = std::allocator,
            template<typename> typename RingAllocator = std::allocator>
        using polygon_type = boost::geometry::model::polygon<
            point_type, ClockWise, Closed, PointContainer, RingContainer, PointAllocator, RingAllocator>;
        template<typename Polygon = polygon_type<>,
            template<typename, typename> typename Container = std::vector,
            template<typename> typename Allocator = std::allocator>
        using multi_polygon_type = boost::geometry::model::multi_polygon<Polygon, Container, Allocator>;
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_TYPES_HPP
