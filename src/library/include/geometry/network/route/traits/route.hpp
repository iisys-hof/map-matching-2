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

#ifndef MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_TRAITS_ROUTE_HPP
#define MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_TRAITS_ROUTE_HPP

#include "../concepts.hpp"

#include "geometry/rich_type/traits/rich_line.hpp"

namespace map_matching_2::geometry::network {

    template<is_route Route>
    struct route_traits {
        using route_type = Route;
        using rich_line_type = typename route_type::rich_line_type;
        using rich_segment_type = typename rich_line_traits<rich_line_type>::rich_segment_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        template<template<typename, typename> class Container = std::vector,
            template<typename> class Allocator = std::allocator>
        using line_type = typename rich_line_traits<rich_line_type>::template line_type<Container, Allocator>;
        template<template<typename, typename> class PointContainer = std::vector,
            template<typename, typename> class LineContainer = std::vector,
            template<typename> class PointAllocator = std::allocator,
            template<typename> class LineAllocator = std::allocator>
        using multi_line_type = typename rich_line_traits<rich_line_type>::template multi_line_type<
            PointContainer, LineContainer, PointAllocator, LineAllocator>;
        using coordinate_system_type = typename rich_line_traits<rich_line_type>::coordinate_system_type;
        using coordinate_type = typename rich_line_traits<rich_line_type>::coordinate_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_type>::angle_type;
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_TRAITS_ROUTE_HPP
