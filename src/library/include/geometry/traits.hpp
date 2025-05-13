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

#ifndef MAP_MATCHING_2_GEOMETRY_TRAITS_HPP
#define MAP_MATCHING_2_GEOMETRY_TRAITS_HPP

#include <boost/geometry.hpp>

namespace map_matching_2::geometry {

    template<typename Geometry>
    struct data {
        using geometry_type = Geometry;
        using coordinate_system_type = typename boost::geometry::coordinate_system<geometry_type>::type;
        using coordinate_type = typename boost::geometry::coordinate_type<geometry_type>::type;
        using distance_type = typename boost::geometry::default_distance_result<geometry_type>::type;
        // using length_type = typename boost::geometry::default_length_result<geometry_type>::type;
        // default_length_result is by default "long double", but "double" from default_distance_result is sufficient
        using length_type = distance_type;
        using angle_type = coordinate_type; // using the same resolution as the coordinate type, i.e., "double"
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_TRAITS_HPP
