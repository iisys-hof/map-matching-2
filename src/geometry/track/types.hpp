// Copyright (C) 2020-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TRACK_TYPES_HPP
#define MAP_MATCHING_2_TRACK_TYPES_HPP

#include "../types.hpp"
#include "measurement.hpp"
#include "multi_track.hpp"
#include "track.hpp"

namespace map_matching_2::geometry::track {

    template<typename CoordinateSystem>
    struct types {
        using measurement_type = measurement<typename geometry::types<CoordinateSystem>::point_type>;
        using track_type = geometry::track::track_type<typename geometry::types<CoordinateSystem>::point_type>;
        using eager_track_type = geometry::track::eager_track_type<typename geometry::types<CoordinateSystem>::point_type>;
        using lazy_track_type = geometry::track::lazy_track_type<typename geometry::types<CoordinateSystem>::point_type>;
        using multi_track_type = geometry::track::multi_track_type<typename geometry::types<CoordinateSystem>::point_type>;
        using eager_multi_track_type = geometry::track::eager_multi_track_type<typename geometry::types<CoordinateSystem>::point_type>;
        using lazy_multi_track_type = geometry::track::lazy_multi_track_type<typename geometry::types<CoordinateSystem>::point_type>;
    };

    using types_geographic = types<geometry::cs_geographic>;
    using types_spherical_equatorial = types<geometry::cs_spherical_equatorial>;
    using types_cartesian = types<geometry::cs_cartesian>;

}

#endif //MAP_MATCHING_2_TRACK_TYPES_HPP
