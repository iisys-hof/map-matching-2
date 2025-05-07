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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_POINT_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_POINT_HPP

#include "types/extern_define.hpp"

#include "geometry/point.hpp"

#define MM2_GEOGRAPHIC boost::geometry::cs::geographic<boost::geometry::degree>
#define MM2_SPHERICAL_EQUATORIAL boost::geometry::cs::spherical_equatorial<boost::geometry::degree>
#define MM2_CARTESIAN boost::geometry::cs::cartesian

#define MM2_POINT(CS) boost::geometry::model::point<double, 2, CS>
#define MM2_TIME_POINT(CS) map_matching_2::geometry::time_point<double, 2, CS>

#ifdef EXPLICIT_TEMPLATES

#define MM2_POINT_TEMPLATE(CS) \
    MM2_EXTERN template class MM2_POINT(CS);

MM2_POINT_TEMPLATE(MM2_GEOGRAPHIC)
MM2_POINT_TEMPLATE(MM2_SPHERICAL_EQUATORIAL)
MM2_POINT_TEMPLATE(MM2_CARTESIAN)

#define MM2_TIME_POINT_TEMPLATE(CS) \
    MM2_EXTERN template class MM2_TIME_POINT(CS);

MM2_TIME_POINT_TEMPLATE(MM2_GEOGRAPHIC)
MM2_TIME_POINT_TEMPLATE(MM2_SPHERICAL_EQUATORIAL)
MM2_TIME_POINT_TEMPLATE(MM2_CARTESIAN)

#endif

namespace map_matching_2::geometry {

    using cs_geographic = MM2_GEOGRAPHIC;
    using cs_spherical_equatorial = MM2_SPHERICAL_EQUATORIAL;
    using cs_cartesian = MM2_CARTESIAN;

    template<typename CoordinateSystem>
    using point_type = MM2_POINT(CoordinateSystem);

    template<typename CoordinateSystem>
    using time_point_type = MM2_TIME_POINT(CoordinateSystem);

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_POINT_HPP
