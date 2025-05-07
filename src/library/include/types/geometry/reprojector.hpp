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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_REPROJECTOR_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_REPROJECTOR_HPP

#include <variant>

#include "types/extern_define.hpp"

#include "geometry/reprojector.hpp"

#include "point.hpp"

#define MM2_POINT_REPROJECTOR(CS_A, CS_B) \
    map_matching_2::geometry::point_reprojector<MM2_POINT(CS_A), MM2_POINT(CS_B)>

#define MM2_TIME_POINT_REPROJECTOR(CS_A, CS_B) \
    map_matching_2::geometry::point_reprojector<MM2_TIME_POINT(CS_A), MM2_TIME_POINT(CS_B)>

#define MM2_POINT_REPROJECTOR_CS(CS) \
    MM2_POINT_REPROJECTOR(CS, MM2_GEOGRAPHIC), \
    MM2_POINT_REPROJECTOR(CS, MM2_SPHERICAL_EQUATORIAL), \
    MM2_POINT_REPROJECTOR(CS, MM2_CARTESIAN)

#define MM2_TIME_POINT_REPROJECTOR_CS(CS) \
    MM2_TIME_POINT_REPROJECTOR(CS, MM2_GEOGRAPHIC), \
    MM2_TIME_POINT_REPROJECTOR(CS, MM2_SPHERICAL_EQUATORIAL), \
    MM2_TIME_POINT_REPROJECTOR(CS, MM2_CARTESIAN)

#define MM2_POINT_REPROJECTOR_VARIANT std::variant< \
    MM2_POINT_REPROJECTOR_CS(MM2_GEOGRAPHIC), \
    MM2_POINT_REPROJECTOR_CS(MM2_SPHERICAL_EQUATORIAL), \
    MM2_POINT_REPROJECTOR_CS(MM2_CARTESIAN), \
    MM2_TIME_POINT_REPROJECTOR_CS(MM2_GEOGRAPHIC), \
    MM2_TIME_POINT_REPROJECTOR_CS(MM2_SPHERICAL_EQUATORIAL), \
    MM2_TIME_POINT_REPROJECTOR_CS(MM2_CARTESIAN)>

#ifdef EXPLICIT_TEMPLATES

#define MM2_POINT_REPROJECTOR_TEMPLATE(CS_A, CS_B) \
    MM2_EXTERN template class MM2_POINT_REPROJECTOR(CS_A, CS_B); \
    MM2_EXTERN template class MM2_TIME_POINT_REPROJECTOR(CS_A, CS_B);

#define MM2_POINT_REPROJECTOR_TEMPLATE_CS(CS) \
    MM2_POINT_REPROJECTOR_TEMPLATE(CS, MM2_GEOGRAPHIC) \
    MM2_POINT_REPROJECTOR_TEMPLATE(CS, MM2_SPHERICAL_EQUATORIAL) \
    MM2_POINT_REPROJECTOR_TEMPLATE(CS, MM2_CARTESIAN)

MM2_POINT_REPROJECTOR_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_POINT_REPROJECTOR_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_POINT_REPROJECTOR_TEMPLATE_CS(MM2_CARTESIAN)

#define MM2_POINT_REPROJECTOR_VARIANT_TEMPLATE \
    MM2_EXTERN template class MM2_POINT_REPROJECTOR_VARIANT;

MM2_POINT_REPROJECTOR_VARIANT_TEMPLATE

#endif

namespace map_matching_2::geometry {

    template<is_point PointA, is_point PointB>
    using point_reprojector_type = point_reprojector<PointA, PointB>;

    using point_reprojector_variant = MM2_POINT_REPROJECTOR_VARIANT;

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_REPROJECTOR_HPP
