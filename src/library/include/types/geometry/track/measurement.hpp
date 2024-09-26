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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_TRACK_MEASUREMENT_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_TRACK_MEASUREMENT_HPP

#include "types/extern_define.hpp"

#include "types/geometry/point.hpp"

#include "geometry/track/measurement.hpp"

#define MM2_MEASUREMENT(CS) map_matching_2::geometry::track::measurement<MM2_POINT(CS)>

#ifdef EXPLICIT_TEMPLATES

#define MM2_MEASUREMENT_TEMPLATE(CS) \
    MM2_EXTERN template class MM2_MEASUREMENT(CS);

MM2_MEASUREMENT_TEMPLATE(MM2_GEOGRAPHIC)
MM2_MEASUREMENT_TEMPLATE(MM2_SPHERICAL_EQUATORIAL)
MM2_MEASUREMENT_TEMPLATE(MM2_CARTESIAN)

#endif

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_TRACK_MEASUREMENT_HPP
