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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_TRACK_TRACK_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_TRACK_TRACK_HPP

#include <variant>

#include "types/extern_define.hpp"

#include "types/geometry/rich_type/eager_rich_line.hpp"

#include "geometry/track/track.hpp"

#define MM2_IMPORT_TRACK(CS) map_matching_2::geometry::track::track<MM2_RICH_TIME_LINE(CS, MM2_MEMORY_TYPES)>

#define MM2_TRACK(CS) map_matching_2::geometry::track::track<MM2_EAGER_RICH_TIME_LINE(CS, MM2_MEMORY_TYPES)>

#define MM2_IMPORT_TRACK_VARIANT std::variant< \
    MM2_IMPORT_TRACK(MM2_GEOGRAPHIC), \
    MM2_IMPORT_TRACK(MM2_SPHERICAL_EQUATORIAL), \
    MM2_IMPORT_TRACK(MM2_CARTESIAN)>

#define MM2_TRACK_VARIANT std::variant< \
    MM2_TRACK(MM2_GEOGRAPHIC), \
    MM2_TRACK(MM2_SPHERICAL_EQUATORIAL), \
    MM2_TRACK(MM2_CARTESIAN)>

#ifdef EXPLICIT_TEMPLATES

#define MM2_TRACK_TEMPLATE(CS) \
    MM2_EXTERN template class MM2_IMPORT_TRACK(CS); \
    MM2_EXTERN template class MM2_TRACK(CS);

MM2_TRACK_TEMPLATE(MM2_GEOGRAPHIC)
MM2_TRACK_TEMPLATE(MM2_SPHERICAL_EQUATORIAL)
MM2_TRACK_TEMPLATE(MM2_CARTESIAN)

#endif

namespace map_matching_2::geometry::track {

    template<geometry::is_time_point TimePoint>
    using import_track_type = track<rich_line_type<TimePoint>>;

    template<geometry::is_time_point TimePoint>
    using track_type = track<eager_rich_line_type<TimePoint>>;

    using import_track_variant_type = MM2_IMPORT_TRACK_VARIANT;

    using track_variant_type = MM2_TRACK_VARIANT;

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_TRACK_TRACK_HPP
