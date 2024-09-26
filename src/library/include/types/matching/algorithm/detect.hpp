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

#ifndef MAP_MATCHING_2_TYPES_MATCHING_ALGORITHM_DETECT_HPP
#define MAP_MATCHING_2_TYPES_MATCHING_ALGORITHM_DETECT_HPP

#include "types/extern_define.hpp"

#include "types/geometry/rich_type/eager_rich_line.hpp"

#include "geometry/rich_type/traits/rich_line.hpp"

#include "matching/algorithm/detect.hpp"

#include "util/macros.hpp"

#define MM2_DETECTOR(RICH_LINE_TYPE) map_matching_2::matching::detector<RICH_LINE_TYPE>

#ifdef EXPLICIT_TEMPLATES

#define MM2_DETECTOR_TEMPLATE(RICH_LINE_TYPE) \
    MM2_EXTERN template class MM2_DETECTOR(UNPACK RICH_LINE_TYPE);

#define MM2_DETECTOR_TEMPLATE_CS_TYPES(CS, TYPES) \
    MM2_DETECTOR_TEMPLATE((MM2_EAGER_RICH_LINE(CS, TYPES)))

#define MM2_DETECTOR_TEMPLATE_CS(CS) \
    MM2_DETECTOR_TEMPLATE_CS_TYPES(CS, MM2_MEMORY_TYPES) \
    MM2_DETECTOR_TEMPLATE_CS_TYPES(CS, MM2_MMAP_TYPES)

MM2_DETECTOR_TEMPLATE_CS(MM2_GEOGRAPHIC)
MM2_DETECTOR_TEMPLATE_CS(MM2_SPHERICAL_EQUATORIAL)
MM2_DETECTOR_TEMPLATE_CS(MM2_CARTESIAN)

#endif

namespace map_matching_2::matching {

    template<geometry::is_rich_line RichLine>
    using detector_type = detector<RichLine>;

}

#endif //MAP_MATCHING_2_TYPES_MATCHING_ALGORITHM_DETECT_HPP
