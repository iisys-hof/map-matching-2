// Copyright (C) 2024-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_RICH_TYPE_RICH_LINE_VARIANT_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_RICH_TYPE_RICH_LINE_VARIANT_HPP

#include <variant>

#include "types/extern_define.hpp"

#include "rich_line.hpp"
#include "eager_rich_line.hpp"
#include "lazy_rich_line.hpp"

#define MM2_RICH_LINE_CS(CS) \
    MM2_RICH_LINE(CS, MM2_MEMORY_TYPES), \
    MM2_RICH_LINE(CS, MM2_MMAP_TYPES)

#define MM2_EAGER_RICH_LINE_CS(CS) \
    MM2_EAGER_RICH_LINE(CS, MM2_MEMORY_TYPES), \
    MM2_EAGER_RICH_LINE(CS, MM2_MMAP_TYPES)

#define MM2_LAZY_RICH_LINE_CS(CS) \
    MM2_LAZY_RICH_LINE(CS, MM2_MEMORY_TYPES), \
    MM2_LAZY_RICH_LINE(CS, MM2_MMAP_TYPES)

#define MM2_RICH_LINE_VARIANT(CS) std::variant< \
    MM2_RICH_LINE_CS(CS), \
    MM2_EAGER_RICH_LINE_CS(CS), \
    MM2_LAZY_RICH_LINE_CS(CS)>

#ifdef EXPLICIT_TEMPLATES

#define MM2_RICH_LINE_VARIANT_TEMPLATE(CS) \
    MM2_EXTERN template class MM2_RICH_LINE_VARIANT(CS);

MM2_RICH_LINE_VARIANT_TEMPLATE(MM2_GEOGRAPHIC)
MM2_RICH_LINE_VARIANT_TEMPLATE(MM2_SPHERICAL_EQUATORIAL)
MM2_RICH_LINE_VARIANT_TEMPLATE(MM2_CARTESIAN)

#endif

namespace map_matching_2::geometry {

    template<typename CoordinateSystem>
    using rich_line_variant = MM2_RICH_LINE_VARIANT(CoordinateSystem);

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_RICH_TYPE_RICH_LINE_VARIANT_HPP
