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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_NETWORK_ROUTE_EAGER_ROUTE_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_NETWORK_ROUTE_EAGER_ROUTE_HPP

#include "types/extern_define.hpp"

#include "types/geometry/rich_type/rich_line_variant.hpp"

#include "route_data.hpp"
#include "route.hpp"

#include "geometry/network/route/eager_route.hpp"

#define MM2_EAGER_ROUTE(CS) map_matching_2::geometry::network::eager_route<MM2_RICH_LINE_VARIANT(CS)>

#ifdef EXPLICIT_TEMPLATES

#define MM2_EAGER_ROUTE_TEMPLATE(CS) \
    MM2_EXTERN template class MM2_EAGER_ROUTE(CS);

MM2_EAGER_ROUTE_TEMPLATE(MM2_GEOGRAPHIC)
MM2_EAGER_ROUTE_TEMPLATE(MM2_SPHERICAL_EQUATORIAL)
MM2_EAGER_ROUTE_TEMPLATE(MM2_CARTESIAN)

#endif

namespace map_matching_2::geometry::network {

    template<typename CoordinateSystem>
    using eager_route_type = eager_route<rich_line_variant<CoordinateSystem>>;

}

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_NETWORK_ROUTE_EAGER_ROUTE_HPP
