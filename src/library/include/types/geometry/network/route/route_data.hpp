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

#ifndef MAP_MATCHING_2_TYPES_GEOMETRY_NETWORK_ROUTE_ROUTE_DATA_HPP
#define MAP_MATCHING_2_TYPES_GEOMETRY_NETWORK_ROUTE_ROUTE_DATA_HPP

#include "types/extern_define.hpp"

#include "geometry/network/route/route_data.hpp"

#define MM2_ROUTE_DATA(LENGTH, ANGLE) map_matching_2::geometry::network::route_data<LENGTH, ANGLE>

#ifdef EXPLICIT_TEMPLATES

#define MM2_ROUTE_DATA_TEMPLATE(LENGTH, ANGLE) \
    MM2_EXTERN template class MM2_ROUTE_DATA(LENGTH, ANGLE);

#define MM2_ROUTE_DATA_TEMPLATE_LENGTH(LENGTH) \
    MM2_ROUTE_DATA_TEMPLATE(LENGTH, float) \
    MM2_ROUTE_DATA_TEMPLATE(LENGTH, double) \
    MM2_ROUTE_DATA_TEMPLATE(LENGTH, long double)

MM2_ROUTE_DATA_TEMPLATE_LENGTH(float)
MM2_ROUTE_DATA_TEMPLATE_LENGTH(double)
MM2_ROUTE_DATA_TEMPLATE_LENGTH(long double)

#endif

#endif //MAP_MATCHING_2_TYPES_GEOMETRY_NETWORK_ROUTE_ROUTE_DATA_HPP
