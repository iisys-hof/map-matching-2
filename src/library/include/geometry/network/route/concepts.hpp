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

#ifndef MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_CONCEPTS_HPP
#define MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_CONCEPTS_HPP

namespace map_matching_2::geometry::network {

    template<typename Route>
    concept is_route = requires {
        typename Route::rich_line_variant_type;
        typename Route::rich_line_reference_variant_type;
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_CONCEPTS_HPP
