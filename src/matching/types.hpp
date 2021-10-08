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

#ifndef MAP_MATCHING_2_MATCHING_TYPES_HPP
#define MAP_MATCHING_2_MATCHING_TYPES_HPP

#include <geometry/track/types.hpp>
#include <geometry/network/types.hpp>

#include "matcher.hpp"

namespace map_matching_2::matching {

    template<typename NetworkTypes, typename Track>
    struct types {
        using matcher_static = matcher<typename NetworkTypes::network_static, Track>;
    };

    using types_geographic = types<geometry::network::types_geographic, geometry::track::types_geographic::track_type>;
    using types_cartesian = types<geometry::network::types_cartesian, geometry::track::types_cartesian::track_type>;

}

#endif //MAP_MATCHING_2_MATCHING_TYPES_HPP
