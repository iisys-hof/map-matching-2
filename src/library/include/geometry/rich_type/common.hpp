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

#ifndef MAP_MATCHING_2_RICH_TYPE_COMMON_HPP
#define MAP_MATCHING_2_RICH_TYPE_COMMON_HPP

#include <cmath>
#include <limits>
#include <concepts>

namespace map_matching_2::geometry {

    template<std::floating_point Float> requires std::numeric_limits<Float>::has_signaling_NaN
    static constexpr Float not_initialized = std::numeric_limits<Float>::signaling_NaN();

    template<std::floating_point Float> requires std::numeric_limits<Float>::has_infinity
    static constexpr Float not_valid = std::numeric_limits<Float>::infinity();

    template<std::floating_point Float>
    [[nodiscard]] constexpr bool initialized(Float data) {
        return not std::isnan(data);
    }

    template<std::floating_point Float>
    [[nodiscard]] constexpr bool valid(Float data) {
        return initialized(data) and not std::isinf(data);
    }

}

#endif //MAP_MATCHING_2_RICH_TYPE_COMMON_HPP
