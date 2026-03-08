// Copyright (C) 2026 Adrian Wöltche
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

#include "util/math.hpp"

#include <cmath>
#include <algorithm>

namespace map_matching_2::util {

    double gaussian_weight(const std::size_t curr, std::size_t center, const std::size_t size,
            const double min_weight, const double max_weight, const double sigma) {
        if (size == 0) {
            return min_weight;
        }
        center = std::min(center, size - 1);
        const double distance = static_cast<double>(curr) - static_cast<double>(center);
        return min_weight + (max_weight - min_weight) * std::exp(-(distance * distance) / (2.0 * sigma * sigma));
    }

}
