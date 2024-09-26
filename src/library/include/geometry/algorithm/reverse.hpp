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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_REVERSE_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_REVERSE_HPP

#include "equals.hpp"

namespace map_matching_2::geometry {

    template<typename Line>
    [[nodiscard]] bool lines_are_reversed(const Line &a, const Line &b) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));

        if (a.empty() or b.empty()) {
            // when one line is empty, they cannot be reverse to each other
            return false;
        }

        if (a.size() != b.size()) {
            // if sizes are not equal, they cannot be reverse to each other
            return false;
        }

        if (not(equals_points(a.front(), b.back()) and equals_points(a.back(), b.front()))) {
            // back and front are not equal, we can already quit
            return false;
        }

        // check the rest of the lines when front and back were equal
        for (std::size_t i = 1, j = b.size() - 2; i < a.size(); ++i, --j) {
            if (not equals_points(a[i], b[j])) {
                // if only one point pair is not equal, they cannot be reverse
                return false;
            }
        }

        // all points were equal concerning reverse lines
        return true;
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_REVERSE_HPP
